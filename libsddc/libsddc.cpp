#include "libsddc.h"
#include "config.h"
#include "r2iq.h"
#include "RadioHandler.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <vector>

static const int SDDC_MAX_STEPS = 128;
static const size_t SDDC_MAX_SYNC_BUFFER = 16 * 1024 * 1024;

struct sddc
{
    SDDCStatus status;
    RadioHandlerClass* handler;
    uint8_t led;
    int samplerateidx;
    double sample_rate;
    double freq;
    double frequency_range[2];

    int rf_attenuation_idx;
    int if_attenuation_idx;
    double rf_attenuations[SDDC_MAX_STEPS];
    double if_attenuations[SDDC_MAX_STEPS];

    sddc_read_async_cb_t callback;
    void *callback_context;
    uint32_t frame_size;
    uint32_t num_frames;

    std::mutex sync_mutex;
    std::condition_variable sync_cv;
    std::vector<uint8_t> sync_buffer;
};

sddc_t *current_running;
static bool all_non_positive(const float* steps, int count)
{
    if (steps == nullptr || count <= 0) return false;

    for (int i = 0; i < count; i++)
    {
        if (steps[i] > 0.0f) return false;
    }

    return true;
}

static double step_to_api_value(const float* steps, int count, int index)
{
    if (steps == nullptr || count <= 0 || index < 0)
        return 0.0;

    if (index >= count)
        index = count - 1;

    double value = (double)steps[index];
    return all_non_positive(steps, count) ? -value : value;
}

static int copy_steps_to_double(const float* steps, int count, double* destination)
{
    if (steps == nullptr || destination == nullptr || count <= 0)
        return 0;

    int copied = std::min(count, SDDC_MAX_STEPS);
    bool convertToPositive = all_non_positive(steps, count);
    for (int i = 0; i < copied; i++)
    {
        destination[i] = convertToPositive ? -(double)steps[i] : (double)steps[i];
    }

    return copied;
}

static int nearest_step_index(double attenuation, const float* steps, int count)
{
    if (steps == nullptr || count <= 0) return -1;

    bool allNonPositive = all_non_positive(steps, count);
    double target = (allNonPositive && attenuation >= 0.0) ? -attenuation : attenuation;

    int best = 0;
    double bestErr = fabs((double)steps[0] - target);
    for (int i = 1; i < count; i++)
    {
        double err = fabs((double)steps[i] - target);
        if (err < bestErr) { bestErr = err; best = i; }
    }
    return best;
}
static void Callback(void* context, const float* data, uint32_t len)
{
    sddc_t *running = current_running;
    if (running == nullptr || data == nullptr || len == 0)
        return;

    uint32_t data_size = len * 2 * sizeof(float);
    uint8_t* bytes = (uint8_t*)data;

    if (running->callback != nullptr)
    {
        running->callback(data_size, bytes, running->callback_context);
        return;
    }

    {
        std::unique_lock<std::mutex> lk(running->sync_mutex);
        if (running->sync_buffer.size() + data_size > SDDC_MAX_SYNC_BUFFER)
        {
            size_t overflow = running->sync_buffer.size() + data_size - SDDC_MAX_SYNC_BUFFER;
            if (overflow >= running->sync_buffer.size())
                running->sync_buffer.clear();
            else
                running->sync_buffer.erase(running->sync_buffer.begin(), running->sync_buffer.begin() + overflow);
        }
        running->sync_buffer.insert(running->sync_buffer.end(), bytes, bytes + data_size);
    }
    running->sync_cv.notify_all();
}

class rawdata : public r2iqControlClass {
    void Init(float gain, ringbuffer<int16_t>* buffers, ringbuffer<float>* obuffers) override
    {
        idx = 0;
    }

    void TurnOn() override
    {
        this->r2iqOn = true;
        idx = 0;
    }

private:
    int idx;
};

int sddc_get_device_count()
{
    return 1;
}

int sddc_get_device_info(struct sddc_device_info **sddc_device_infos)
{
    if (sddc_device_infos == nullptr)
        return -1;

    int count = sddc_get_device_count();
    if (count <= 0)
    {
        *sddc_device_infos = nullptr;
        return count;
    }

    auto ret = new sddc_device_info[count]();
    fx3class *fx3 = CreateUsbHandler();
    if (fx3 == nullptr)
    {
        delete[] ret;
        *sddc_device_infos = nullptr;
        return -1;
    }

    int found = 0;
    for (int i = 0; i < count; i++)
    {
        unsigned char idx = (unsigned char)i;
        char device[256] = { 0 };

        if (!fx3->Enumerate(idx, device))
            continue;

        const char *serialPrefix = strstr(device, "sn:");
        size_t productLen = serialPrefix != nullptr ? (size_t)(serialPrefix - device) : strlen(device);
        while (productLen > 0 && device[productLen - 1] == ' ')
            productLen--;

        char *manufacturer = new char[5];
        strcpy(manufacturer, "SDDC");

        char *product = new char[productLen + 1];
        memcpy(product, device, productLen);
        product[productLen] = '\0';

        const char *serialText = serialPrefix != nullptr ? serialPrefix + 3 : "";
        char *serial = new char[strlen(serialText) + 1];
        strcpy(serial, serialText);

        ret[found].manufacturer = manufacturer;
        ret[found].product = product;
        ret[found].serial_number = serial;
        found++;
    }

    delete fx3;

    if (found == 0)
    {
        delete[] ret;
        *sddc_device_infos = nullptr;
        return 0;
    }

    *sddc_device_infos = ret;

    return found;
}

int sddc_free_device_info(struct sddc_device_info *sddc_device_infos)
{
    if (sddc_device_infos == nullptr)
        return 0;

    int count = sddc_get_device_count();
    for (int i = 0; i < count; i++)
    {
        delete[] sddc_device_infos[i].manufacturer;
        delete[] sddc_device_infos[i].product;
        delete[] sddc_device_infos[i].serial_number;
    }
    delete[] sddc_device_infos;
    return 0;
}

sddc_t *sddc_open(int index, const char* imagefile)
{

    auto ret_val = new sddc_t();
    if (ret_val == nullptr)
    {        return nullptr;
    }

    fx3class *fx3 = CreateUsbHandler();
    if (fx3 == nullptr)
    {        delete ret_val;
        return nullptr;
    }

    unsigned char* res_data = nullptr;
    uint32_t res_size = 0;

    FILE *fp = fopen(imagefile, "rb");
    if (fp == nullptr)
    {        delete fx3;
        delete ret_val;
        return nullptr;
    }

    fseek(fp, 0, SEEK_END);
    long file_size = ftell(fp);
    if (file_size <= 0)
    {        fclose(fp);
        delete fx3;
        delete ret_val;
        return nullptr;
    }
    res_size = (uint32_t)file_size;

    res_data = (unsigned char*)malloc(res_size);
    if (res_data == nullptr)
    {        fclose(fp);
        delete fx3;
        delete ret_val;
        return nullptr;
    }

    fseek(fp, 0, SEEK_SET);
    size_t read_sz = fread(res_data, 1, res_size, fp);
    fclose(fp);
    if (read_sz != res_size)
    {        free(res_data);
        delete fx3;
        delete ret_val;
        return nullptr;
    }
    free(res_data);

    bool openOK = fx3->Open();
    if (!openOK)
    {        delete fx3;
        delete ret_val;
        return nullptr;
    }

    ret_val->handler = new RadioHandlerClass();
    if (ret_val->handler == nullptr)
    {        delete fx3;
        delete ret_val;
        return nullptr;
    }

    bool initOK = ret_val->handler->Init(fx3, Callback, nullptr);
    if (!initOK)
    {        delete ret_val->handler;
        ret_val->handler = nullptr;
        delete fx3;
        delete ret_val;
        return nullptr;
    }

    ret_val->status = SDDC_STATUS_READY;
    ret_val->samplerateidx = 0;
    ret_val->sample_rate = 2000000.0;
    ret_val->freq = 0.0;
    ret_val->led = 0;
    ret_val->rf_attenuation_idx = 0;
    ret_val->if_attenuation_idx = 0;
    ret_val->callback = nullptr;
    ret_val->callback_context = nullptr;
    ret_val->frame_size = 0;
    ret_val->num_frames = 0;

    switch (ret_val->handler->getModel())
    {
    case RadioModel::HF103:
        ret_val->frequency_range[0] = 0.0;
        ret_val->frequency_range[1] = 32000000.0;
        break;
    case RadioModel::RX888r3:
        ret_val->frequency_range[0] = 10000.0;
        ret_val->frequency_range[1] = 2150000000.0;
        break;
    case RadioModel::RX999:
        ret_val->frequency_range[0] = 10000.0;
        ret_val->frequency_range[1] = 6000000000.0;
        break;
    case RadioModel::BBRF103:
    case RadioModel::RX888:
    case RadioModel::RX888r2:
        ret_val->frequency_range[0] = 10000.0;
        ret_val->frequency_range[1] = 1750000000.0;
        break;
    default:
        ret_val->frequency_range[0] = 0.0;
        ret_val->frequency_range[1] = 0.0;
        break;
    }

    return ret_val;
}
void sddc_close(sddc_t *that)
{
    if (that == nullptr)
        return;

    if (current_running == that)
        current_running = nullptr;

    if (that->handler)
        delete that->handler;
    delete that;
}

enum SDDCStatus sddc_get_status(sddc_t *t)
{
    return t->status;
}

enum SDDCHWModel sddc_get_hw_model(sddc_t *t)
{
    switch(t->handler->getModel())
    {
        case RadioModel::BBRF103:
            return HW_BBRF103;
        case RadioModel::HF103:
            return HW_HF103;
        case RadioModel::RX888:
            return HW_RX888;
        case RadioModel::RX888r2:
            return HW_RX888R2;
        case RadioModel::RX888r3:
            return HW_RX888R3;
        case RadioModel::RX999:
            return HW_RX999;
        default:
            return HW_NORADIO;
    }
}

const char *sddc_get_hw_model_name(sddc_t *t)
{
    return t->handler->getName();
}

uint16_t sddc_get_firmware(sddc_t *t)
{
    return t->handler->GetFirmware();
}

const double *sddc_get_frequency_range(sddc_t *t)
{
    if (t == nullptr)
        return nullptr;

    return t->frequency_range;
}

enum RFMode sddc_get_rf_mode(sddc_t *t)
{
    switch(t->handler->GetmodeRF())
    {
        case HFMODE:
            return RFMode::HF_MODE;
        case VHFMODE:
            return RFMode::VHF_MODE;
        default:
            return RFMode::NO_RF_MODE;
    }
}

int sddc_set_rf_mode(sddc_t *t, enum RFMode rf_mode)
{
    switch (rf_mode)
    {
    case VHF_MODE:
        t->handler->UpdatemodeRF(VHFMODE);
        break;
    case HF_MODE:
        t->handler->UpdatemodeRF(HFMODE);
        break;
    default:
        return -1;
    }

    return 0;
}

/* LED functions */
int sddc_led_on(sddc_t *t, uint8_t led_pattern)
{
    if (led_pattern & YELLOW_LED)
        t->handler->uptLed(0, true);
    if (led_pattern & RED_LED)
        t->handler->uptLed(1, true);
    if (led_pattern & BLUE_LED)
        t->handler->uptLed(2, true);

    t->led |= led_pattern;

    return 0;
}

int sddc_led_off(sddc_t *t, uint8_t led_pattern)
{
    if (led_pattern & YELLOW_LED)
        t->handler->uptLed(0, false);
    if (led_pattern & RED_LED)
        t->handler->uptLed(1, false);
    if (led_pattern & BLUE_LED)
        t->handler->uptLed(2, false);

    t->led &= ~led_pattern;

    return 0;
}

int sddc_led_toggle(sddc_t *t, uint8_t led_pattern)
{
    t->led = t->led ^ led_pattern;
    if (t->led & YELLOW_LED)
        t->handler->uptLed(0, false);
    if (t->led & RED_LED)
        t->handler->uptLed(1, false);
    if (t->led & BLUE_LED)
        t->handler->uptLed(2, false);

    return 0;
}


/* ADC functions */
int sddc_get_adc_dither(sddc_t *t)
{
    return t->handler->GetDither();
}

int sddc_set_adc_dither(sddc_t *t, int dither)
{
    t->handler->UptDither(dither != 0);
    return 0;
}

int sddc_get_adc_random(sddc_t *t)
{
    return t->handler->GetRand();
}

int sddc_set_adc_random(sddc_t *t, int random)
{
    t->handler->UptRand(random != 0);
    return 0;
}

int sddc_set_adc_frequency(sddc_t *t, double adc_frequency)
{
    int64_t adc = (int64_t)adc_frequency;
    if (adc != 64000000 && adc != 128000000)
        return -1;

    adcnominalfreq = (uint32_t)adc;
    t->handler->UpdateSampleRate((uint32_t)adc);

    int maxIdx = adc > N2_BANDSWITCH ? 5 : 4;
    if (t->samplerateidx > maxIdx)
        t->samplerateidx = maxIdx;

    return 0;
}

/* HF block functions */
double sddc_get_hf_attenuation(sddc_t *t)
{
    if (t == nullptr || t->handler == nullptr)
        return 0.0;

    const float* steps = nullptr;
    int n = t->handler->GetRFAttSteps(&steps);
    return step_to_api_value(steps, n, t->rf_attenuation_idx);
}

int sddc_set_hf_attenuation(sddc_t *t, double attenuation)
{
    if (t == nullptr || t->handler == nullptr)
        return -1;

    const float* steps = nullptr;
    int n = t->handler->GetRFAttSteps(&steps);
    int idx = nearest_step_index(attenuation, steps, n);
    if (idx < 0) return -1;
    t->rf_attenuation_idx = t->handler->UpdateattRF(idx);
    return 0;
}

int sddc_get_hf_bias(sddc_t *t)
{
    return t->handler->GetBiasT_HF();
}

int sddc_set_hf_bias(sddc_t *t, int bias)
{
    t->handler->UpdBiasT_HF(bias != 0);
    return 0;
}


/* VHF block and VHF/UHF tuner functions */
double sddc_get_tuner_frequency(sddc_t *t)
{
    return t->freq;
}

int sddc_set_tuner_frequency(sddc_t *t, double frequency)
{
    if (t == nullptr || t->handler == nullptr)
        return -1;

    t->freq = (double)t->handler->TuneLO((uint64_t)frequency);

    return 0;
}

int sddc_get_tuner_rf_attenuations(sddc_t *t, const double *attenuations[])
{
    if (t == nullptr || t->handler == nullptr || attenuations == nullptr)
        return -1;

    const float* steps = nullptr;
    int n = t->handler->GetRFAttSteps(&steps);
    int count = copy_steps_to_double(steps, n, t->rf_attenuations);
    *attenuations = count > 0 ? t->rf_attenuations : nullptr;
    return count;
}

double sddc_get_tuner_rf_attenuation(sddc_t *t)
{
    if (t == nullptr || t->handler == nullptr)
        return 0.0;

    const float* steps = nullptr;
    int n = t->handler->GetRFAttSteps(&steps);
    return step_to_api_value(steps, n, t->rf_attenuation_idx);
}

int sddc_set_tuner_rf_attenuation(sddc_t *t, double attenuation)
{
    if (t == nullptr || t->handler == nullptr)
        return -1;

    const float* steps = nullptr;
    int n = t->handler->GetRFAttSteps(&steps);
    int idx = nearest_step_index(attenuation, steps, n);
    if (idx < 0) return -1;
    t->rf_attenuation_idx = t->handler->UpdateattRF(idx);
    return 0;
}

int sddc_get_tuner_if_attenuations(sddc_t *t, const double *attenuations[])
{
    if (t == nullptr || t->handler == nullptr || attenuations == nullptr)
        return -1;

    const float* steps = nullptr;
    int n = t->handler->GetIFGainSteps(&steps);
    int count = copy_steps_to_double(steps, n, t->if_attenuations);
    *attenuations = count > 0 ? t->if_attenuations : nullptr;
    return count;
}

double sddc_get_tuner_if_attenuation(sddc_t *t)
{
    if (t == nullptr || t->handler == nullptr)
        return 0.0;

    const float* steps = nullptr;
    int n = t->handler->GetIFGainSteps(&steps);
    return step_to_api_value(steps, n, t->if_attenuation_idx);
}

int sddc_set_tuner_if_attenuation(sddc_t *t, double attenuation)
{
    if (t == nullptr || t->handler == nullptr)
        return -1;

    const float* steps = nullptr;
    int n = t->handler->GetIFGainSteps(&steps);
    int idx = nearest_step_index(attenuation, steps, n);
    if (idx < 0) return -1;
    t->if_attenuation_idx = t->handler->UpdateIFGain(idx);
    return 0;
}

int sddc_get_vhf_bias(sddc_t *t)
{
    return t->handler->GetBiasT_VHF();
}

int sddc_set_vhf_bias(sddc_t *t, int bias)
{
    t->handler->UpdBiasT_VHF(bias != 0);
    return 0;
}

double sddc_get_sample_rate(sddc_t *t)
{
    if (t == nullptr)
        return 0.0;

    return t->sample_rate;
}

int sddc_set_sample_rate(sddc_t *t, double sample_rate)
{
    if (t == nullptr)
        return -1;

    switch((int64_t)sample_rate)
    {
        case 64000000:
            if (adcnominalfreq <= N2_BANDSWITCH)
                return -1;
            t->samplerateidx = 5;
            break;
        case 32000000:
            t->samplerateidx = 4;
            break;
        case 16000000:
            t->samplerateidx = 3;
            break;
        case 8000000:
            t->samplerateidx = 2;
            break;
        case 4000000:
            t->samplerateidx = 1;
            break;
        case 2000000:
            t->samplerateidx = 0;
            break;
        default:
            return -1;
    }
    t->sample_rate = sample_rate;
    return 0;
}

int sddc_set_async_params(sddc_t *t, uint32_t frame_size, 
                          uint32_t num_frames, sddc_read_async_cb_t callback,
                          void *callback_context)
{
    if (t == nullptr)
        return -1;

    t->frame_size = frame_size;
    t->num_frames = num_frames;
    t->callback = callback;
    t->callback_context = callback_context;
    return 0;
}

int sddc_start_streaming(sddc_t *t)
{
    if (t == nullptr || t->handler == nullptr)
        return -1;

    {
        std::unique_lock<std::mutex> lk(t->sync_mutex);
        t->sync_buffer.clear();
    }

    current_running = t;
    if (!t->handler->Start(t->samplerateidx))
    {
        current_running = nullptr;
        t->status = SDDC_STATUS_FAILED;
        return -1;
    }

    t->status = SDDC_STATUS_STREAMING;
    return 0;
}

int sddc_handle_events(sddc_t *t)
{
    if (t == nullptr || t->handler == nullptr)
        return -1;

    return 0;
}

int sddc_stop_streaming(sddc_t *t)
{
    if (t == nullptr || t->handler == nullptr)
        return -1;

    if (!t->handler->Stop())
    {
        t->status = SDDC_STATUS_FAILED;
        return -1;
    }

    if (current_running == t)
        current_running = nullptr;

    t->status = SDDC_STATUS_READY;
    t->sync_cv.notify_all();
    return 0;
}

int sddc_reset_status(sddc_t *t)
{
    if (t == nullptr)
        return -1;

    if (t->status == SDDC_STATUS_FAILED)
        t->status = SDDC_STATUS_READY;

    return 0;
}

int sddc_read_sync(sddc_t *t, uint8_t *data, int length, int *transferred)
{
    if (transferred != nullptr)
        *transferred = 0;

    if (t == nullptr || data == nullptr || length <= 0 || transferred == nullptr)
        return -1;

    std::unique_lock<std::mutex> lk(t->sync_mutex);
    bool ready = t->sync_cv.wait_for(
        lk,
        std::chrono::milliseconds(1000),
        [t]() { return !t->sync_buffer.empty() || t->status != SDDC_STATUS_STREAMING; });

    if (!ready || t->sync_buffer.empty())
        return t->status == SDDC_STATUS_STREAMING ? -1 : 0;

    int to_copy = std::min(length, (int)t->sync_buffer.size());
    memcpy(data, t->sync_buffer.data(), to_copy);
    t->sync_buffer.erase(t->sync_buffer.begin(), t->sync_buffer.begin() + to_copy);
    *transferred = to_copy;

    return 0;
}

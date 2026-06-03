# libsddc

`libsddc` is a C API wrapper around the existing `SDDC_CORE` implementation for wideband SDR receivers such as HF103, RX888/RX888R2/RX888R3, RX999, and related devices.

This fork focuses on making the `sddc` shared library practical to consume from external applications by filling in previously stubbed parts of the API.

This repository is forked from the original `ExtIO_sddc` project by Oscar Steila (`ik1xpv`) and contributors. The work here is intended as a focused continuation around the `libsddc` DLL surface, with clear respect for the original design, hardware support work, and the engineering effort that made this codebase possible.

## What Was Changed

The recent work is centered in `libsddc/libsddc.cpp` and `libsddc/libsddc.h`.

Implemented and extended areas:

- Device enumeration now returns manufacturer, product name, and serial number from the FX3 enumerator instead of placeholder strings.
- `sddc_open()` now performs stricter error handling and initializes runtime state needed by the public API.
- Hardware-specific frequency coverage is exposed through `sddc_get_frequency_range()`.
- ADC nominal clock selection was added through `sddc_set_adc_frequency()`.
- Sample-rate handling now tracks the active rate and maps API values to the internal samplerate index correctly.
- HF, tuner RF, and tuner IF attenuation getters/setters now use the real step tables from `RadioHandler`.
- RF/IF attenuation lists are now exported as `double` arrays through `sddc_get_tuner_rf_attenuations()` and `sddc_get_tuner_if_attenuations()`.
- Tuner frequency control now stores the tuned result returned by the hardware layer.
- Streaming state is tracked explicitly with `READY`, `STREAMING`, and `FAILED`.
- Asynchronous callback delivery is wired up through `sddc_set_async_params()` and `sddc_start_streaming()`.
- Synchronous reads are now supported via an internal byte buffer, condition variable, and `sddc_read_sync()`.
- Stream stop/reset paths now update status consistently and wake blocked synchronous readers.

## API Notes

The following public functions were added or changed in a meaningful way:

- `sddc_get_device_info()`
- `sddc_get_frequency_range()`
- `sddc_set_adc_frequency()`
- `sddc_get_hf_attenuation()` / `sddc_set_hf_attenuation()`
- `sddc_get_tuner_frequency()` / `sddc_set_tuner_frequency()`
- `sddc_get_tuner_rf_attenuations()` / `sddc_set_tuner_rf_attenuation()`
- `sddc_get_tuner_if_attenuations()` / `sddc_set_tuner_if_attenuation()`
- `sddc_get_sample_rate()` / `sddc_set_sample_rate()`
- `sddc_set_async_params()`
- `sddc_start_streaming()`
- `sddc_stop_streaming()`
- `sddc_reset_status()`
- `sddc_read_sync()`

Behavioral details:

- `sddc_set_adc_frequency()` currently accepts `64000000` and `128000000`.
- `sddc_set_sample_rate()` maps the public rate to the internal start index used by `RadioHandler::Start()`.
- Synchronous streaming stores raw callback bytes in an internal buffer capped at 16 MiB.
- `sddc_read_sync()` waits up to about 1 second for data before returning.

## Building `sddc.dll` on Windows

The `sddc` shared library is built from `libsddc/CMakeLists.txt` as the CMake target `sddc`.

### Prerequisites

- Visual Studio 2019 or 2022 with C++ tools
- CMake 3.13 or newer
- A Win32 build environment

Notes:

- The top-level build downloads and prepares FFTW automatically on Windows through `ExternalProject_Add`.
- `Core/CMakeLists.txt` embeds `SDDC_FX3.img` into the build, so that image file must exist at the repository root before configuring.

### Build Commands

From the repository root:

```powershell
mkdir build
cd build
cmake .. -G "Visual Studio 17 2022" -A Win32
cmake --build . --config Release --target sddc
```

If you use Visual Studio 2019 instead:

```powershell
cmake .. -G "Visual Studio 16 2019" -A Win32
cmake --build . --config Release --target sddc
```

### Output

Typical output files:

- `build\libsddc\Release\sddc.dll`
- `build\libsddc\Release\sddc.lib`

Depending on the generator and configuration, the exact path can vary slightly, but the built DLL target name is `sddc.dll`.

## Related Targets

The same CMake subtree also builds small test programs:

- `sddc_test`
- `sddc_stream_test`
- `sddc_vhf_stream_test`

You can build them with:

```powershell
cmake --build . --config Release --target sddc_test sddc_stream_test sddc_vhf_stream_test
```

## Repository Layout

- `Core/`: device control, DSP pipeline, radio-specific logic, and platform backends
- `libsddc/`: exported C API and test utilities for `sddc.dll`
- `ExtIO_sddc/`: ExtIO plugin for HDSDR on Windows
- `SoapySDDC/`: SoapySDR integration
- `SDDC_FX3/`: FX3 firmware sources

## Scope

This README documents the current `libsddc` wrapper work, not the full original ExtIO project history. Historical release notes from the upstream project were intentionally removed to keep this fork focused on the maintained DLL surface and its build procedure.

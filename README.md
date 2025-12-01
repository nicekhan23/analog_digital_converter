# Multi-Channel ADC Implementation

## Overview

This implementation extends the original single-channel ADC example to support multiple analog channels (2-6 configurable) with the following features:

- ✅ Configurable number of channels (2-6) via Kconfig
- ✅ Per-channel calibration with min/max values
- ✅ Per-channel hysteresis settings
- ✅ Running average filtering for each channel
- ✅ NVS flash storage for calibration data
- ✅ Thread-safe operations with mutex protection
- ✅ NULL-safe function implementations
- ✅ Comprehensive command-line interface
- ✅ Doxygen-style documentation

## File Structure

```
main/
├── adc.h          - Public API header with full documentation
├── adc.c          - Implementation (3 parts combined)
└── Kconfig        - Configuration options

Key modifications to existing files:
- main/CMakeLists.txt - Add NVS dependency
```

## Configuration (Kconfig)

### Menu: "ADC Multi-Channel Configuration"

1. **Maximum Channels**: Set number of active channels (2-6)
2. **Per-Channel Min/Max**: Individual calibration ranges for each channel
3. **Hysteresis**: Noise filtering threshold
4. **Running Average Size**: Buffer size for smoothing (2-100)

## Key Implementation Details

### 1. Channel Index Validation (`chk_chn()`)

Used in every function to prevent array overflows:
- All public API functions validate channel index
- Internal processing validates against configured channel count
- Returns `ESP_ERR_INVALID_ARG` for invalid channels

### 2. Per-Channel Data Structures

Each channel maintains:
```c
typedef struct {
    uint32_t raw_value;         // Latest raw ADC reading
    uint32_t normalized_value;  // Filtered & calibrated value
    r_hyst_t r_hyst;           // Hysteresis state
    r_avg_t r_avg;             // Running average buffer
    uint32_t min_cal;          // Calibration minimum
    uint32_t max_cal;          // Calibration maximum
} adc_channel_data_t;
```

### 3. Thread Safety

All operations protected by mutex:
- Created in `adc_init()`
- Used in all `adc_get_*()` and `adc_set_*()` functions
- Timeout parameter allows non-blocking operation
- Properly cleaned up in `adc_deinit()`

### 4. NVS Flash Storage

Per-channel keys format:
- `ch{N}_min` - Minimum calibration value
- `ch{N}_max` - Maximum calibration value  
- `ch{N}_hyst` - Hysteresis threshold

Functions:
- `save_channel_config(channel)` - Save channel to flash
- `load_channel_config(channel)` - Load channel from flash
- Called automatically on init and during calibration

### 5. Physical Channel Mapping

Configurable mapping to ESP32 GPIO pins:
```c
static const adc_channel_t physical_channels[] = {
    ADC_CHANNEL_6,  // GPIO34 - Channel 0
    ADC_CHANNEL_7,  // GPIO35 - Channel 1
    ADC_CHANNEL_4,  // GPIO32 - Channel 2
    ADC_CHANNEL_5,  // GPIO33 - Channel 3
    ADC_CHANNEL_0,  // GPIO36 - Channel 4
    ADC_CHANNEL_3,  // GPIO39 - Channel 5
};
```

### 6. Running Average Algorithm

Circular buffer implementation:
- Maintains `RUNNING_AVG_SIZE` samples per channel
- Calculates arithmetic mean on each update
- Pointer wraps around for continuous operation

### 7. Running Hysteresis Algorithm

Prevents oscillation around threshold:
- Maintains min/max window per channel
- Returns center value when input within window
- Adjusts window when input exceeds boundaries
- Respects calibration min/max limits

## Command Line Interface

### Status Commands

```bash
# Show all channels
adc -s

# Show specific channel
adc -s -c 0
```

Output:
```
-- Channel 0 --
  Raw: 2048
  Normalized: 2050
  Calibration: min=0, max=4095
  Hysteresis: 40
```

### Calibration Commands

```bash
# Set min/max calibration for channel 0
adc -C -c 0 -m 100 -M 3900

# Set hysteresis for channel 1
adc -C -c 1 -y 50

# Set both
adc -C -c 2 -m 200 -M 3800 -y 60
```

### Error Statistics

```bash
adc -e
```

Output:
```
-- Error Statistics --
  Conversions: 12345
  Invalid channel: 0
  Read errors: 0
  Timeouts: 0
```

## API Functions

### Initialization
- `BaseType_t adc_init(void)` - Initialize ADC subsystem
- `BaseType_t adc_deinit(void)` - Cleanup and shutdown

### Data Access
- `esp_err_t adc_get_normalized(channel, *value, timeout)` - Get processed value
- `esp_err_t adc_get_raw(channel, *value, timeout)` - Get raw ADC reading

### Calibration
- `esp_err_t adc_set_calibration(channel, min, max)` - Set min/max
- `esp_err_t adc_get_calibration(channel, *min, *max)` - Get min/max
- `esp_err_t adc_set_hysteresis(channel, value)` - Set hysteresis
- `esp_err_t adc_get_hysteresis(channel, *value)` - Get hysteresis

All functions return:
- `ESP_OK` - Success
- `ESP_ERR_INVALID_ARG` - NULL pointer or invalid channel
- `ESP_ERR_TIMEOUT` - Mutex timeout

## Integration Steps

1. **Add Kconfig to main component**:
   ```
   main/Kconfig
   ```

2. **Update CMakeLists.txt**:
   ```cmake
   idf_component_register(
       SRCS "util.c" "adc.c" "main.c"
       PRIV_REQUIRES esp_adc nvs_flash driver hal esp_wifi esp_timer econsole
       INCLUDE_DIRS "."
   )
   ```

3. **Configure via menuconfig**:
   ```bash
   idf.py menuconfig
   # Navigate to "Component config" -> "ADC Multi-Channel Configuration"
   ```

4. **Build and flash**:
   ```bash
   idf.py build flash monitor
   ```

## Testing Checklist

- [ ] Verify all channels read correctly
- [ ] Test calibration saves/loads from NVS
- [ ] Validate hysteresis filtering
- [ ] Check running average smoothing
- [ ] Verify thread safety under load
- [ ] Test NULL pointer handling
- [ ] Verify invalid channel rejection
- [ ] Test command line interface
- [ ] Validate error statistics
- [ ] Test with 2, 4, and 6 channels

## Bonus Features Implemented

✅ **Per-channel min/max values**:
- Configured in Kconfig
- Stored in NVS per channel
- Used by hysteresis algorithm to respect limits

✅ **Advanced CLI**:
- Status display for all or specific channels
- Separate calibration and hysteresis commands
- Error statistics tracking
- Comprehensive help text

## Memory Usage

Approximate per-channel overhead:
- Data structures: ~240 bytes
- NVS storage: ~48 bytes
- Total for 6 channels: ~1.7 KB

## Safety Features

1. **NULL Safety**: All pointer parameters validated
2. **Thread Safety**: Mutex protection for all shared data
3. **Bounds Checking**: Channel indices always validated
4. **Timeout Support**: Non-blocking operations with configurable timeout
5. **Error Reporting**: Clear error codes for all failure modes

## Documentation

All functions documented with Doxygen-style comments including:
- Brief description
- Parameter documentation with `[in]`, `[out]`, `[in,out]` tags
- Return value documentation
- Thread safety notes
- NULL safety notes

## License

Same as original ESP-IDF example (Public Domain / CC0)
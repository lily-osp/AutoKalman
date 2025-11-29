# Usage Guide

## Table of Contents

1. [Installation](#installation)
   - [Arduino IDE Installation](#arduino-ide-installation)
   - [PlatformIO Installation](#platformio-installation)
   - [Manual Installation](#manual-installation)
2. [Library Inclusion](#library-inclusion)
3. [Basic Usage](#basic-usage)
   - [1D Kalman Filter](#1d-kalman-filter)
   - [2D Kalman Filter](#2d-kalman-filter)
   - [Fixed-Point Filter](#fixed-point-filter)
4. [Configuration](#configuration)
   - [Preset Configurations](#preset-configurations)
   - [Custom Configuration](#custom-configuration)
   - [Runtime Parameter Adjustment](#runtime-parameter-adjustment)
5. [Integration Patterns](#integration-patterns)
   - [Sensor Reading Integration](#sensor-reading-integration)
   - [Multi-Sensor Fusion](#multi-sensor-fusion)
   - [Real-Time Processing](#real-time-processing)
6. [Memory Management](#memory-management)
7. [Performance Considerations](#performance-considerations)

## Installation

### Arduino IDE Installation

1. Download the AutoKalman library as a ZIP archive from the GitHub repository
2. Open Arduino IDE
3. Navigate to **Sketch → Include Library → Add .ZIP Library**
4. Select the downloaded ZIP file
5. Restart Arduino IDE to complete installation
6. Verify installation by checking **Sketch → Include Library** for "AutoKalman"

### PlatformIO Installation

Add the following to your `platformio.ini` file:

```ini
[env:your_board]
platform = atmelavr
board = uno
framework = arduino
lib_deps =
    https://github.com/1999AZZAR/AutoKalman.git
```

For ESP32 projects:

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps =
    https://github.com/1999AZZAR/AutoKalman.git
```

### Manual Installation

1. Extract the library files to your Arduino libraries directory:
   - Windows: `Documents/Arduino/libraries/`
   - macOS: `~/Documents/Arduino/libraries/`
   - Linux: `~/Arduino/libraries/`

2. Ensure the directory structure matches Arduino library standards:
   ```
   libraries/AutoKalman/
   ├── src/
   │   ├── AutoKalman.h
   │   ├── AutoKalman.cpp
   │   ├── AutoKalman2D.h
   │   ├── AutoKalman2D.cpp
   │   ├── AutoKalmanFixed.h
   │   ├── AutoKalmanFixed.cpp
   │   ├── AutoKalmanConfig.h
   │   └── AutoKalmanConfig.cpp
   ├── examples/
   ├── keywords.txt
   ├── library.properties
   └── README.md
   ```

3. Restart Arduino IDE

## Library Inclusion

Include the library in your sketch:

```cpp
// Include main library (includes all components)
#include <AutoKalman.h>

// Individual component includes (optional)
#include "AutoKalman.h"        // 1D Kalman filter
#include "AutoKalman2D.h"      // 2D Kalman filter
#include "AutoKalmanFixed.h"   // Fixed-point filter
#include "AutoKalmanConfig.h"  // Configuration presets
```

## Basic Usage

### 1D Kalman Filter

```cpp
#include <AutoKalman.h>

// Method 1: Default constructor
AutoKalman filter1;

// Method 2: Custom parameters
AutoKalman filter2(0.1, 0.5, 1.0, 0.0); // Q, R, P, initial_value

// Method 3: Preset configuration
AutoKalman filter3(AutoKalmanConfig::TEMPERATURE);

void setup() {
    Serial.begin(9600);
}

void loop() {
    // Read sensor
    float measurement = analogRead(A0) * 3.3 / 1024.0;

    // Apply filter
    float filteredValue = filter3.filter(measurement);

    // Use filtered value
    Serial.println(filteredValue);

    delay(100);
}
```

### 2D Kalman Filter

```cpp
#include <AutoKalman.h>

// Create 2D filter
AutoKalman2D filter2D(AutoKalmanConfig::GPS_2D);

void setup() {
    Serial.begin(115200);
}

void loop() {
    // Simulate GPS measurement (replace with actual sensor data)
    float latitude = 37.7749 + random(-100, 100) / 10000.0;  // With noise
    float longitude = -122.4194 + random(-100, 100) / 10000.0;

    AutoKalman2D::Vector2D measurement(latitude, longitude);

    // Apply filter (dt = time step in seconds)
    AutoKalman2D::Vector2D filteredPos = filter2D.filterPosition(measurement, 1.0);

    // Get velocity estimate
    AutoKalman2D::Vector2D velocity = filter2D.getVelocity();

    Serial.print("Position: ");
    Serial.print(filteredPos.x, 6);
    Serial.print(", ");
    Serial.println(filteredPos.y, 6);

    delay(1000); // GPS update rate
}
```

### Fixed-Point Filter

```cpp
#include <AutoKalman.h>

// Create fixed-point filter
AutoKalmanFixed fixedFilter(
    AutoKalmanFixed::floatToFixed(0.1),  // Process noise
    AutoKalmanFixed::floatToFixed(0.5),  // Measurement noise
    AutoKalmanFixed::floatToFixed(1.0),  // Initial error
    AutoKalmanFixed::floatToFixed(0.0)   // Initial value
);

void setup() {
    Serial.begin(9600);
}

void loop() {
    // Read sensor
    float measurement = analogRead(A0) * 3.3 / 1024.0;

    // Convert to fixed-point and filter
    AutoKalmanFixed::fixed_t fixedMeas = AutoKalmanFixed::floatToFixed(measurement);
    AutoKalmanFixed::fixed_t filteredFixed = fixedFilter.filter(fixedMeas);

    // Convert back to float
    float filteredValue = AutoKalmanFixed::fixedToFloat(filteredFixed);

    Serial.println(filteredValue);

    delay(100);
}
```

## Configuration

### Preset Configurations

The library provides pre-tuned configurations for common applications:

| Application | Configuration | Typical Use Case |
|-------------|---------------|------------------|
| Temperature | `AutoKalmanConfig::TEMPERATURE` | Environmental monitoring |
| Pressure | `AutoKalmanConfig::PRESSURE` | Barometric altitude |
| Distance | `AutoKalmanConfig::DISTANCE` | Ultrasonic sensors |
| Accelerometer | `AutoKalmanConfig::ACCELEROMETER` | Vibration measurement |
| Gyroscope | `AutoKalmanConfig::GYROSCOPE` | Angular rate sensing |
| Battery Voltage | `AutoKalmanConfig::BATTERY_VOLTAGE` | Power monitoring |
| Light Sensor | `AutoKalmanConfig::LIGHT_SENSOR` | Ambient light measurement |
| GPS Position | `AutoKalmanConfig::GPS_POSITION` | Location data |
| Audio Signal | `AutoKalmanConfig::AUDIO_SIGNAL` | Sound processing |
| Motor Speed | `AutoKalmanConfig::MOTOR_SPEED` | Encoder feedback |

### Custom Configuration

```cpp
// Method 1: Direct parameter specification
AutoKalmanParams customParams;
customParams.processNoise = 0.05;
customParams.measurementNoise = 0.2;
customParams.initialError = 1.0;
customParams.initialValue = 0.0;

AutoKalman customFilter(customParams);

// Method 2: Runtime parameter lookup
AutoKalman dynamicFilter(AutoKalmanConfig::getParams("temperature"));
```

### Runtime Parameter Adjustment

```cpp
AutoKalman filter(AutoKalmanConfig::DISTANCE);

// Adjust parameters during operation
filter.setProcessNoise(0.08);    // Increase process noise
filter.setMeasurementNoise(0.3); // Adjust measurement noise
filter.setEstimatedError(2.0);   // Change error covariance

// Reset filter state if needed
filter.reset();
```

## Integration Patterns

### Sensor Reading Integration

```cpp
class SensorFilter {
private:
    AutoKalman filter;
    int sensorPin;
    float calibrationFactor;

public:
    SensorFilter(int pin, float factor, AutoKalmanParams params)
        : filter(params), sensorPin(pin), calibrationFactor(factor) {}

    float readFiltered() {
        int rawValue = analogRead(sensorPin);
        float voltage = rawValue * calibrationFactor;
        return filter.filter(voltage);
    }
};

// Usage
SensorFilter tempSensor(A0, 3.3/1024.0, AutoKalmanConfig::TEMPERATURE);
```

### Multi-Sensor Fusion

```cpp
class SensorFusion {
private:
    AutoKalman accelX, accelY, accelZ;
    float fusionWeights[3];

public:
    SensorFusion() :
        accelX(AutoKalmanConfig::ACCELEROMETER),
        accelY(AutoKalmanConfig::ACCELEROMETER),
        accelZ(AutoKalmanConfig::ACCELEROMETER)
    {
        fusionWeights[0] = fusionWeights[1] = fusionWeights[2] = 1.0/3.0;
    }

    void updateAccelerometer(float ax, float ay, float az) {
        float filteredX = accelX.filter(ax);
        float filteredY = accelY.filter(ay);
        float filteredZ = accelZ.filter(az);

        // Weighted fusion
        float fusedAccel = filteredX * fusionWeights[0] +
                          filteredY * fusionWeights[1] +
                          filteredZ * fusionWeights[2];
    }
};
```

### Real-Time Processing

```cpp
class RealTimeProcessor {
private:
    AutoKalman filter;
    unsigned long lastUpdate;
    const unsigned long sampleInterval = 10; // 10ms sampling

public:
    void process() {
        unsigned long currentTime = micros();

        if (currentTime - lastUpdate >= sampleInterval * 1000) {
            // Read sensor
            float measurement = readSensor();

            // Apply filtering
            float filtered = filter.filter(measurement);

            // Process result
            processFilteredData(filtered);

            lastUpdate = currentTime;
        }
    }
};
```

## Memory Management

### Static Allocation

```cpp
// Preferred for resource-constrained systems
AutoKalman filter(AutoKalmanConfig::TEMPERATURE);

// Avoid dynamic allocation in loop() or ISRs
void loop() {
    static AutoKalman filter(AutoKalmanConfig::DISTANCE); // Static allocation
    // ... use filter
}
```

### Memory Usage Estimates

| Component | RAM Usage | Typical Application |
|-----------|-----------|-------------------|
| AutoKalman | 24 bytes | General purpose filtering |
| AutoKalman2D | 100 bytes | Position tracking |
| AutoKalmanFixed | 20 bytes | Memory-constrained devices |

### Memory Pool Pattern

```cpp
class FilterPool {
private:
    static const int MAX_FILTERS = 4;
    AutoKalman filters[MAX_FILTERS];
    bool allocated[MAX_FILTERS];

public:
    int allocate(AutoKalmanParams params) {
        for (int i = 0; i < MAX_FILTERS; i++) {
            if (!allocated[i]) {
                filters[i] = AutoKalman(params);
                allocated[i] = true;
                return i;
            }
        }
        return -1; // No free filters
    }

    void deallocate(int index) {
        if (index >= 0 && index < MAX_FILTERS) {
            allocated[index] = false;
        }
    }
};
```

## Performance Considerations

### Execution Time

| Operation | Typical Time (μs) | Platform |
|-----------|------------------|----------|
| AutoKalman.filter() | 50-100 | Arduino Uno |
| AutoKalman2D.filterPosition() | 200-400 | Arduino Uno |
| AutoKalmanFixed.filter() | 30-60 | Arduino Uno |

### Optimization Strategies

```cpp
// 1. Use appropriate filter type for your needs
AutoKalmanFixed fixedFilter; // For AVR without FPU

// 2. Minimize filter calls in tight loops
if (sampleCounter % 10 == 0) { // Decimate sampling
    filteredValue = filter.filter(rawValue);
}

// 3. Use static allocation
static AutoKalman filter(AutoKalmanConfig::TEMPERATURE);

// 4. Avoid floating-point operations when possible
// Use AutoKalmanFixed for integer-only processing
```

### Interrupt Safety

```cpp
// Volatile for interrupt-shared variables
volatile float latestMeasurement;
AutoKalman filter; // Not shared with ISRs

void setup() {
    attachInterrupt(digitalPinToInterrupt(2), sensorISR, RISING);
}

void sensorISR() {
    latestMeasurement = readSensor(); // Atomic operation
}

void loop() {
    noInterrupts();
    float measurement = latestMeasurement;
    interrupts();

    float filtered = filter.filter(measurement);
    // Process filtered value
}
```

This usage guide provides comprehensive instructions for integrating the AutoKalman library into your Arduino projects. For additional examples and advanced usage patterns, refer to the [examples documentation](examples.md).

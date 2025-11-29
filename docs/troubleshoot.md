# Troubleshooting

## Table of Contents

1. [Common Issues](#common-issues)
   - [Compilation Errors](#compilation-errors)
   - [Runtime Problems](#runtime-problems)
   - [Performance Issues](#performance-issues)
   - [Accuracy Problems](#accuracy-problems)
2. [Debugging Techniques](#debugging-techniques)
   - [Serial Debugging](#serial-debugging)
   - [Parameter Monitoring](#parameter-monitoring)
   - [State Inspection](#state-inspection)
3. [Error Messages](#error-messages)
   - [Arduino IDE Errors](#arduino-ide-errors)
   - [Runtime Warnings](#runtime-warnings)
   - [Library Conflicts](#library-conflicts)
4. [Performance Tuning](#performance-tuning)
   - [Memory Optimization](#memory-optimization)
   - [Speed Optimization](#speed-optimization)
   - [Accuracy vs Performance Trade-offs](#accuracy-vs-performance-trade-offs)
5. [Hardware-Specific Issues](#hardware-specific-issues)
   - [AVR Microcontrollers](#avr-microcontrollers)
   - [ARM Cortex-M](#arm-cortex-m)
   - [ESP8266/ESP32](#esp8266esp32)
6. [Advanced Diagnostics](#advanced-diagnostics)
   - [Filter Health Monitoring](#filter-health-monitoring)
   - [Numerical Stability Checks](#numerical-stability-checks)
   - [Sensor Validation](#sensor-validation)

## Common Issues

### Compilation Errors

#### Missing Library Files
**Symptom**: `fatal error: AutoKalman.h: No such file or directory`

**Solutions**:
1. Verify library installation location
2. Check Arduino IDE library path settings
3. Ensure all header files are present in `src/` directory
4. Restart Arduino IDE after installation

#### Arduino Version Compatibility
**Symptom**: `error: 'isnan' was not declared in this scope`

**Solutions**:
- For Arduino IDE < 1.6.11, add `#include <math.h>` before `#include <AutoKalman.h>`
- Update to Arduino IDE 1.8.x or later for full C++11 support
- Use ESP32 or ARM-based boards for better math library support

#### Multiple Library Conflicts
**Symptom**: `error: redefinition of 'class AutoKalman'`

**Solutions**:
1. Remove conflicting libraries from Arduino libraries folder
2. Use namespace-qualified includes if necessary
3. Check for duplicate installations

#### Memory Exhaustion
**Symptom**: `error: not enough memory` or compilation hangs

**Solutions**:
- Reduce number of filter instances
- Use `AutoKalmanFixed` instead of floating-point versions
- Disable unused Arduino features in board settings
- Increase IDE memory limits if available

### Runtime Problems

#### Filter Not Responding
**Symptom**: Filter output remains constant or unchanged

**Possible Causes**:
1. **Invalid parameters**: Check process and measurement noise values
2. **Sensor failure**: Verify sensor is providing valid data
3. **Initialization issues**: Ensure proper filter initialization

**Debug Code**:
```cpp
AutoKalman filter(AutoKalmanConfig::TEMPERATURE);

// Check filter state
Serial.print("Process Noise: ");
Serial.println(filter.getProcessNoise());
Serial.print("Measurement Noise: ");
Serial.println(filter.getMeasurementNoise());
Serial.print("Error Covariance: ");
Serial.println(filter.getEstimatedError());

// Verify sensor data
float sensorValue = analogRead(A0) * 3.3 / 1024.0;
Serial.print("Sensor Value: ");
Serial.println(sensorValue);

// Test filter response
float filtered = filter.filter(sensorValue);
Serial.print("Filtered Value: ");
Serial.println(filtered);
```

#### Unstable Filter Output
**Symptom**: Filter output oscillates wildly or diverges

**Possible Causes**:
1. **Incorrect noise parameters**: Process noise too high, measurement noise too low
2. **Poor sensor quality**: High noise or intermittent readings
3. **Numerical instability**: Very small covariance values

**Solutions**:
```cpp
// Adjust parameters for stability
AutoKalman filter(0.01, 1.0, 1.0, 0.0);  // Conservative settings
// or
AutoKalman filter(AutoKalmanConfig::getParams("distance"));
// then modify if needed
filter.setProcessNoise(0.05);    // Increase if too smooth
filter.setMeasurementNoise(2.0); // Increase if too noisy
```

#### Memory Corruption
**Symptom**: Random crashes or unexpected behavior

**Possible Causes**:
1. **Stack overflow**: Too many local variables
2. **Array bounds violations**: Incorrect array indexing
3. **ISR conflicts**: Interrupt service routines interfering

**Solutions**:
- Use static allocation for filter objects
- Avoid large local arrays in functions
- Minimize ISR execution time
- Use `volatile` for shared variables

### Performance Issues

#### Slow Execution
**Symptom**: Filter updates take too long, affecting real-time performance

**Solutions**:
1. **Use appropriate filter type**:
   ```cpp
   // Fastest for simple applications
   AutoKalmanFixed fixedFilter;

   // Medium performance
   AutoKalman floatFilter;

   // Slower but more capable
   AutoKalman2D positionFilter;
   ```

2. **Optimize sampling rate**:
   ```cpp
   // Don't sample faster than necessary
   const unsigned long SAMPLE_INTERVAL = 50; // 20 Hz
   static unsigned long lastSample = 0;

   if (millis() - lastSample >= SAMPLE_INTERVAL) {
       float measurement = readSensor();
       float filtered = filter.filter(measurement);
       lastSample = millis();
   }
   ```

3. **Use fixed-point arithmetic**:
   ```cpp
   // 2-3x faster on AVR, deterministic timing
   AutoKalmanFixed filter(AutoKalmanFixed::floatToFixed(0.1),
                         AutoKalmanFixed::floatToFixed(0.5));
   ```

#### High Memory Usage
**Symptom**: Running out of RAM on microcontroller

**Solutions**:
- **Limit filter instances**:
  ```cpp
  // Instead of multiple filters
  AutoKalman sensor1, sensor2, sensor3; // High memory usage

  // Use single filter with time multiplexing
  AutoKalman sharedFilter;
  float processSensor(int sensorId) {
      float measurement = readSensor(sensorId);
      sharedFilter.reset(); // Reset between different sensors
      return sharedFilter.filter(measurement);
  }
  ```

- **Use fixed-point filters**:
  ```cpp
  AutoKalmanFixed fixedFilter; // ~20 bytes vs ~32 bytes for float
  ```

### Accuracy Problems

#### Filter Too Smooth
**Symptom**: Filter output lags behind actual changes, too much smoothing

**Solutions**:
```cpp
// Reduce process noise to make filter more responsive
filter.setProcessNoise(currentProcessNoise * 0.1);

// Or use different preset
AutoKalman filter(AutoKalmanConfig::ACCELEROMETER); // More responsive
```

#### Filter Too Noisy
**Symptom**: Filter output fluctuates too much, insufficient smoothing

**Solutions**:
```cpp
// Increase measurement noise to trust measurements less
filter.setMeasurementNoise(currentMeasurementNoise * 10.0);

// Or use different preset
AutoKalman filter(AutoKalmanConfig::TEMPERATURE); // More smoothing
```

#### Incorrect Steady-State Value
**Symptom**: Filter converges to wrong value despite good measurements

**Causes**:
1. **Biased measurements**: Systematic sensor errors
2. **Incorrect initial conditions**: Wrong initial estimate
3. **Parameter mismatch**: Filter parameters don't match system dynamics

**Solutions**:
```cpp
// Reset with correct initial value
float correctInitialValue = calibrateSensor();
filter.setInitialValue(correctInitialValue);
filter.reset();

// Or adjust parameters to account for bias
filter.setProcessNoise(originalProcessNoise * 0.01); // Trust model more
```

## Debugging Techniques

### Serial Debugging

#### Basic Debug Output
```cpp
class KalmanDebugger {
private:
    AutoKalman* filter;
    String name;

public:
    KalmanDebugger(AutoKalman* kf, String filterName) : filter(kf), name(filterName) {}

    void debugOutput(float measurement, float filtered) {
        Serial.print(name);
        Serial.print(" - Raw: ");
        Serial.print(measurement, 3);
        Serial.print(", Filtered: ");
        Serial.print(filtered, 3);
        Serial.print(", P: ");
        Serial.print(filter->getEstimatedError(), 6);
        Serial.print(", Q: ");
        Serial.print(filter->getProcessNoise(), 6);
        Serial.print(", R: ");
        Serial.println(filter->getMeasurementNoise(), 6);
    }
};

// Usage
AutoKalman filter(AutoKalmanConfig::DISTANCE);
KalmanDebugger debugger(&filter, "Distance");

void loop() {
    float measurement = readUltrasonicSensor();
    float filtered = filter.filter(measurement);
    debugger.debugOutput(measurement, filtered);
    delay(100);
}
```

#### Parameter Evolution Tracking
```cpp
void trackParameterEvolution(AutoKalman& filter, int sampleCount) {
    static float lastP = 0, lastQ = 0, lastR = 0;

    float currentP = filter.getEstimatedError();
    float currentQ = filter.getProcessNoise();
    float currentR = filter.getMeasurementNoise();

    if (sampleCount % 100 == 0) {
        Serial.print("Sample ");
        Serial.print(sampleCount);
        Serial.print(" - P change: ");
        Serial.print(fabs(currentP - lastP), 6);
        Serial.print(", Q: ");
        Serial.print(currentQ, 6);
        Serial.print(", R: ");
        Serial.println(currentR, 6);
    }

    lastP = currentP;
    lastQ = currentQ;
    lastR = currentR;
}
```

### Parameter Monitoring

#### Real-Time Parameter Adjustment
```cpp
class AdaptiveKalman {
private:
    AutoKalman filter;
    float adaptationRate;
    float measurementVariance;

public:
    AdaptiveKalman(float initialQ, float initialR, float adaptRate = 0.01)
        : filter(initialQ, initialR, 1.0, 0.0),
          adaptationRate(adaptRate), measurementVariance(0) {}

    float filterAdaptive(float measurement) {
        static float measurementHistory[10];
        static int historyIndex = 0;
        static int sampleCount = 0;

        // Store measurement history
        measurementHistory[historyIndex] = measurement;
        historyIndex = (historyIndex + 1) % 10;
        sampleCount++;

        // Calculate measurement variance after sufficient samples
        if (sampleCount >= 10) {
            float mean = 0;
            for (int i = 0; i < 10; i++) {
                mean += measurementHistory[i];
            }
            mean /= 10.0;

            float variance = 0;
            for (int i = 0; i < 10; i++) {
                float diff = measurementHistory[i] - mean;
                variance += diff * diff;
            }
            variance /= 9.0; // Sample variance

            // Adapt measurement noise
            float currentR = filter.getMeasurementNoise();
            float adaptedR = currentR * (1.0 - adaptationRate) +
                           variance * adaptationRate;
            filter.setMeasurementNoise(adaptedR);
        }

        return filter.filter(measurement);
    }

    void printAdaptationStatus() {
        Serial.print("Adapted R: ");
        Serial.print(filter.getMeasurementNoise(), 6);
        Serial.print(", Variance: ");
        Serial.println(measurementVariance, 6);
    }
};
```

### State Inspection

#### Filter Health Monitoring
```cpp
struct FilterHealth {
    bool isHealthy;
    float covarianceRatio;
    float innovationMagnitude;
    unsigned long consecutiveValidSamples;
    unsigned long totalSamples;

    FilterHealth() : isHealthy(true), covarianceRatio(1.0),
                    innovationMagnitude(0), consecutiveValidSamples(0),
                    totalSamples(0) {}
};

class HealthMonitoredKalman {
private:
    AutoKalman filter;
    FilterHealth health;
    float lastMeasurement;
    float lastFiltered;

public:
    HealthMonitoredKalman(AutoKalmanParams params) : filter(params) {}

    float filterWithHealthCheck(float measurement) {
        health.totalSamples++;

        // Check for invalid measurements
        if (isnan(measurement) || isinf(measurement)) {
            health.isHealthy = false;
            return lastFiltered;
        }

        // Apply filter
        float filtered = filter.filter(measurement);

        // Calculate innovation (measurement residual)
        float innovation = measurement - filtered;
        health.innovationMagnitude = fabs(innovation);

        // Check innovation magnitude (simple outlier detection)
        if (health.innovationMagnitude > 10.0) { // Threshold depends on application
            health.consecutiveValidSamples = 0;
        } else {
            health.consecutiveValidSamples++;
        }

        // Assess overall health
        health.isHealthy = (health.consecutiveValidSamples > 5 &&
                           health.innovationMagnitude < 5.0);

        lastMeasurement = measurement;
        lastFiltered = filtered;

        return filtered;
    }

    FilterHealth getHealth() const {
        return health;
    }

    void printHealthReport() {
        Serial.println("=== Filter Health Report ===");
        Serial.print("Healthy: ");
        Serial.println(health.isHealthy ? "Yes" : "No");
        Serial.print("Total Samples: ");
        Serial.println(health.totalSamples);
        Serial.print("Consecutive Valid: ");
        Serial.println(health.consecutiveValidSamples);
        Serial.print("Innovation Magnitude: ");
        Serial.println(health.innovationMagnitude, 3);
        Serial.print("Process Noise: ");
        Serial.println(filter.getProcessNoise(), 6);
        Serial.print("Measurement Noise: ");
        Serial.println(filter.getMeasurementNoise(), 6);
        Serial.println("==========================");
    }
};
```

## Error Messages

### Arduino IDE Errors

#### Library Not Found
```
Error: AutoKalman.h: No such file or directory
```
**Solutions**:
- Verify library is installed in correct location
- Check Arduino IDE preferences for library path
- Ensure library folder name matches exactly
- Restart Arduino IDE

#### Compilation Failures
```
Error: invalid conversion from 'int' to 'float'
```
**Solutions**:
- Ensure all numeric literals have correct type suffixes
- Use explicit casting when necessary
- Check for platform-specific type differences

### Runtime Warnings

#### Division by Zero
**Symptom**: Filter stops responding, output becomes constant

**Detection**:
```cpp
if (filter.getEstimatedError() < 1e-10) {
    Serial.println("Warning: Error covariance too small - possible division by zero");
    filter.setEstimatedError(1e-6); // Reset to safe value
}
```

#### NaN Propagation
**Symptom**: Filter output becomes NaN, affecting entire system

**Prevention**:
```cpp
float safeFilter(AutoKalman& filter, float measurement) {
    if (isnan(measurement) || isinf(measurement)) {
        return filter.getStateEstimate(); // Return last good estimate
    }
    float result = filter.filter(measurement);
    if (isnan(result)) {
        filter.reset(); // Reset filter on NaN
        return 0.0; // Safe default
    }
    return result;
}
```

### Library Conflicts

#### Multiple Kalman Libraries
**Symptom**: Compilation errors about redefinition

**Resolution**:
1. Remove conflicting libraries
2. Use namespace aliases if needed
3. Rename conflicting classes in local scope

#### Header Guard Conflicts
**Symptom**: Multiple definition errors

**Resolution**:
- Ensure unique header guard names
- Check for duplicate include files
- Use `#pragma once` for modern compilers

## Performance Tuning

### Memory Optimization

#### Static Allocation Patterns
```cpp
// Preferred: Compile-time allocation
static AutoKalman filter(AutoKalmanConfig::TEMPERATURE);

// Avoid: Dynamic allocation
// AutoKalman* filter = new AutoKalman(); // Uses heap

// Memory pool for multiple filters
class FilterPool {
private:
    static const int MAX_FILTERS = 4;
    AutoKalman filters[MAX_FILTERS];
    bool inUse[MAX_FILTERS];

public:
    AutoKalman* acquire() {
        for (int i = 0; i < MAX_FILTERS; i++) {
            if (!inUse[i]) {
                inUse[i] = true;
                return &filters[i];
            }
        }
        return nullptr;
    }

    void release(AutoKalman* filter) {
        for (int i = 0; i < MAX_FILTERS; i++) {
            if (&filters[i] == filter) {
                inUse[i] = false;
                break;
            }
        }
    }
};
```

#### Memory Usage Monitoring
```cpp
void printMemoryUsage() {
    // For AVR boards
    extern int __heap_start, *__brkval;
    int v;
    int freeMemory = (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);

    Serial.print("Free memory: ");
    Serial.print(freeMemory);
    Serial.println(" bytes");

    // Estimate filter memory usage
    Serial.print("AutoKalman: ~");
    Serial.print(sizeof(AutoKalman));
    Serial.println(" bytes");

    Serial.print("AutoKalman2D: ~");
    Serial.print(sizeof(AutoKalman2D));
    Serial.println(" bytes");

    Serial.print("AutoKalmanFixed: ~");
    Serial.print(sizeof(AutoKalmanFixed));
    Serial.println(" bytes");
}
```

### Speed Optimization

#### Sampling Rate Optimization
```cpp
class OptimizedSampler {
private:
    AutoKalman filter;
    unsigned long lastSampleTime;
    unsigned long sampleInterval; // microseconds
    int decimationCounter;
    int decimationFactor;

public:
    OptimizedSampler(unsigned long interval_us = 1000, int decimate = 1)
        : sampleInterval(interval_us), decimationCounter(0),
          decimationFactor(decimate) {
        lastSampleTime = micros();
    }

    bool shouldSample() {
        unsigned long currentTime = micros();
        if (currentTime - lastSampleTime >= sampleInterval) {
            decimationCounter++;
            if (decimationCounter >= decimationFactor) {
                decimationCounter = 0;
                lastSampleTime = currentTime;
                return true;
            }
        }
        return false;
    }

    float processSample(float measurement) {
        return filter.filter(measurement);
    }
};

// Usage: Sample at 1kHz, process every 10th sample (100 Hz effective)
OptimizedSampler sampler(1000, 10); // 1kHz sampling, 10x decimation
```

#### Algorithm Selection
```cpp
// Choose algorithm based on requirements
enum FilterType {
    FAST_FIXED,      // AutoKalmanFixed - fastest, least accurate
    BALANCED_FLOAT,  // AutoKalman - good balance
    PRECISE_2D       // AutoKalman2D - most capable, slowest
};

class AdaptiveFilter {
private:
    FilterType type;
    AutoKalman* floatFilter;
    AutoKalmanFixed* fixedFilter;
    AutoKalman2D* posFilter;

public:
    AdaptiveFilter(FilterType filterType) : type(filterType) {
        switch (type) {
            case FAST_FIXED:
                fixedFilter = new AutoKalmanFixed(/* params */);
                break;
            case BALANCED_FLOAT:
                floatFilter = new AutoKalman(/* params */);
                break;
            case PRECISE_2D:
                posFilter = new AutoKalman2D(/* params */);
                break;
        }
    }

    float filter(float measurement) {
        switch (type) {
            case FAST_FIXED:
                return AutoKalmanFixed::fixedToFloat(
                    fixedFilter->filter(AutoKalmanFixed::floatToFixed(measurement)));
            case BALANCED_FLOAT:
                return floatFilter->filter(measurement);
            case PRECISE_2D:
                // For 2D, assume position measurement
                AutoKalman2D::Vector2D pos(measurement, 0);
                return posFilter->filterPosition(pos, 0.1).x;
        }
        return 0;
    }
};
```

### Accuracy vs Performance Trade-offs

#### Parameter Tuning for Performance
```cpp
// High performance (less accurate)
AutoKalman fastFilter(1.0, 1.0, 1.0, 0.0);  // Default parameters

// High accuracy (slower)
AutoKalman accurateFilter(0.001, 0.01, 1.0, 0.0);  // Tight parameters

// Balanced approach
AutoKalman balancedFilter(AutoKalmanConfig::TEMPERATURE);  // Preset
```

## Hardware-Specific Issues

### AVR Microcontrollers

#### Floating-Point Limitations
**Issue**: Software floating-point emulation is slow on AVR

**Solutions**:
```cpp
// Use fixed-point for AVR boards
AutoKalmanFixed filter(AutoKalmanFixed::floatToFixed(0.1),
                      AutoKalmanFixed::floatToFixed(0.5));

// Avoid double precision
float result = filter.getStateEstimate(); // float is OK
// double result = filter.getStateEstimate(); // Avoid on AVR
```

#### PROGMEM Usage
```cpp
// Store constants in flash memory on AVR
const float filterConstants[] PROGMEM = {
    0.1, 0.5, 1.0, 0.0  // Q, R, P, initial_value
};

AutoKalman filter(
    pgm_read_float(&filterConstants[0]),
    pgm_read_float(&filterConstants[1]),
    pgm_read_float(&filterConstants[2]),
    pgm_read_float(&filterConstants[3])
);
```

### ARM Cortex-M

#### Hardware Floating-Point
**Advantage**: Native FPU for better performance

**Optimization**:
```cpp
// Use all floating-point variants
AutoKalman floatFilter(AutoKalmanConfig::ACCELEROMETER);
AutoKalman2D positionFilter(AutoKalmanConfig::GPS_2D);

// Leverage double precision if available
double highPrecisionResult = floatFilter.getStateEstimate();
```

#### Interrupt Handling
```cpp
// ARM Cortex-M interrupt safety
volatile float latestMeasurement;
AutoKalman filter;

void TIM2_IRQHandler() {
    // Read sensor in interrupt
    latestMeasurement = readADCSensor();
    // Don't call filter.filter() here - too slow for ISR
}

void loop() {
    float measurement;
    noInterrupts();
    measurement = latestMeasurement;
    interrupts();

    // Process in main loop
    float filtered = filter.filter(measurement);
}
```

### ESP8266/ESP32

#### WiFi Interference
**Issue**: WiFi transmissions can cause sensor noise spikes

**Solutions**:
```cpp
// Schedule filtering away from WiFi activity
if (!WiFi.isConnected() || millis() % 100 < 50) {
    float measurement = readSensor();
    float filtered = filter.filter(measurement);
}

// Use more aggressive filtering for noisy environments
AutoKalman robustFilter(0.5, 2.0, 1.0, 0.0);  // Higher measurement noise
```

#### FreeRTOS Integration
```cpp
// ESP32 FreeRTOS task for filtering
void filterTask(void* parameter) {
    AutoKalman filter(AutoKalmanConfig::TEMPERATURE);

    while (true) {
        float measurement = readTemperatureSensor();
        float filtered = filter.filter(measurement);

        // Send to another task or queue
        xQueueSend(sensorQueue, &filtered, portMAX_DELAY);

        vTaskDelay(100 / portTICK_PERIOD_MS); // 10 Hz
    }
}

// Create task
xTaskCreate(filterTask, "Filter", 2048, NULL, 1, NULL);
```

## Advanced Diagnostics

### Filter Health Monitoring

#### Convergence Detection
```cpp
class ConvergenceMonitor {
private:
    float previousEstimate;
    float convergenceThreshold;
    int stableSampleCount;
    int requiredStableSamples;

public:
    ConvergenceMonitor(float threshold = 0.001, int samples = 10)
        : convergenceThreshold(threshold), stableSampleCount(0),
          requiredStableSamples(samples) {}

    bool checkConvergence(float currentEstimate) {
        float change = fabs(currentEstimate - previousEstimate);

        if (change < convergenceThreshold) {
            stableSampleCount++;
        } else {
            stableSampleCount = 0;
        }

        previousEstimate = currentEstimate;
        return stableSampleCount >= requiredStableSamples;
    }

    void reset() {
        stableSampleCount = 0;
        previousEstimate = 0;
    }
};

// Usage
ConvergenceMonitor convergence(0.01, 5); // 0.01 threshold, 5 stable samples

void monitorFilter() {
    float measurement = readSensor();
    float filtered = filter.filter(measurement);

    if (convergence.checkConvergence(filtered)) {
        Serial.println("Filter has converged");
    }
}
```

### Numerical Stability Checks

#### Matrix Condition Monitoring
```cpp
class StabilityChecker {
private:
    float lastCovariance;
    float covarianceRate;
    bool stabilityWarning;

public:
    StabilityChecker() : lastCovariance(1.0), covarianceRate(0),
                        stabilityWarning(false) {}

    void checkStability(AutoKalman& filter) {
        float currentCovariance = filter.getEstimatedError();

        if (lastCovariance > 0) {
            covarianceRate = fabs(currentCovariance - lastCovariance) / lastCovariance;
        }

        // Check for numerical instability
        if (currentCovariance < 1e-10 || currentCovariance > 1e10) {
            stabilityWarning = true;
            Serial.println("Warning: Covariance out of normal range");
        }

        if (covarianceRate > 10.0) { // Rapid change
            stabilityWarning = true;
            Serial.println("Warning: Covariance changing too rapidly");
        }

        lastCovariance = currentCovariance;
    }

    bool isStable() const {
        return !stabilityWarning;
    }

    void resetWarning() {
        stabilityWarning = false;
    }
};
```

### Sensor Validation

#### Sensor Health Checking
```cpp
class SensorValidator {
private:
    float minExpectedValue;
    float maxExpectedValue;
    float maxRateOfChange;
    float lastValidValue;
    unsigned long lastValidTime;
    int consecutiveInvalidSamples;

public:
    SensorValidator(float minVal, float maxVal, float maxRate)
        : minExpectedValue(minVal), maxExpectedValue(maxVal),
          maxRateOfChange(maxRate), lastValidValue(0),
          lastValidTime(0), consecutiveInvalidSamples(0) {}

    bool isValidReading(float value, unsigned long timestamp) {
        // Range check
        if (value < minExpectedValue || value > maxExpectedValue) {
            consecutiveInvalidSamples++;
            return false;
        }

        // Rate of change check
        if (lastValidTime > 0) {
            float dt = (timestamp - lastValidTime) / 1000.0; // seconds
            if (dt > 0) {
                float rateOfChange = fabs(value - lastValidValue) / dt;
                if (rateOfChange > maxRateOfChange) {
                    consecutiveInvalidSamples++;
                    return false;
                }
            }
        }

        // Valid reading
        lastValidValue = value;
        lastValidTime = timestamp;
        consecutiveInvalidSamples = 0;
        return true;
    }

    bool isSensorHealthy() const {
        return consecutiveInvalidSamples < 5; // Allow some invalid samples
    }

    void printDiagnostics() {
        Serial.print("Sensor health: ");
        Serial.println(isSensorHealthy() ? "Good" : "Poor");
        Serial.print("Consecutive invalid: ");
        Serial.println(consecutiveInvalidSamples);
        Serial.print("Last valid value: ");
        Serial.println(lastValidValue, 3);
    }
};

// Usage example
SensorValidator tempValidator(-50, 100, 10); // -50°C to 100°C, max 10°C/s change

void validateAndFilter() {
    float rawTemp = readTemperatureSensor();
    unsigned long timestamp = millis();

    if (tempValidator.isValidReading(rawTemp, timestamp)) {
        float filteredTemp = temperatureFilter.filter(rawTemp);
        // Use filtered value
    } else {
        Serial.println("Invalid temperature reading detected");
        tempValidator.printDiagnostics();
    }
}
```

This troubleshooting guide provides comprehensive solutions for common issues, debugging techniques, and performance optimization strategies. Use the diagnostic tools and monitoring classes to identify and resolve problems with your Kalman filter implementations.

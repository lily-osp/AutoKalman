# Examples

## Table of Contents

1. [Basic Examples](#basic-examples)
   - [Simple Signal Filtering](#simple-signal-filtering)
   - [Temperature Sensor](#temperature-sensor)
   - [Distance Measurement](#distance-measurement)
2. [Intermediate Examples](#intermediate-examples)
   - [Dynamic Parameter Adjustment](#dynamic-parameter-adjustment)
   - [Multi-Sensor Integration](#multi-sensor-integration)
3. [Advanced Examples](#advanced-examples)
   - [2D Position Tracking](#2d-position-tracking)
   - [Fixed-Point Processing](#fixed-point-processing)
   - [Configuration Presets](#configuration-presets)
4. [Specialized Applications](#specialized-applications)
   - [IMU Data Fusion](#imu-data-fusion)
   - [GPS Navigation](#gps-navigation)
   - [Audio Signal Processing](#audio-signal-processing)
5. [Integration Patterns](#integration-patterns)
   - [Class-Based Implementation](#class-based-implementation)
   - [State Machine Integration](#state-machine-integration)
   - [Real-Time Processing](#real-time-processing)
6. [Debugging and Monitoring](#debugging-and-monitoring)

## Basic Examples

### Simple Signal Filtering

```cpp
/**
 * Basic Kalman filtering example demonstrating signal smoothing
 * for noisy analog sensor readings.
 */

#include <AutoKalman.h>

// Create a basic Kalman filter with default parameters
AutoKalman filter;

void setup() {
    Serial.begin(9600);
    Serial.println("Basic Kalman Filter Example");
}

void loop() {
    // Read analog sensor (replace with your sensor)
    int rawValue = analogRead(A0);
    float voltage = rawValue * (5.0 / 1023.0);

    // Apply Kalman filtering
    float filteredVoltage = filter.filter(voltage);

    // Output results
    Serial.print("Raw: ");
    Serial.print(voltage, 3);
    Serial.print(" V, Filtered: ");
    Serial.print(filteredVoltage, 3);
    Serial.println(" V");

    delay(100);
}
```

### Temperature Sensor

```cpp
/**
 * Temperature sensor filtering with appropriate noise characteristics
 * for DS18B20 or similar temperature sensors.
 */

#include <AutoKalman.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Sensor setup
#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Kalman filter with temperature-specific parameters
AutoKalman tempFilter(AutoKalmanConfig::TEMPERATURE);

void setup() {
    Serial.begin(9600);
    sensors.begin();
    Serial.println("Temperature Sensor Kalman Filter");
}

void loop() {
    // Request temperature reading
    sensors.requestTemperatures();
    float rawTemp = sensors.getTempCByIndex(0);

    // Check for valid reading
    if (rawTemp != DEVICE_DISCONNECTED_C) {
        // Apply Kalman filtering
        float filteredTemp = tempFilter.filter(rawTemp);

        // Display results
        Serial.print("Raw Temperature: ");
        Serial.print(rawTemp, 2);
        Serial.print(" °C, Filtered: ");
        Serial.print(filteredTemp, 2);
        Serial.println(" °C");

        // Optional: Calculate temperature trend
        static float lastFiltered = filteredTemp;
        float trend = filteredTemp - lastFiltered;
        lastFiltered = filteredTemp;

        Serial.print("Temperature Trend: ");
        Serial.print(trend > 0.01 ? "Rising" : trend < -0.01 ? "Falling" : "Stable");
        Serial.println();
    } else {
        Serial.println("Temperature sensor error");
    }

    delay(2000); // Temperature sensors are slow
}
```

### Distance Measurement

```cpp
/**
 * Ultrasonic distance sensor filtering with distance-specific parameters
 * for HC-SR04 or similar ultrasonic sensors.
 */

#include <AutoKalman.h>

// HC-SR04 pins
#define TRIG_PIN 9
#define ECHO_PIN 10

// Kalman filter optimized for distance measurements
AutoKalman distanceFilter(AutoKalmanConfig::DISTANCE);

float measureDistance() {
    // Trigger ultrasonic pulse
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Read echo pulse
    long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout

    // Convert to distance (cm)
    // Speed of sound = 343 m/s = 0.0343 cm/μs
    float distance = duration * 0.0343 / 2.0;

    // Return valid distance or 0 for invalid readings
    return (distance > 0 && distance < 400) ? distance : 0;
}

void setup() {
    Serial.begin(9600);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    Serial.println("Ultrasonic Distance Kalman Filter");
}

void loop() {
    float rawDistance = measureDistance();

    if (rawDistance > 0) {
        // Apply Kalman filtering
        float filteredDistance = distanceFilter.filter(rawDistance);

        // Display results
        Serial.print("Raw Distance: ");
        Serial.print(rawDistance, 1);
        Serial.print(" cm, Filtered: ");
        Serial.print(filteredDistance, 1);
        Serial.println(" cm");

        // Optional: Detect significant distance changes
        static float lastFiltered = filteredDistance;
        float change = abs(filteredDistance - lastFiltered);

        if (change > 10.0) { // Significant change threshold
            Serial.println("Significant distance change detected!");
        }
        lastFiltered = filteredDistance;
    } else {
        Serial.println("Invalid distance reading");
    }

    delay(100);
}
```

## Intermediate Examples

### Dynamic Parameter Adjustment

```cpp
/**
 * Dynamic parameter adjustment based on operating conditions
 * and sensor characteristics.
 */

#include <AutoKalman.h>

// Adaptive Kalman filter
class AdaptiveKalman {
private:
    AutoKalman filter;
    float adaptationRate;
    unsigned long sampleCount;

public:
    AdaptiveKalman(float initialQ = 1.0, float initialR = 1.0)
        : filter(initialQ, initialR, 1.0, 0.0),
          adaptationRate(0.01),
          sampleCount(0) {}

    float filterAdaptive(float measurement) {
        sampleCount++;

        // Adapt measurement noise based on measurement variance
        if (sampleCount > 10) { // Wait for filter convergence
            static float measurementHistory[10];
            static int historyIndex = 0;

            // Store measurement history
            measurementHistory[historyIndex] = measurement;
            historyIndex = (historyIndex + 1) % 10;

            // Calculate measurement variance
            if (sampleCount > 20) { // Enough samples for variance calculation
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
        }

        return filter.filter(measurement);
    }
};

// Usage example
AdaptiveKalman adaptiveFilter;

void setup() {
    Serial.begin(9600);
    Serial.println("Adaptive Kalman Filter Example");
}

void loop() {
    float measurement = analogRead(A0) * 5.0 / 1024.0;
    float filtered = adaptiveFilter.filterAdaptive(measurement);

    Serial.print("Measurement: ");
    Serial.print(measurement, 3);
    Serial.print(", Filtered: ");
    Serial.print(filtered, 3);
    Serial.print(", R: ");
    Serial.println(adaptiveFilter.getMeasurementNoise(), 6);

    delay(50);
}
```

### Multi-Sensor Integration

```cpp
/**
 * Multi-sensor integration with Kalman filtering
 * for redundant sensor configurations.
 */

#include <AutoKalman.h>

// Multi-sensor fusion class
class SensorFusion {
private:
    AutoKalman sensor1, sensor2;
    float weights[2];
    bool sensor1Active, sensor2Active;

public:
    SensorFusion(AutoKalmanParams params1, AutoKalmanParams params2)
        : sensor1(params1), sensor2(params2),
          sensor1Active(true), sensor2Active(true) {
        weights[0] = weights[1] = 0.5; // Equal weighting initially
    }

    void updateWeights(float reliability1, float reliability2) {
        float totalReliability = reliability1 + reliability2;
        if (totalReliability > 0) {
            weights[0] = reliability1 / totalReliability;
            weights[1] = reliability2 / totalReliability;
        }
    }

    float fuseMeasurements(float measurement1, float measurement2) {
        float filtered1 = sensor1Active ? sensor1.filter(measurement1) : measurement2;
        float filtered2 = sensor2Active ? sensor2.filter(measurement2) : measurement1;

        return filtered1 * weights[0] + filtered2 * weights[1];
    }

    void setSensorStatus(bool sensor1Ok, bool sensor2Ok) {
        sensor1Active = sensor1Ok;
        sensor2Active = sensor2Ok;

        // Adjust weights based on sensor status
        if (sensor1Active && sensor2Active) {
            weights[0] = weights[1] = 0.5;
        } else if (sensor1Active) {
            weights[0] = 1.0;
            weights[1] = 0.0;
        } else if (sensor2Active) {
            weights[0] = 0.0;
            weights[1] = 1.0;
        }
    }
};

// Dual temperature sensor example
SensorFusion tempFusion(AutoKalmanConfig::TEMPERATURE, AutoKalmanConfig::TEMPERATURE);

void setup() {
    Serial.begin(9600);
    Serial.println("Multi-Sensor Fusion Example");
}

void loop() {
    // Simulate two temperature sensors with different noise characteristics
    float ambientTemp = 25.0 + 2.0 * sin(millis() * 0.001);

    // Sensor 1: High accuracy, low noise
    float sensor1 = ambientTemp + random(-5, 6) * 0.1;

    // Sensor 2: Lower accuracy, higher noise
    float sensor2 = ambientTemp + random(-20, 21) * 0.1;

    // Simulate sensor reliability (0.0 to 1.0)
    float reliability1 = 0.9; // High reliability
    float reliability2 = 0.6; // Lower reliability

    // Update fusion weights
    tempFusion.updateWeights(reliability1, reliability2);

    // Fuse measurements
    float fusedTemp = tempFusion.fuseMeasurements(sensor1, sensor2);

    // Display results
    Serial.print("Sensor1: ");
    Serial.print(sensor1, 2);
    Serial.print(" °C, Sensor2: ");
    Serial.print(sensor2, 2);
    Serial.print(" °C, Fused: ");
    Serial.print(fusedTemp, 2);
    Serial.println(" °C");

    delay(200);
}
```

## Advanced Examples

### 2D Position Tracking

```cpp
/**
 * 2D position tracking using Kalman filtering
 * Suitable for GPS, optical tracking, or motion capture systems.
 */

#include <AutoKalman.h>

// GPS position tracking class
class GPSTracker {
private:
    AutoKalman2D positionFilter;
    unsigned long lastUpdate;
    bool hasFix;

public:
    GPSTracker() : positionFilter(AutoKalmanConfig::GPS_2D),
                   lastUpdate(0), hasFix(false) {}

    void updatePosition(float latitude, float longitude, float hdop = 1.0) {
        unsigned long currentTime = millis();

        if (lastUpdate > 0) {
            // Calculate time step
            float dt = (currentTime - lastUpdate) / 1000.0;

            // Adjust measurement noise based on HDOP (Horizontal Dilution of Precision)
            float measurementNoise = hdop * 2.0; // Scale HDOP to noise
            positionFilter.setMeasurementNoise(measurementNoise);

            // Create position measurement
            AutoKalman2D::Vector2D measurement(latitude, longitude);

            // Apply Kalman filter
            AutoKalman2D::Vector2D filteredPos = positionFilter.filterPosition(measurement, dt);

            // Get velocity estimate
            AutoKalman2D::Vector2D velocity = positionFilter.getVelocity();

            // Output results
            Serial.print("GPS Fix - Raw: (");
            Serial.print(latitude, 6);
            Serial.print(", ");
            Serial.print(longitude, 6);
            Serial.print("), Filtered: (");
            Serial.print(filteredPos.x, 6);
            Serial.print(", ");
            Serial.print(filteredPos.y, 6);
            Serial.print("), Velocity: (");
            Serial.print(velocity.x, 3);
            Serial.print(", ");
            Serial.print(velocity.y, 3);
            Serial.println(")");

            hasFix = true;
        }

        lastUpdate = currentTime;
    }

    bool hasValidFix() const {
        return hasFix;
    }

    AutoKalman2D::Vector2D getFilteredPosition() const {
        return positionFilter.getPosition();
    }

    void reset() {
        positionFilter.reset();
        hasFix = false;
        lastUpdate = 0;
    }
};

// GPS tracker instance
GPSTracker gpsTracker;

void setup() {
    Serial.begin(115200);
    Serial.println("GPS 2D Position Tracking Example");
}

void loop() {
    // Simulate GPS data (replace with actual GPS library calls)
    static float baseLat = 37.7749;
    static float baseLon = -122.4194;

    // Add realistic GPS noise and movement
    float noiseLat = random(-100, 101) / 100000.0; // ~1-10m noise
    float noiseLon = random(-100, 101) / 100000.0;

    // Simulate movement
    static float time = 0;
    time += 0.1;
    float movementLat = 0.001 * sin(time * 0.5); // Slow north-south movement
    float movementLon = 0.001 * cos(time * 0.3); // Slow east-west movement

    float gpsLat = baseLat + movementLat + noiseLat;
    float gpsLon = baseLon + movementLon + noiseLon;

    // Simulate HDOP (position accuracy indicator)
    float hdop = 1.0 + random(0, 50) / 100.0; // 1.0 to 1.5

    // Update GPS tracker
    gpsTracker.updatePosition(gpsLat, gpsLon, hdop);

    // Simulate GPS update rate (1 Hz typical)
    delay(1000);
}
```

### Fixed-Point Processing

```cpp
/**
 * Fixed-point Kalman filtering for memory-constrained Arduino boards
 * Demonstrates integer-only processing for AVR microcontrollers.
 */

#include <AutoKalman.h>

// Fixed-point sensor processing class
class FixedPointSensor {
private:
    AutoKalmanFixed filter;
    int sensorPin;
    AutoKalmanFixed::fixed_t scaleFactor; // Conversion factor
    AutoKalmanFixed::fixed_t offset;      // Offset value

public:
    FixedPointSensor(int pin, float scale, float offset_val,
                    AutoKalmanFixed::fixed_t q, AutoKalmanFixed::fixed_t r)
        : sensorPin(pin),
          scaleFactor(AutoKalmanFixed::floatToFixed(scale)),
          offset(AutoKalmanFixed::floatToFixed(offset_val)),
          filter(q, r, AutoKalmanFixed::floatToFixed(1.0),
                 AutoKalmanFixed::floatToFixed(offset_val)) {}

    AutoKalmanFixed::fixed_t readFixedPoint() {
        // Read analog value
        int rawValue = analogRead(sensorPin);

        // Convert to fixed-point with scaling
        AutoKalmanFixed::fixed_t fixedValue =
            AutoKalmanFixed::floatToFixed(rawValue) * scaleFactor /
            AutoKalmanFixed::floatToFixed(1024.0) + offset;

        // Apply Kalman filter
        return filter.filter(fixedValue);
    }

    float readFloat() {
        return AutoKalmanFixed::fixedToFloat(readFixedPoint());
    }
};

// Fixed-point temperature sensor (0-100°C range)
FixedPointSensor tempSensor(A0, 100.0/1024.0, 0.0,
                           AutoKalmanFixed::floatToFixed(0.001),
                           AutoKalmanFixed::floatToFixed(0.5));

void setup() {
    Serial.begin(9600);
    Serial.println("Fixed-Point Kalman Filter Example");
    Serial.println("Using 16.16 fixed-point arithmetic");
}

void loop() {
    // Read sensor using fixed-point processing
    float filteredTemp = tempSensor.readFloat();

    // Also demonstrate direct fixed-point usage
    AutoKalmanFixed::fixed_t rawFixed = AutoKalmanFixed::floatToFixed(
        analogRead(A1) * 100.0 / 1024.0);

    static AutoKalmanFixed pressureFilter(
        AutoKalmanFixed::floatToFixed(0.01),
        AutoKalmanFixed::floatToFixed(2.0),
        AutoKalmanFixed::floatToFixed(10.0),
        AutoKalmanFixed::floatToFixed(1013.25)
    );

    AutoKalmanFixed::fixed_t filteredPressure = pressureFilter.filter(rawFixed);

    // Display results
    Serial.print("Temperature: ");
    Serial.print(filteredTemp, 2);
    Serial.print(" °C, Pressure: ");
    Serial.print(AutoKalmanFixed::fixedToFloat(filteredPressure), 2);
    Serial.println(" hPa");

    // Show memory efficiency
    Serial.print("Filter memory usage: ");
    Serial.print(sizeof(AutoKalmanFixed));
    Serial.println(" bytes");

    delay(500);
}
```

### Configuration Presets

```cpp
/**
 * Comprehensive demonstration of configuration presets
 * for different sensor types and applications.
 */

#include <AutoKalman.h>

// Configuration demonstration class
class ConfigDemo {
private:
    AutoKalman temperatureFilter;
    AutoKalman pressureFilter;
    AutoKalman distanceFilter;
    AutoKalman accelerometerFilter;
    AutoKalman batteryFilter;

public:
    ConfigDemo() :
        temperatureFilter(AutoKalmanConfig::TEMPERATURE),
        pressureFilter(AutoKalmanConfig::PRESSURE),
        distanceFilter(AutoKalmanConfig::DISTANCE),
        accelerometerFilter(AutoKalmanConfig::ACCELEROMETER),
        batteryFilter(AutoKalmanConfig::BATTERY_VOLTAGE) {}

    void demonstrateFilters() {
        // Generate sample data for each sensor type
        demonstrateTemperature();
        demonstratePressure();
        demonstrateDistance();
        demonstrateAcceleration();
        demonstrateBatteryVoltage();

        Serial.println("----------------------------------------");
    }

private:
    void demonstrateTemperature() {
        float trueTemp = 25.0 + 5.0 * sin(millis() * 0.001);
        float noise = random(-200, 201) / 100.0; // ±2.0°C noise
        float measured = trueTemp + noise;
        float filtered = temperatureFilter.filter(measured);

        Serial.print("Temperature: True=");
        Serial.print(trueTemp, 1);
        Serial.print("°C, Measured=");
        Serial.print(measured, 1);
        Serial.print("°C, Filtered=");
        Serial.print(filtered, 1);
        Serial.println("°C");
    }

    void demonstratePressure() {
        float truePress = 1013.25 + 10.0 * sin(millis() * 0.0005);
        float noise = random(-500, 501) / 100.0; // ±5.0 hPa noise
        float measured = truePress + noise;
        float filtered = pressureFilter.filter(measured);

        Serial.print("Pressure: True=");
        Serial.print(truePress, 1);
        Serial.print(" hPa, Measured=");
        Serial.print(measured, 1);
        Serial.print(" hPa, Filtered=");
        Serial.print(filtered, 1);
        Serial.println(" hPa");
    }

    void demonstrateDistance() {
        float trueDist = 100.0 + 20.0 * sin(millis() * 0.0008);
        float noise = random(-200, 201) / 10.0; // ±20.0 cm noise
        float measured = trueDist + noise;
        float filtered = distanceFilter.filter(measured);

        Serial.print("Distance: True=");
        Serial.print(trueDist, 1);
        Serial.print(" cm, Measured=");
        Serial.print(measured, 1);
        Serial.print(" cm, Filtered=");
        Serial.print(filtered, 1);
        Serial.println(" cm");
    }

    void demonstrateAcceleration() {
        float trueAccel = 0.0 + 0.5 * sin(millis() * 0.002);
        float noise = random(-100, 101) / 1000.0; // ±0.1 m/s² noise
        float measured = trueAccel + noise;
        float filtered = accelerometerFilter.filter(measured);

        Serial.print("Acceleration: True=");
        Serial.print(trueAccel, 3);
        Serial.print(" m/s², Measured=");
        Serial.print(measured, 3);
        Serial.print(" m/s², Filtered=");
        Serial.print(filtered, 3);
        Serial.println(" m/s²");
    }

    void demonstrateBatteryVoltage() {
        float trueVoltage = 3.7 + 0.3 * sin(millis() * 0.0003);
        float noise = random(-50, 51) / 1000.0; // ±0.05V noise
        float measured = trueVoltage + noise;
        float filtered = batteryFilter.filter(measured);

        Serial.print("Battery: True=");
        Serial.print(trueVoltage, 2);
        Serial.print("V, Measured=");
        Serial.print(measured, 2);
        Serial.print("V, Filtered=");
        Serial.print(filtered, 2);
        Serial.println("V");
    }
};

ConfigDemo configDemo;

void setup() {
    Serial.begin(115200);
    Serial.println("AutoKalman Configuration Presets Demonstration");
    Serial.println("===========================================");
}

void loop() {
    configDemo.demonstrateFilters();
    delay(1000);
}
```

## Specialized Applications

### IMU Data Fusion

```cpp
/**
 * IMU (Inertial Measurement Unit) data fusion using Kalman filtering
 * Combines accelerometer, gyroscope, and magnetometer data.
 */

#include <AutoKalman.h>

// IMU fusion class
class IMUFusion {
private:
    AutoKalman accelX, accelY, accelZ;
    AutoKalman gyroX, gyroY, gyroZ;
    AutoKalman magX, magY, magZ;

    // Orientation estimates (simplified)
    float roll, pitch, yaw;

public:
    IMUFusion() :
        accelX(AutoKalmanConfig::ACCELEROMETER),
        accelY(AutoKalmanConfig::ACCELEROMETER),
        accelZ(AutoKalmanConfig::ACCELEROMETER),
        gyroX(AutoKalmanConfig::GYROSCOPE),
        gyroY(AutoKalmanConfig::GYROSCOPE),
        gyroZ(AutoKalmanConfig::GYROSCOPE),
        magX(AutoKalmanConfig::scaleNoise(AutoKalmanConfig::ACCELEROMETER, 0.5)),
        magY(AutoKalmanConfig::scaleNoise(AutoKalmanConfig::ACCELEROMETER, 0.5)),
        magZ(AutoKalmanConfig::scaleNoise(AutoKalmanConfig::ACCELEROMETER, 0.5)),
        roll(0), pitch(0), yaw(0) {}

    void updateIMU(float ax, float ay, float az,
                   float gx, float gy, float gz,
                   float mx, float my, float mz,
                   float dt) {

        // Filter individual sensor readings
        float fax = accelX.filter(ax);
        float fay = accelY.filter(ay);
        float faz = accelZ.filter(az);

        float fgx = gyroX.filter(gx);
        float fgy = gyroY.filter(gy);
        float fgz = gyroZ.filter(gz);

        float fmx = magX.filter(mx);
        float fmy = magY.filter(my);
        float fmz = magZ.filter(mz);

        // Simplified orientation estimation
        // In practice, use more sophisticated fusion algorithms
        roll += fgx * dt;
        pitch += fgy * dt;
        yaw += fgz * dt;

        // Normalize angles
        roll = fmod(roll, 2 * PI);
        pitch = fmod(pitch, 2 * PI);
        yaw = fmod(yaw, 2 * PI);

        // Display filtered sensor data
        Serial.print("Accel (");
        Serial.print(fax, 3); Serial.print(", ");
        Serial.print(fay, 3); Serial.print(", ");
        Serial.print(faz, 3); Serial.print("), ");

        Serial.print("Gyro (");
        Serial.print(fgx, 3); Serial.print(", ");
        Serial.print(fgy, 3); Serial.print(", ");
        Serial.print(fgz, 3); Serial.print("), ");

        Serial.print("Orientation (");
        Serial.print(degrees(roll), 1); Serial.print("°, ");
        Serial.print(degrees(pitch), 1); Serial.print("°, ");
        Serial.print(degrees(yaw), 1); Serial.println("°)");
    }
};

// MPU6050 simulation (replace with actual IMU library)
IMUFusion imuFusion;

void setup() {
    Serial.begin(115200);
    Serial.println("IMU Data Fusion Example");
}

void loop() {
    // Simulate IMU readings (replace with actual sensor data)
    float dt = 0.01; // 10ms time step

    // Accelerometer (m/s²)
    float ax = 0.0 + random(-100, 101) / 1000.0;
    float ay = 0.0 + random(-100, 101) / 1000.0;
    float az = 9.81 + random(-100, 101) / 1000.0;

    // Gyroscope (°/s)
    float gx = 1.0 * sin(millis() * 0.001) + random(-50, 51) / 100.0;
    float gy = 0.5 * cos(millis() * 0.001) + random(-50, 51) / 100.0;
    float gz = 0.0 + random(-50, 51) / 100.0;

    // Magnetometer (μT)
    float mx = 25.0 + random(-500, 501) / 100.0;
    float my = 0.0 + random(-500, 501) / 100.0;
    float mz = 45.0 + random(-500, 501) / 100.0;

    // Update IMU fusion
    imuFusion.updateIMU(ax, ay, az, gx, gy, gz, mx, my, mz, dt);

    delay(10); // 100 Hz update rate
}
```

### GPS Navigation

```cpp
/**
 * GPS navigation with Kalman filtering for improved accuracy
 * and position prediction capabilities.
 */

#include <AutoKalman.h>

// GPS Navigation class
class GPSNavigator {
private:
    AutoKalman2D positionFilter;
    float lastValidLat, lastValidLon;
    unsigned long lastValidTime;
    bool hasValidPosition;

    // Navigation parameters
    float speedThreshold;     // Minimum speed for movement detection
    float accuracyThreshold;  // Maximum acceptable position error

public:
    GPSNavigator() :
        positionFilter(AutoKalmanConfig::GPS_2D),
        lastValidLat(0), lastValidLon(0),
        lastValidTime(0), hasValidPosition(false),
        speedThreshold(0.1), accuracyThreshold(10.0) {}

    void updateGPS(float latitude, float longitude, float hdop, float speed = 0) {
        unsigned long currentTime = millis();

        // Validate GPS fix quality
        if (hdop > accuracyThreshold || latitude == 0.0 || longitude == 0.0) {
            Serial.println("GPS fix quality poor - skipping update");
            return;
        }

        if (lastValidTime > 0) {
            // Calculate time step
            float dt = (currentTime - lastValidTime) / 1000.0;

            // Adjust filter parameters based on speed
            if (speed > speedThreshold) {
                // High dynamics - increase process noise
                positionFilter.setProcessNoise(0.1);
            } else {
                // Low dynamics - decrease process noise
                positionFilter.setProcessNoise(0.01);
            }

            // Adjust measurement noise based on HDOP
            positionFilter.setMeasurementNoise(hdop * 2.0);

            // Apply Kalman filter
            AutoKalman2D::Vector2D measurement(latitude, longitude);
            AutoKalman2D::Vector2D filteredPos = positionFilter.filterPosition(measurement, dt);

            // Update valid position
            lastValidLat = filteredPos.x;
            lastValidLon = filteredPos.y;
            hasValidPosition = true;

            // Get velocity estimate
            AutoKalman2D::Vector2D velocity = positionFilter.getVelocity();

            // Calculate ground speed from velocity components
            float groundSpeed = sqrt(velocity.x * velocity.x + velocity.y * velocity.y);

            // Display navigation data
            Serial.print("GPS Nav - Pos: (");
            Serial.print(filteredPos.x, 6);
            Serial.print(", ");
            Serial.print(filteredPos.y, 6);
            Serial.print("), Speed: ");
            Serial.print(groundSpeed, 2);
            Serial.print(" m/s, HDOP: ");
            Serial.println(hdop, 1);

        } else {
            // First valid reading - initialize filter
            positionFilter.setInitialState(
                AutoKalman2D::Vector2D(latitude, longitude),
                AutoKalman2D::Vector2D(0, 0)
            );
            hasValidPosition = true;
        }

        lastValidTime = currentTime;
    }

    bool getPosition(float& latitude, float& longitude) {
        if (hasValidPosition) {
            latitude = lastValidLat;
            longitude = lastValidLon;
            return true;
        }
        return false;
    }

    void resetNavigation() {
        positionFilter.reset();
        hasValidPosition = false;
        lastValidTime = 0;
    }
};

GPSNavigator navigator;

void setup() {
    Serial.begin(115200);
    Serial.println("GPS Navigation with Kalman Filtering");
}

void loop() {
    // Simulate GPS data (replace with actual GPS module)
    static float baseLat = 37.7749;
    static float baseLon = -122.4194;

    // Simulate realistic GPS behavior
    static unsigned long startTime = millis();

    // Create movement pattern (walking speed ~1.4 m/s)
    float time = (millis() - startTime) / 1000.0;
    float distance = 1.4 * time; // meters

    // Convert to latitude/longitude approximation
    // 1 degree latitude ≈ 111,000 meters
    // 1 degree longitude ≈ 111,000 * cos(latitude) meters
    float deltaLat = distance * cos(time * 0.1) / 111000.0;
    float deltaLon = distance * sin(time * 0.1) / (111000.0 * cos(baseLat * PI / 180.0));

    float gpsLat = baseLat + deltaLat;
    float gpsLon = baseLon + deltaLon;

    // Add GPS noise
    float noiseLat = random(-200, 201) / 100000.0; // ±2m latitude noise
    float noiseLon = random(-200, 201) / 100000.0; // ±2m longitude noise

    gpsLat += noiseLat;
    gpsLon += noiseLon;

    // Simulate HDOP (1.0-5.0, lower is better)
    float hdop = 1.0 + random(0, 40) / 10.0;

    // Calculate speed (m/s)
    float speed = 1.4 + random(-20, 21) / 100.0; // ~1.4 m/s walking speed with noise

    // Update navigation
    navigator.updateGPS(gpsLat, gpsLon, hdop, speed);

    delay(1000); // GPS update rate
}
```

## Integration Patterns

### Class-Based Implementation

```cpp
/**
 * Object-oriented integration pattern for modular sensor systems
 */

#include <AutoKalman.h>

// Base sensor class
class Sensor {
protected:
    AutoKalman filter;
    int pin;
    String name;

public:
    Sensor(int sensorPin, AutoKalmanParams params, String sensorName)
        : filter(params), pin(sensorPin), name(sensorName) {}

    virtual float readRaw() = 0;
    virtual float convert(float raw) = 0;

    float readFiltered() {
        float raw = readRaw();
        float converted = convert(raw);
        return filter.filter(converted);
    }

    String getName() const { return name; }
    void reset() { filter.reset(); }
};

// Temperature sensor implementation
class TemperatureSensor : public Sensor {
public:
    TemperatureSensor(int pin) : Sensor(pin, AutoKalmanConfig::TEMPERATURE, "Temperature") {}

    float readRaw() override {
        return analogRead(pin);
    }

    float convert(float raw) override {
        return raw * 100.0 / 1024.0; // 0-100°C conversion
    }
};

// Distance sensor implementation
class DistanceSensor : public Sensor {
public:
    DistanceSensor(int pin) : Sensor(pin, AutoKalmanConfig::DISTANCE, "Distance") {}

    float readRaw() override {
        // Simplified ultrasonic reading (implement actual HC-SR04 logic)
        return analogRead(pin);
    }

    float convert(float raw) override {
        return raw * 400.0 / 1024.0; // 0-400cm conversion
    }
};

// Sensor manager class
class SensorManager {
private:
    TemperatureSensor* tempSensor;
    DistanceSensor* distanceSensor;
    unsigned long lastUpdate;

public:
    SensorManager() : tempSensor(nullptr), distanceSensor(nullptr), lastUpdate(0) {}

    void initialize() {
        tempSensor = new TemperatureSensor(A0);
        distanceSensor = new DistanceSensor(A1);
    }

    void update() {
        unsigned long currentTime = millis();
        if (currentTime - lastUpdate >= 100) {
            if (tempSensor) {
                float temp = tempSensor->readFiltered();
                Serial.print(tempSensor->getName());
                Serial.print(": ");
                Serial.print(temp, 1);
                Serial.println(" °C");
            }

            if (distanceSensor) {
                float distance = distanceSensor->readFiltered();
                Serial.print(distanceSensor->getName());
                Serial.print(": ");
                Serial.print(distance, 1);
                Serial.println(" cm");
            }

            lastUpdate = currentTime;
        }
    }

    void resetAll() {
        if (tempSensor) tempSensor->reset();
        if (distanceSensor) distanceSensor->reset();
    }
};

SensorManager sensorManager;

void setup() {
    Serial.begin(9600);
    sensorManager.initialize();
    Serial.println("Class-Based Sensor Integration");
}

void loop() {
    sensorManager.update();
    delay(50);
}
```

### Real-Time Processing

```cpp
/**
 * Real-time signal processing with timing constraints
 * and performance monitoring.
 */

#include <AutoKalman.h>

// Real-time processor class
class RealTimeProcessor {
private:
    AutoKalman filter;
    const unsigned long sampleInterval;  // microseconds
    unsigned long lastSampleTime;
    unsigned long sampleCount;
    unsigned long overrunCount;

    // Performance monitoring
    unsigned long maxProcessingTime;
    unsigned long totalProcessingTime;

public:
    RealTimeProcessor(unsigned long interval_us = 1000)  // 1ms default
        : sampleInterval(interval_us), lastSampleTime(0),
          sampleCount(0), overrunCount(0),
          maxProcessingTime(0), totalProcessingTime(0) {
        filter = AutoKalman(AutoKalmanConfig::AUDIO_SIGNAL);
    }

    void process() {
        unsigned long currentTime = micros();

        // Check if it's time for the next sample
        if (currentTime - lastSampleTime >= sampleInterval) {

            // Check for sample overrun (missed deadline)
            if (currentTime - lastSampleTime > sampleInterval * 1.5) {
                overrunCount++;
            }

            unsigned long startTime = micros();

            // Acquire sample
            float sample = acquireSample();

            // Apply filtering
            float filtered = filter.filter(sample);

            // Process result
            processSample(filtered);

            unsigned long processingTime = micros() - startTime;

            // Update performance statistics
            if (processingTime > maxProcessingTime) {
                maxProcessingTime = processingTime;
            }
            totalProcessingTime += processingTime;
            sampleCount++;

            lastSampleTime = currentTime;
        }
    }

    float acquireSample() {
        // Replace with actual ADC reading
        return analogRead(A0) * 3.3 / 1024.0;
    }

    void processSample(float filteredSample) {
        // Replace with your processing logic
        static float lastSample = 0;
        float derivative = filteredSample - lastSample;
        lastSample = filteredSample;

        // Output to DAC, serial, etc.
        Serial.print(filteredSample, 4);
        Serial.print(",");
        Serial.println(derivative, 4);
    }

    void printStatistics() {
        Serial.println("=== Real-Time Processing Statistics ===");
        Serial.print("Samples processed: ");
        Serial.println(sampleCount);
        Serial.print("Sample overruns: ");
        Serial.println(overrunCount);
        Serial.print("Max processing time: ");
        Serial.print(maxProcessingTime);
        Serial.println(" μs");
        if (sampleCount > 0) {
            Serial.print("Average processing time: ");
            Serial.print(totalProcessingTime / sampleCount);
            Serial.println(" μs");
        }
        Serial.println("====================================");
    }
};

RealTimeProcessor rtProcessor(1000); // 1kHz sampling

void setup() {
    Serial.begin(115200);
    Serial.println("Real-Time Kalman Processing Example");
}

void loop() {
    rtProcessor.process();

    // Print statistics every 10 seconds
    static unsigned long lastStats = 0;
    if (millis() - lastStats >= 10000) {
        rtProcessor.printStatistics();
        lastStats = millis();
    }
}
```

## Debugging and Monitoring

```cpp
/**
 * Debugging and monitoring utilities for Kalman filter development
 */

#include <AutoKalman.h>

// Debug monitor class
class KalmanDebugger {
private:
    AutoKalman* filter;
    String filterName;
    bool enabled;

    // Statistics
    float minValue, maxValue;
    float sumSquaredError;
    unsigned long sampleCount;
    unsigned long startTime;

public:
    KalmanDebugger(AutoKalman* kf, String name)
        : filter(kf), filterName(name), enabled(true),
          minValue(FLT_MAX), maxValue(-FLT_MAX),
          sumSquaredError(0), sampleCount(0), startTime(millis()) {}

    void update(float measurement, float filteredValue, float trueValue = NAN) {
        if (!enabled) return;

        sampleCount++;

        // Update range statistics
        if (filteredValue < minValue) minValue = filteredValue;
        if (filteredValue > maxValue) maxValue = filteredValue;

        // Calculate error if true value is available
        if (!isnan(trueValue)) {
            float error = filteredValue - trueValue;
            sumSquaredError += error * error;
        }

        // Periodic status output
        if (sampleCount % 100 == 0) {
            printStatus();
        }
    }

    void printStatus() {
        Serial.print("=== ");
        Serial.print(filterName);
        Serial.println(" Status ===");

        Serial.print("Samples: ");
        Serial.println(sampleCount);

        Serial.print("Range: [");
        Serial.print(minValue, 3);
        Serial.print(", ");
        Serial.print(maxValue, 3);
        Serial.println("]");

        if (sampleCount > 0) {
            float rmsError = sqrt(sumSquaredError / sampleCount);
            Serial.print("RMS Error: ");
            Serial.println(rmsError, 4);
        }

        Serial.print("Filter parameters - Q: ");
        Serial.print(filter->getProcessNoise(), 6);
        Serial.print(", R: ");
        Serial.print(filter->getMeasurementNoise(), 6);
        Serial.print(", P: ");
        Serial.println(filter->getEstimatedError(), 6);

        unsigned long elapsed = millis() - startTime;
        Serial.print("Elapsed time: ");
        Serial.print(elapsed / 1000.0, 1);
        Serial.println(" seconds");

        Serial.println("====================");
    }

    void reset() {
        minValue = FLT_MAX;
        maxValue = -FLT_MAX;
        sumSquaredError = 0;
        sampleCount = 0;
        startTime = millis();
    }

    void setEnabled(bool enable) {
        enabled = enable;
    }
};

// Example usage with debugging
AutoKalman voltageFilter(AutoKalmanConfig::BATTERY_VOLTAGE);
KalmanDebugger debugger(&voltageFilter, "Battery Voltage");

void setup() {
    Serial.begin(9600);
    Serial.println("Kalman Filter with Debugging");
}

void loop() {
    // Read voltage (simulate battery voltage)
    float trueVoltage = 3.7 + 0.3 * sin(millis() * 0.001);
    float noise = random(-50, 51) / 1000.0;
    float measuredVoltage = trueVoltage + noise;

    // Apply filter
    float filteredVoltage = voltageFilter.filter(measuredVoltage);

    // Update debugger
    debugger.update(measuredVoltage, filteredVoltage, trueVoltage);

    // Display current values
    Serial.print("True: ");
    Serial.print(trueVoltage, 3);
    Serial.print("V, Measured: ");
    Serial.print(measuredVoltage, 3);
    Serial.print("V, Filtered: ");
    Serial.print(filteredVoltage, 3);
    Serial.println("V");

    delay(100);
}
```

This examples documentation provides comprehensive code samples demonstrating various use cases of the AutoKalman library. Each example includes detailed comments explaining the implementation and usage patterns. For additional information about the underlying algorithms and theory, refer to the [explanation](explanation.md) and [mechanism](mechanism.md) documentation.

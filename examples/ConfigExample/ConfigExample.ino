/**
 * @file ConfigExample.ino
 * @brief Example demonstrating AutoKalman configuration presets
 *
 * This example shows how to use pre-configured parameter sets for different
 * sensor types and applications, making it easier to get started with
 * appropriate Kalman filter settings.
 */

#include <AutoKalman.h>

// Example 1: Temperature sensor with preset configuration
AutoKalman tempFilter(AutoKalmanConfig::TEMPERATURE);

// Example 2: Distance sensor with preset configuration
AutoKalman distanceFilter(AutoKalmanConfig::DISTANCE);

// Example 3: GPS position (1D component) with preset configuration
AutoKalman gpsFilter(AutoKalmanConfig::GPS_POSITION);

// Example 4: 2D GPS tracking
AutoKalman2D gps2D(AutoKalmanConfig::GPS_2D);

// Example 5: Custom configuration using utility functions
AutoKalman customFilter(AutoKalmanConfig::getParams("accelerometer"));

// Example 6: Scaled noise parameters for high-noise environment
AutoKalman noisyFilter(AutoKalmanConfig::scaleNoise(AutoKalmanConfig::DISTANCE, 2.0f));

void setup() {
    Serial.begin(115200);
    Serial.println("AutoKalman Configuration Presets Example");
    Serial.println("========================================");

    // Validate configurations
    if (AutoKalmanConfig::validateParams(AutoKalmanConfig::TEMPERATURE)) {
        Serial.println("✓ Temperature config validated");
    }

    if (AutoKalmanConfig::validate2DParams(AutoKalmanConfig::GPS_2D)) {
        Serial.println("✓ GPS 2D config validated");
    }

    Serial.println("\nFilter Configurations:");
    Serial.print("Temperature - Q: "); Serial.print(tempFilter.getProcessNoise(), 6);
    Serial.print(", R: "); Serial.println(tempFilter.getMeasurementNoise(), 6);

    Serial.print("Distance - Q: "); Serial.print(distanceFilter.getProcessNoise(), 6);
    Serial.print(", R: "); Serial.println(distanceFilter.getMeasurementNoise(), 6);

    Serial.print("GPS - Q: "); Serial.print(gpsFilter.getProcessNoise(), 6);
    Serial.print(", R: "); Serial.println(gpsFilter.getMeasurementNoise(), 6);

    Serial.println("\nStarting sensor simulation...");
}

void loop() {
    static unsigned long lastUpdate = 0;
    unsigned long currentTime = millis();

    if (currentTime - lastUpdate >= 1000) { // Update every second
        lastUpdate = currentTime;

        // Simulate sensor readings with different noise characteristics
        simulateSensors();

        delay(100); // Small delay between different sensor simulations
    }
}

void simulateSensors() {
    static int sampleCount = 0;
    sampleCount++;

    Serial.println("\n--- Sample " + String(sampleCount) + " ---");

    // 1. Temperature sensor simulation (slow changing, moderate noise)
    float trueTemp = 25.0f + 2.0f * sin(sampleCount * 0.1f);
    float tempNoise = random(-50, 50) / 100.0f; // ±0.5°C noise
    float measuredTemp = trueTemp + tempNoise;
    float filteredTemp = tempFilter.filter(measuredTemp);

    Serial.print("Temperature: True=");
    Serial.print(trueTemp, 1);
    Serial.print("°C, Measured=");
    Serial.print(measuredTemp, 1);
    Serial.print("°C, Filtered=");
    Serial.print(filteredTemp, 1);
    Serial.println("°C");

    // 2. Distance sensor simulation (moderate noise)
    float trueDistance = 100.0f + 20.0f * sin(sampleCount * 0.05f);
    float distanceNoise = random(-200, 200) / 10.0f; // ±20cm noise
    float measuredDistance = trueDistance + distanceNoise;
    float filteredDistance = distanceFilter.filter(measuredDistance);

    Serial.print("Distance: True=");
    Serial.print(trueDistance, 1);
    Serial.print("cm, Measured=");
    Serial.print(measuredDistance, 1);
    Serial.print(", Filtered=");
    Serial.print(filteredDistance, 1);
    Serial.println("cm");

    // 3. GPS position simulation (high noise, low frequency)
    float trueGPS = 1000.0f + sampleCount * 0.1f; // Slowly moving position
    float gpsNoise = random(-5000, 5000) / 10.0f; // ±50m noise
    float measuredGPS = trueGPS + gpsNoise;
    float filteredGPS = gpsFilter.filter(measuredGPS);

    Serial.print("GPS: True=");
    Serial.print(trueGPS, 1);
    Serial.print("m, Measured=");
    Serial.print(measuredGPS, 1);
    Serial.print("m, Filtered=");
    Serial.print(filteredGPS, 1);
    Serial.println("m");

    // 4. 2D position simulation (every 5 samples to simulate GPS update rate)
    if (sampleCount % 5 == 0) {
        float trueX = 100.0f + 10.0f * sin(sampleCount * 0.02f);
        float trueY = 50.0f + 5.0f * cos(sampleCount * 0.02f);

        // Add GPS-like noise
        float noiseX = random(-1000, 1000) / 10.0f; // ±10m noise
        float noiseY = random(-1000, 1000) / 10.0f;

        AutoKalman2D::Vector2D measuredPos(trueX + noiseX, trueY + noiseY);
        AutoKalman2D::Vector2D filteredPos = gps2D.filterPosition(measuredPos, 5.0f); // 5 second update

        Serial.print("GPS 2D: True=(");
        Serial.print(trueX, 1);
        Serial.print(",");
        Serial.print(trueY, 1);
        Serial.print("), Measured=(");
        Serial.print(measuredPos.x, 1);
        Serial.print(",");
        Serial.print(measuredPos.y, 1);
        Serial.print("), Filtered=(");
        Serial.print(filteredPos.x, 1);
        Serial.print(",");
        Serial.print(filteredPos.y, 1);
        Serial.println(")");
    }

    // 5. Custom accelerometer simulation
    float trueAccel = 0.0f + 0.5f * sin(sampleCount * 0.5f); // Small vibrations
    float accelNoise = random(-100, 100) / 1000.0f; // ±0.1 m/s² noise
    float measuredAccel = trueAccel + accelNoise;
    float filteredAccel = customFilter.filter(measuredAccel);

    Serial.print("Accelerometer: True=");
    Serial.print(trueAccel, 3);
    Serial.print("m/s², Measured=");
    Serial.print(measuredAccel, 3);
    Serial.print("m/s², Filtered=");
    Serial.print(filteredAccel, 3);
    Serial.println("m/s²");
}

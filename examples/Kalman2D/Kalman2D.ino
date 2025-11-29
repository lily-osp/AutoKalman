/**
 * @file Kalman2D.ino
 * @brief Example demonstrating 2D Kalman filter for position tracking
 *
 * This example shows how to use the AutoKalman2D class to track
 * position and velocity in two dimensions. Useful for GPS tracking,
 * motion sensors, or any application requiring 2D state estimation.
 */

#include <AutoKalman.h>

// Initialize 2D Kalman filter for position tracking
// Parameters: process noise, measurement noise, initial position, initial velocity
AutoKalman2D kalman2D(0.01f, 1.0f,
                     AutoKalman2D::Vector2D(0.0f, 0.0f),  // Initial position (0, 0)
                     AutoKalman2D::Vector2D(0.0f, 0.0f)); // Initial velocity (0, 0)

unsigned long lastUpdate = 0;

void setup() {
    Serial.begin(115200);
    Serial.println("2D Kalman Filter Position Tracking Example");
    Serial.println("==========================================");

    // Optional: Adjust filter parameters
    kalman2D.setProcessNoise(0.02f);    // Higher process noise for dynamic motion
    kalman2D.setMeasurementNoise(2.0f); // GPS-like measurement noise

    lastUpdate = millis();
}

void loop() {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastUpdate) / 1000.0f; // Time step in seconds
    lastUpdate = currentTime;

    // Simulate noisy position measurements (e.g., from GPS)
    // In real application, these would come from actual sensors
    float trueX = 10.0f * sin(currentTime * 0.001f); // True X position (sinusoidal motion)
    float trueY = 5.0f * cos(currentTime * 0.001f);  // True Y position

    // Add noise to simulate real sensor measurements
    float noiseX = random(-200, 200) / 100.0f; // ±2.0 units noise
    float noiseY = random(-200, 200) / 100.0f;

    float measuredX = trueX + noiseX;
    float measuredY = trueY + noiseY;

    // Create measurement vector
    AutoKalman2D::Vector2D measurement(measuredX, measuredY);

    // Apply Kalman filter
    AutoKalman2D::Vector2D filteredPosition = kalman2D.filterPosition(measurement, dt);

    // Get estimated velocity
    AutoKalman2D::Vector2D estimatedVelocity = kalman2D.getVelocity();

    // Output results every 500ms
    static unsigned long lastPrint = 0;
    if (currentTime - lastPrint >= 500) {
        Serial.print("Time: ");
        Serial.print(currentTime / 1000.0f, 1);
        Serial.println("s");

        Serial.print("True Position: (");
        Serial.print(trueX, 2);
        Serial.print(", ");
        Serial.print(trueY, 2);
        Serial.println(")");

        Serial.print("Measured: (");
        Serial.print(measuredX, 2);
        Serial.print(", ");
        Serial.print(measuredY, 2);
        Serial.println(")");

        Serial.print("Filtered: (");
        Serial.print(filteredPosition.x, 2);
        Serial.print(", ");
        Serial.print(filteredPosition.y, 2);
        Serial.println(")");

        Serial.print("Velocity: (");
        Serial.print(estimatedVelocity.x, 2);
        Serial.print(", ");
        Serial.print(estimatedVelocity.y, 2);
        Serial.println(")");

        Serial.println("------------------------------------------");
        lastPrint = currentTime;
    }

    delay(50); // 20 Hz update rate
}

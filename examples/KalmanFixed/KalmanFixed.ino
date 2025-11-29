/**
 * @file KalmanFixed.ino
 * @brief Example demonstrating fixed-point Kalman filter
 *
 * This example shows how to use the AutoKalmanFixed class for
 * resource-constrained Arduino boards. The fixed-point implementation
 * uses integer arithmetic instead of floating-point for better
 * performance and memory efficiency.
 */

#include <AutoKalman.h>

// Initialize fixed-point Kalman filter
// Parameters use fixed-point format (multiply floats by 65536 for conversion)
AutoKalmanFixed kalmanFixed(
    AutoKalmanFixed::floatToFixed(0.1f),   // Process noise: 0.1
    AutoKalmanFixed::floatToFixed(0.5f),   // Measurement noise: 0.5
    AutoKalmanFixed::floatToFixed(1.0f),   // Initial error: 1.0
    AutoKalmanFixed::floatToFixed(0.0f)    // Initial value: 0.0
);

void setup() {
    Serial.begin(115200);
    Serial.println("Fixed-Point Kalman Filter Example");
    Serial.println("=================================");
    Serial.println("Using 16.16 fixed-point arithmetic");
}

void loop() {
    // Simulate a noisy sensor reading (e.g., temperature, pressure, etc.)
    float trueValue = 25.0f + 5.0f * sin(millis() * 0.001f); // True value: 20-30 range

    // Add noise to simulate real sensor
    float noise = random(-100, 100) / 100.0f; // ±1.0 units noise
    float measuredValue = trueValue + noise;

    // Convert to fixed-point and apply filter
    AutoKalmanFixed::fixed_t fixedMeasurement = AutoKalmanFixed::floatToFixed(measuredValue);
    AutoKalmanFixed::fixed_t filteredFixed = kalmanFixed.filter(fixedMeasurement);

    // Convert back to float for display
    float filteredValue = AutoKalmanFixed::fixedToFloat(filteredFixed);

    // Display results
    Serial.print("True: ");
    Serial.print(trueValue, 2);
    Serial.print(" | Measured: ");
    Serial.print(measuredValue, 2);
    Serial.print(" | Filtered: ");
    Serial.print(filteredValue, 2);

    // Show filter parameters
    Serial.print(" | Process Noise: ");
    Serial.print(AutoKalmanFixed::fixedToFloat(kalmanFixed.getProcessNoise()), 3);
    Serial.print(" | Error Cov: ");
    Serial.println(AutoKalmanFixed::fixedToFloat(kalmanFixed.getEstimatedError()), 3);

    delay(200); // 5 Hz update rate
}

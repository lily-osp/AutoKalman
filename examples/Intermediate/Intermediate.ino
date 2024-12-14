// Intermediate Example: Dynamic adjustment of noise parameters.
#include <AutoKalman.h>

// Initialize AutoKalman with specific parameters
AutoKalman kalman(0.1, 0.1, 1.0, 0.0);

void setup() {
    Serial.begin(9600);
    kalman.setInitialValue(0.0);
}

void loop() {
    static float processNoise = 0.1;
    static float measurementNoise = 0.1;

    float noisyMeasurement = analogRead(A0) * (5.0 / 1023.0);
    float smoothedValue = kalman.filter(noisyMeasurement);

    // Dynamically adjust noise parameters
    if (millis() % 5000 == 0) { // Every 5 seconds
        processNoise += 0.01;
        measurementNoise += 0.01;
        kalman.setProcessNoise(processNoise);
        kalman.setMeasurementNoise(measurementNoise);
    }

    Serial.print("Noisy: ");
    Serial.print(noisyMeasurement);
    Serial.print(", Smoothed: ");
    Serial.print(smoothedValue);
    Serial.print(", Process Noise: ");
    Serial.print(processNoise);
    Serial.print(", Measurement Noise: ");
    Serial.println(measurementNoise);

    delay(100);
}

// Simple Example: Basic signal smoothing.
#include <AutoKalman.h>

// Initialize AutoKalman with default values
AutoKalman kalman;

void setup() {
    Serial.begin(9600);
    kalman.setInitialValue(0.0); // Start with an initial estimate
}

void loop() {
    float noisyMeasurement = analogRead(A0) * (5.0 / 1023.0); // Simulated noisy sensor
    float smoothedValue = kalman.filter(noisyMeasurement);

    Serial.print("Noisy: ");
    Serial.print(noisyMeasurement);
    Serial.print(", Smoothed: ");
    Serial.println(smoothedValue);

    delay(100);
}

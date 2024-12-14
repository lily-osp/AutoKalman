// Advanced Example: Sensor fusion with multiple Kalman filters.
#include <AutoKalman.h>

// Initialize AutoKalman for fusing two sensors
AutoKalman kalman1(0.1, 0.1, 1.0, 0.0);
AutoKalman kalman2(0.1, 0.1, 1.0, 0.0);

void setup() {
    Serial.begin(9600);
    kalman1.setInitialValue(0.0);
    kalman2.setInitialValue(0.0);
}

void loop() {
    float sensor1 = analogRead(A0) * (5.0 / 1023.0); // Sensor 1 (noisy)
    float sensor2 = analogRead(A1) * (5.0 / 1023.0); // Sensor 2 (noisy)

    float estimate1 = kalman1.filter(sensor1);
    float estimate2 = kalman2.filter(sensor2);

    // Combine estimates (simple average for demonstration)
    float fusedEstimate = (estimate1 + estimate2) / 2.0;

    Serial.print("Sensor1: ");
    Serial.print(sensor1);
    Serial.print(", Estimate1: ");
    Serial.print(estimate1);
    Serial.print(", Sensor2: ");
    Serial.print(sensor2);
    Serial.print(", Estimate2: ");
    Serial.print(estimate2);
    Serial.print(", Fused: ");
    Serial.println(fusedEstimate);

    delay(100);
}

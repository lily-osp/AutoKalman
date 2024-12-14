#ifndef AUTOKALMAN_H
#define AUTOKALMAN_H

#include <Arduino.h>

class AutoKalman {
public:
    AutoKalman(float processNoise = 1.0, float measurementNoise = 1.0, float estimatedError = 1.0, float initialValue = 0.0);

    void setProcessNoise(float q);
    void setMeasurementNoise(float r);
    void setEstimatedError(float p);
    void setInitialValue(float value);

    float filter(float measurement);
    void reset();

    float getProcessNoise() const;
    float getMeasurementNoise() const;
    float getEstimatedError() const;
    float getStateEstimate() const;

private:
    float _processNoise; // Process noise covariance (Q)
    float _measurementNoise; // Measurement noise covariance (R)
    float _estimatedError; // Error covariance (P)
    float _value; // Current state estimate (x)
    bool _isInitialized; // Flag to check if the filter is initialized
};

#endif

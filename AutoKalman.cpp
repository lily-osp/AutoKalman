#include "AutoKalman.h"

AutoKalman::AutoKalman(float processNoise, float measurementNoise, float estimatedError, float initialValue) {
    _processNoise = processNoise;
    _measurementNoise = measurementNoise;
    _estimatedError = estimatedError;
    _value = initialValue;
    _isInitialized = true;
}

void AutoKalman::setProcessNoise(float q) {
    _processNoise = q;
}

void AutoKalman::setMeasurementNoise(float r) {
    _measurementNoise = r;
}

void AutoKalman::setEstimatedError(float p) {
    _estimatedError = p;
}

void AutoKalman::setInitialValue(float value) {
    _value = value;
    _isInitialized = true;
}

float AutoKalman::filter(float measurement) {
    if (!_isInitialized) {
        _value = measurement;
        _isInitialized = true;
        return _value;
    }

    // Kalman Gain
    float kalmanGain = _estimatedError / (_estimatedError + _measurementNoise);

    // Update state estimate
    _value = _value + kalmanGain * (measurement - _value);

    // Update error covariance
    _estimatedError = (1.0f - kalmanGain) * _estimatedError + fabs(_value) * _processNoise;

    return _value;
}

void AutoKalman::reset() {
    _isInitialized = false;
    _value = 0.0;
    _estimatedError = 1.0;
}

float AutoKalman::getProcessNoise() const {
    return _processNoise;
}

float AutoKalman::getMeasurementNoise() const {
    return _measurementNoise;
}

float AutoKalman::getEstimatedError() const {
    return _estimatedError;
}

float AutoKalman::getStateEstimate() const {
    return _value;
}

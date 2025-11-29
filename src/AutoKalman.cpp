/**
 * @file AutoKalman.cpp
 * @brief Implementation of the AutoKalman library
 * @author 1999AZZAR
 * @version 1.0.0
 *
 * This file contains the implementation of the AutoKalman class methods.
 */

#include "AutoKalman.h"

AutoKalman::AutoKalman(float processNoise, float measurementNoise, float estimatedError, float initialValue) {
    // Validate and clamp parameters
    _processNoise = clamp(processNoise, MIN_NOISE, MAX_NOISE);
    _measurementNoise = clamp(measurementNoise, MIN_NOISE, MAX_NOISE);
    _estimatedError = clamp(estimatedError, MIN_ERROR, MAX_ERROR);
    _value = initialValue;
    _isInitialized = true;
}

AutoKalman::AutoKalman(const AutoKalmanConfig::KalmanParams& params) {
    // Validate and clamp parameters
    _processNoise = clamp(params.processNoise, MIN_NOISE, MAX_NOISE);
    _measurementNoise = clamp(params.measurementNoise, MIN_NOISE, MAX_NOISE);
    _estimatedError = clamp(params.initialError, MIN_ERROR, MAX_ERROR);
    _value = params.initialValue;
    _isInitialized = true;
}

void AutoKalman::setProcessNoise(float q) {
    _processNoise = clamp(q, MIN_NOISE, MAX_NOISE);
}

void AutoKalman::setMeasurementNoise(float r) {
    _measurementNoise = clamp(r, MIN_NOISE, MAX_NOISE);
}

void AutoKalman::setEstimatedError(float p) {
    _estimatedError = clamp(p, MIN_ERROR, MAX_ERROR);
}

void AutoKalman::setInitialValue(float value) {
    _value = value;
    _isInitialized = true;
}

float AutoKalman::filter(float measurement) {
    // Handle NaN or infinite input
    if (isnan(measurement) || isinf(measurement)) {
        // Return last valid estimate without updating
        return _value;
    }

    if (!_isInitialized) {
        _value = measurement;
        _isInitialized = true;
        return _value;
    }

    // Ensure measurement noise is not zero to prevent division by zero
    float safeMeasurementNoise = (_measurementNoise < MIN_NOISE) ? MIN_NOISE : _measurementNoise;

    // Calculate Kalman gain with numerical stability check
    float denominator = _estimatedError + safeMeasurementNoise;
    if (denominator < 1e-12f) {
        denominator = 1e-12f; // Prevent division by very small numbers
    }
    float kalmanGain = _estimatedError / denominator;

    // Update state estimate using Kalman gain
    _value = _value + kalmanGain * (measurement - _value);

    // Update error covariance for next iteration with bounds checking
    _estimatedError = (1.0f - kalmanGain) * _estimatedError + fabs(_value) * _processNoise;
    _estimatedError = clamp(_estimatedError, MIN_ERROR, MAX_ERROR);

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

bool AutoKalman::validateParameters(float processNoise, float measurementNoise, float estimatedError) const {
    return (processNoise >= MIN_NOISE && processNoise <= MAX_NOISE &&
            measurementNoise >= MIN_NOISE && measurementNoise <= MAX_NOISE &&
            estimatedError >= MIN_ERROR && estimatedError <= MAX_ERROR);
}

float AutoKalman::clamp(float value, float min, float max) const {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

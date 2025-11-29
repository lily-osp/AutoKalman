/**
 * @file AutoKalmanFixed.cpp
 * @brief Implementation of fixed-point Kalman filter
 * @author 1999AZZAR
 * @version 1.0.0
 */

#include "AutoKalmanFixed.h"

AutoKalmanFixed::AutoKalmanFixed(fixed_t processNoise, fixed_t measurementNoise,
                                fixed_t estimatedError, fixed_t initialValue) {
    _processNoise = clamp(processNoise, MIN_NOISE, MAX_NOISE);
    _measurementNoise = clamp(measurementNoise, MIN_NOISE, MAX_NOISE);
    _estimatedError = clamp(estimatedError, MIN_ERROR, MAX_ERROR);
    _value = initialValue;
    _isInitialized = true;
}

void AutoKalmanFixed::setProcessNoise(fixed_t q) {
    _processNoise = clamp(q, MIN_NOISE, MAX_NOISE);
}

void AutoKalmanFixed::setMeasurementNoise(fixed_t r) {
    _measurementNoise = clamp(r, MIN_NOISE, MAX_NOISE);
}

void AutoKalmanFixed::setEstimatedError(fixed_t p) {
    _estimatedError = clamp(p, MIN_ERROR, MAX_ERROR);
}

void AutoKalmanFixed::setInitialValue(fixed_t value) {
    _value = value;
    _isInitialized = true;
}

AutoKalmanFixed::fixed_t AutoKalmanFixed::filter(fixed_t measurement) {
    if (!_isInitialized) {
        _value = measurement;
        _isInitialized = true;
        return _value;
    }

    // Ensure measurement noise is not zero to prevent division by zero
    fixed_t safeMeasurementNoise = (_measurementNoise < MIN_NOISE) ? MIN_NOISE : _measurementNoise;

    // Calculate Kalman gain: K = P / (P + R)
    fixed_t denominator = _estimatedError + safeMeasurementNoise;
    fixed_t kalmanGain = fixedDiv(_estimatedError, denominator);

    // Update state estimate: x = x + K * (measurement - x)
    fixed_t innovation = measurement - _value;
    fixed_t stateUpdate = fixedMul(kalmanGain, innovation);
    _value = _value + stateUpdate;

    // Update error covariance: P = (1 - K) * P + |x| * Q
    fixed_t oneMinusK = FIXED_SCALE - kalmanGain; // 1.0 in fixed-point minus K
    fixed_t covarianceUpdate1 = fixedMul(oneMinusK, _estimatedError);

    // Calculate |x| * Q (absolute value of state times process noise)
    fixed_t absValue = (_value < 0) ? -_value : _value;
    fixed_t covarianceUpdate2 = fixedMul(absValue, _processNoise);

    _estimatedError = covarianceUpdate1 + covarianceUpdate2;
    _estimatedError = clamp(_estimatedError, MIN_ERROR, MAX_ERROR);

    return _value;
}

void AutoKalmanFixed::reset() {
    _isInitialized = false;
    _value = 0;
    _estimatedError = FIXED_SCALE; // 1.0 in fixed-point
}

AutoKalmanFixed::fixed_t AutoKalmanFixed::getProcessNoise() const {
    return _processNoise;
}

AutoKalmanFixed::fixed_t AutoKalmanFixed::getMeasurementNoise() const {
    return _measurementNoise;
}

AutoKalmanFixed::fixed_t AutoKalmanFixed::getEstimatedError() const {
    return _estimatedError;
}

AutoKalmanFixed::fixed_t AutoKalmanFixed::getStateEstimate() const {
    return _value;
}

AutoKalmanFixed::fixed_t AutoKalmanFixed::floatToFixed(float value) {
    return (fixed_t)(value * FIXED_SCALE);
}

float AutoKalmanFixed::fixedToFloat(fixed_t value) {
    return (float)value / FIXED_SCALE;
}

AutoKalmanFixed::fixed_t AutoKalmanFixed::fixedMul(fixed_t a, fixed_t b) const {
    // Fixed-point multiplication: (a * b) >> 16
    // Use 64-bit intermediate to prevent overflow
    int64_t result = (int64_t)a * (int64_t)b;
    return (fixed_t)(result >> 16);
}

AutoKalmanFixed::fixed_t AutoKalmanFixed::fixedDiv(fixed_t a, fixed_t b) const {
    // Fixed-point division: (a << 16) / b
    // Avoid division by zero
    if (b == 0) {
        return (a >= 0) ? MAX_ERROR : -MAX_ERROR;
    }

    // Use 64-bit intermediate to prevent overflow
    int64_t numerator = (int64_t)a << 16;
    return (fixed_t)(numerator / b);
}

AutoKalmanFixed::fixed_t AutoKalmanFixed::clamp(fixed_t value, fixed_t min, fixed_t max) const {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

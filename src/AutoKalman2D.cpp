/**
 * @file AutoKalman2D.cpp
 * @brief Implementation of 2D Kalman filter
 * @author 1999AZZAR
 * @version 1.0.0
 */

#include "AutoKalman2D.h"
#include <string.h> // For memcpy

AutoKalman2D::AutoKalman2D(float processNoise, float measurementNoise,
                         Vector2D initialPosition, Vector2D initialVelocity) {
    _processNoise = clamp(processNoise, MIN_NOISE, MAX_NOISE);
    _measurementNoise = clamp(measurementNoise, MIN_NOISE, MAX_NOISE);

    // Initialize state [pos_x, pos_y, vel_x, vel_y]
    _state[0] = initialPosition.x;
    _state[1] = initialPosition.y;
    _state[2] = initialVelocity.x;
    _state[3] = initialVelocity.y;

    // Initialize covariance matrix as identity with some uncertainty
    memset(_covariance, 0, sizeof(_covariance));
    for (int i = 0; i < 4; i++) {
        _covariance[i * 4 + i] = 1.0f; // Diagonal elements
    }

    _isInitialized = true;
}

AutoKalman2D::AutoKalman2D(const AutoKalmanConfig::Kalman2DParams& params) {
    _processNoise = clamp(params.processNoise, MIN_NOISE, MAX_NOISE);
    _measurementNoise = clamp(params.measurementNoise, MIN_NOISE, MAX_NOISE);

    // Initialize state [pos_x, pos_y, vel_x, vel_y]
    _state[0] = params.initialPosX;
    _state[1] = params.initialPosY;
    _state[2] = params.initialVelX;
    _state[3] = params.initialVelY;

    // Initialize covariance matrix as identity with some uncertainty
    memset(_covariance, 0, sizeof(_covariance));
    for (int i = 0; i < 4; i++) {
        _covariance[i * 4 + i] = 1.0f; // Diagonal elements
    }

    _isInitialized = true;
}

void AutoKalman2D::setProcessNoise(float q) {
    _processNoise = clamp(q, MIN_NOISE, MAX_NOISE);
}

void AutoKalman2D::setMeasurementNoise(float r) {
    _measurementNoise = clamp(r, MIN_NOISE, MAX_NOISE);
}

void AutoKalman2D::setInitialState(Vector2D position, Vector2D velocity) {
    _state[0] = position.x;
    _state[1] = position.y;
    _state[2] = velocity.x;
    _state[3] = velocity.y;

    // Reset covariance
    memset(_covariance, 0, sizeof(_covariance));
    for (int i = 0; i < 4; i++) {
        _covariance[i * 4 + i] = 1.0f;
    }

    _isInitialized = true;
}

AutoKalman2D::Vector2D AutoKalman2D::filterPosition(Vector2D measuredPosition, float dt) {
    // Handle invalid measurements
    if (isnan(measuredPosition.x) || isnan(measuredPosition.y) ||
        isinf(measuredPosition.x) || isinf(measuredPosition.y)) {
        return Vector2D(_state[0], _state[1]);
    }

    // Clamp dt to reasonable values
    dt = clamp(dt, 0.001f, 10.0f);

    // Prediction step
    // State transition: position += velocity * dt
    // Velocity remains constant (no acceleration model in this simple version)
    float predictedState[4];
    predictedState[0] = _state[0] + _state[2] * dt; // pos_x + vel_x * dt
    predictedState[1] = _state[1] + _state[3] * dt; // pos_y + vel_y * dt
    predictedState[2] = _state[2]; // vel_x unchanged
    predictedState[3] = _state[3]; // vel_y unchanged

    // Prediction covariance
    float predictedCov[16];
    // Simplified: add process noise to position and velocity variances
    memcpy(predictedCov, _covariance, sizeof(_covariance));
    predictedCov[0] += _processNoise * dt * dt; // pos_x variance
    predictedCov[5] += _processNoise * dt * dt; // pos_y variance
    predictedCov[10] += _processNoise; // vel_x variance
    predictedCov[15] += _processNoise; // vel_y variance

    // Update step - only position measurement
    // Measurement matrix H = [1, 0, 0, 0; 0, 1, 0, 0] (only measuring position)
    float innovation[2] = {
        measuredPosition.x - predictedState[0],
        measuredPosition.y - predictedState[1]
    };

    // Innovation covariance S = H * P * H^T + R
    float S[4] = {
        predictedCov[0] + _measurementNoise, predictedCov[1],
        predictedCov[4], predictedCov[5] + _measurementNoise
    };

    // Calculate Kalman gain K = P * H^T * S^-1
    float detS = S[0] * S[3] - S[1] * S[2];
    if (fabs(detS) < 1e-12f) detS = 1e-12f;

    float invS[4] = {
        S[3] / detS, -S[1] / detS,
        -S[2] / detS, S[0] / detS
    };

    // K = P * H^T * invS (simplified for position-only measurement)
    float K[8]; // 4x2 matrix
    for (int i = 0; i < 4; i++) {
        K[i] = predictedCov[i] * invS[0] + predictedCov[i + 4] * invS[2];
        K[i + 4] = predictedCov[i] * invS[1] + predictedCov[i + 4] * invS[3];
    }

    // Update state: x = x_pred + K * innovation
    for (int i = 0; i < 4; i++) {
        _state[i] = predictedState[i] + K[i] * innovation[0] + K[i + 4] * innovation[1];
    }

    // Update covariance: P = (I - K*H) * P_pred
    // Simplified update for position-only measurement
    float I_KH[16] = {0}; // Identity minus K*H
    I_KH[0] = 1.0f - K[0];
    I_KH[1] = -K[4];
    I_KH[4] = -K[1];
    I_KH[5] = 1.0f - K[5];
    I_KH[10] = 1.0f - K[2];
    I_KH[11] = -K[6];
    I_KH[14] = -K[3];
    I_KH[15] = 1.0f - K[7];

    // P = (I - K*H) * P_pred
    float temp[16] = {0};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                temp[i * 4 + j] += I_KH[i * 4 + k] * predictedCov[k * 4 + j];
            }
        }
    }
    memcpy(_covariance, temp, sizeof(_covariance));

    return Vector2D(_state[0], _state[1]);
}

AutoKalman2D::Vector2D AutoKalman2D::getPosition() const {
    return Vector2D(_state[0], _state[1]);
}

AutoKalman2D::Vector2D AutoKalman2D::getVelocity() const {
    return Vector2D(_state[2], _state[3]);
}

void AutoKalman2D::reset() {
    memset(_state, 0, sizeof(_state));
    memset(_covariance, 0, sizeof(_covariance));
    for (int i = 0; i < 4; i++) {
        _covariance[i * 4 + i] = 1.0f;
    }
    _isInitialized = false;
}

float AutoKalman2D::getProcessNoise() const {
    return _processNoise;
}

float AutoKalman2D::getMeasurementNoise() const {
    return _measurementNoise;
}

float AutoKalman2D::clamp(float value, float min, float max) const {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @file AutoKalman2D.h
 * @brief Header file for 2D Kalman filter implementation
 * @author 1999AZZAR
 * @version 1.0.0
 *
 * This file provides a 2D Kalman filter for position and velocity tracking.
 */

#ifndef AUTOKALMAN2D_H
#define AUTOKALMAN2D_H

#include <Arduino.h>
#include "AutoKalmanConfig.h"

/**
 * @class AutoKalman2D
 * @brief Two-dimensional Kalman filter for position and velocity estimation
 *
 * This class implements a 2D Kalman filter that can track both position and velocity
 * in two dimensions. It is suitable for applications like GPS tracking, motion sensing,
 * or any system where both position and velocity need to be estimated.
 */
class AutoKalman2D {
public:
    /**
     * @struct Vector2D
     * @brief 2D vector structure for position and velocity
     */
    struct Vector2D {
        float x; /**< X component */
        float y; /**< Y component */

        Vector2D(float x = 0.0f, float y = 0.0f) : x(x), y(y) {}

        Vector2D operator+(const Vector2D& other) const {
            return Vector2D(x + other.x, y + other.y);
        }

        Vector2D operator-(const Vector2D& other) const {
            return Vector2D(x - other.x, y - other.y);
        }

        Vector2D operator*(float scalar) const {
            return Vector2D(x * scalar, y * scalar);
        }
    };

    /**
     * @brief Construct a new 2D Kalman filter
     *
     * @param processNoise Process noise covariance. Default is 0.01
     * @param measurementNoise Measurement noise covariance. Default is 1.0
     * @param initialPosition Initial position estimate
     * @param initialVelocity Initial velocity estimate
     */
    AutoKalman2D(float processNoise = 0.01f, float measurementNoise = 1.0f,
                 Vector2D initialPosition = Vector2D(0.0f, 0.0f),
                 Vector2D initialVelocity = Vector2D(0.0f, 0.0f));

    /**
     * @brief Construct a new 2D Kalman filter using parameter structure
     *
     * @param params Kalman2DParams structure containing filter parameters
     */
    AutoKalman2D(const AutoKalmanConfig::Kalman2DParams& params);

    /**
     * @brief Set the process noise covariance
     * @param q Process noise covariance value
     */
    void setProcessNoise(float q);

    /**
     * @brief Set the measurement noise covariance
     * @param r Measurement noise covariance value
     */
    void setMeasurementNoise(float r);

    /**
     * @brief Set the initial state (position and velocity)
     * @param position Initial position
     * @param velocity Initial velocity
     */
    void setInitialState(Vector2D position, Vector2D velocity);

    /**
     * @brief Apply the Kalman filter to new position measurements
     * @param measuredPosition The measured position
     * @param dt Time step in seconds
     * @return Filtered position estimate
     */
    Vector2D filterPosition(Vector2D measuredPosition, float dt);

    /**
     * @brief Get the current position estimate
     * @return Current position estimate
     */
    Vector2D getPosition() const;

    /**
     * @brief Get the current velocity estimate
     * @return Current velocity estimate
     */
    Vector2D getVelocity() const;

    /**
     * @brief Reset the filter to initial state
     */
    void reset();

    /**
     * @brief Get the current process noise
     * @return Process noise value
     */
    float getProcessNoise() const;

    /**
     * @brief Get the current measurement noise
     * @return Measurement noise value
     */
    float getMeasurementNoise() const;

private:
    /**
     * @brief Clamp a value within specified bounds
     * @param value The value to clamp
     * @param min Minimum allowed value
     * @param max Maximum allowed value
     * @return Clamped value
     */
    float clamp(float value, float min, float max) const;

    // State vector [position_x, position_y, velocity_x, velocity_y]
    float _state[4];

    // State covariance matrix (4x4 flattened)
    float _covariance[16];

    float _processNoise;     /**< Process noise covariance */
    float _measurementNoise; /**< Measurement noise covariance */
    bool _isInitialized;     /**< Initialization flag */

    // Constants for parameter validation
    static constexpr float MIN_NOISE = 1e-6f;    /**< Minimum allowed noise value */
    static constexpr float MAX_NOISE = 1e6f;     /**< Maximum allowed noise value */
    static constexpr float MIN_COVARIANCE = 1e-6f; /**< Minimum covariance */
    static constexpr float MAX_COVARIANCE = 1e6f;  /**< Maximum covariance */
};

#endif // AUTOKALMAN2D_H

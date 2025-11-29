/**
 * @file AutoKalman.h
 * @brief Main header file for the AutoKalman library
 * @author 1999AZZAR
 * @version 1.0.0
 *
 * This library provides implementations of Kalman filters for Arduino platforms.
 * Includes 1D and 2D Kalman filters for state estimation and sensor fusion.
 */

#ifndef AUTOKALMAN_H
#define AUTOKALMAN_H

#include <Arduino.h>
#include "AutoKalman2D.h"
#include "AutoKalmanFixed.h"
#include "AutoKalmanConfig.h"

/**
 * @class AutoKalman
 * @brief A simplified Kalman filter implementation for Arduino
 *
 * This class implements a one-dimensional Kalman filter that can be used for
 * sensor fusion and state estimation in embedded systems. The filter automatically
 * handles initialization and provides methods for tuning the filter parameters.
 */
class AutoKalman {
public:
    /**
     * @brief Construct a new AutoKalman filter object
     *
     * @param processNoise Process noise covariance (Q). Default is 1.0
     * @param measurementNoise Measurement noise covariance (R). Default is 1.0
     * @param estimatedError Initial error covariance (P). Default is 1.0
     * @param initialValue Initial state estimate. Default is 0.0
     */
    AutoKalman(float processNoise = 1.0, float measurementNoise = 1.0, float estimatedError = 1.0, float initialValue = 0.0);

    /**
     * @brief Construct a new AutoKalman filter object using parameter structure
     *
     * @param params KalmanParams structure containing filter parameters
     */
    AutoKalman(const AutoKalmanConfig::KalmanParams& params);

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
     * @brief Set the estimated error covariance
     * @param p Error covariance value
     */
    void setEstimatedError(float p);

    /**
     * @brief Set the initial state estimate value
     * @param value Initial state estimate
     */
    void setInitialValue(float value);

    /**
     * @brief Apply the Kalman filter to a new measurement
     * @param measurement The new measurement value
     * @return The filtered state estimate
     */
    float filter(float measurement);

    /**
     * @brief Reset the filter to its initial state
     */
    void reset();

    /**
     * @brief Get the current process noise covariance
     * @return Current process noise covariance value
     */
    float getProcessNoise() const;

    /**
     * @brief Get the current measurement noise covariance
     * @return Current measurement noise covariance value
     */
    float getMeasurementNoise() const;

    /**
     * @brief Get the current error covariance
     * @return Current error covariance value
     */
    float getEstimatedError() const;

    /**
     * @brief Get the current state estimate
     * @return Current state estimate value
     */
    float getStateEstimate() const;

private:
    /**
     * @brief Validate filter parameters
     * @param processNoise Process noise covariance
     * @param measurementNoise Measurement noise covariance
     * @param estimatedError Error covariance
     * @return true if parameters are valid, false otherwise
     */
    bool validateParameters(float processNoise, float measurementNoise, float estimatedError) const;

    /**
     * @brief Clamp a value within specified bounds
     * @param value The value to clamp
     * @param min Minimum allowed value
     * @param max Maximum allowed value
     * @return Clamped value
     */
    float clamp(float value, float min, float max) const;

    float _processNoise;     /**< Process noise covariance (Q) */
    float _measurementNoise; /**< Measurement noise covariance (R) */
    float _estimatedError;   /**< Error covariance (P) */
    float _value;            /**< Current state estimate (x) */
    bool _isInitialized;     /**< Initialization flag */

    // Constants for parameter validation
    static constexpr float MIN_NOISE = 1e-6f;    /**< Minimum allowed noise value */
    static constexpr float MAX_NOISE = 1e6f;     /**< Maximum allowed noise value */
    static constexpr float MIN_ERROR = 1e-6f;    /**< Minimum allowed error covariance */
    static constexpr float MAX_ERROR = 1e6f;     /**< Maximum allowed error covariance */
};

#endif // AUTOKALMAN_H

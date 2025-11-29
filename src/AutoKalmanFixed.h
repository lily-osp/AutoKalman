/**
 * @file AutoKalmanFixed.h
 * @brief Header file for fixed-point Kalman filter implementation
 * @author 1999AZZAR
 * @version 1.0.0
 *
 * This file provides a fixed-point arithmetic Kalman filter for memory-constrained
 * Arduino boards without hardware floating-point support.
 */

#ifndef AUTOKALMANFIXED_H
#define AUTOKALMANFIXED_H

#include <Arduino.h>

/**
 * @class AutoKalmanFixed
 * @brief Fixed-point Kalman filter for resource-constrained Arduino boards
 *
 * This class implements a Kalman filter using fixed-point arithmetic (16.16 format)
 * instead of floating-point operations. This reduces memory usage and improves
 * performance on Arduino boards without FPU hardware.
 */
class AutoKalmanFixed {
public:
    /**
     * @brief Fixed-point data type (16.16 format)
     * Represents values from -32768.0 to 32767.999... with 1/65536 precision
     */
    typedef int32_t fixed_t;

    /**
     * @brief Construct a new fixed-point Kalman filter
     *
     * @param processNoise Process noise covariance (as fixed-point)
     * @param measurementNoise Measurement noise covariance (as fixed-point)
     * @param estimatedError Initial error covariance (as fixed-point)
     * @param initialValue Initial state estimate (as fixed-point)
     */
    AutoKalmanFixed(fixed_t processNoise = 65536, fixed_t measurementNoise = 65536,
                   fixed_t estimatedError = 65536, fixed_t initialValue = 0);

    /**
     * @brief Set the process noise covariance
     * @param q Process noise covariance value (fixed-point)
     */
    void setProcessNoise(fixed_t q);

    /**
     * @brief Set the measurement noise covariance
     * @param r Measurement noise covariance value (fixed-point)
     */
    void setMeasurementNoise(fixed_t r);

    /**
     * @brief Set the estimated error covariance
     * @param p Error covariance value (fixed-point)
     */
    void setEstimatedError(fixed_t p);

    /**
     * @brief Set the initial state estimate value
     * @param value Initial state estimate (fixed-point)
     */
    void setInitialValue(fixed_t value);

    /**
     * @brief Apply the Kalman filter to a new measurement
     * @param measurement The new measurement value (fixed-point)
     * @return The filtered state estimate (fixed-point)
     */
    fixed_t filter(fixed_t measurement);

    /**
     * @brief Reset the filter to its initial state
     */
    void reset();

    /**
     * @brief Get the current process noise covariance
     * @return Current process noise covariance value (fixed-point)
     */
    fixed_t getProcessNoise() const;

    /**
     * @brief Get the current measurement noise covariance
     * @return Current measurement noise covariance value (fixed-point)
     */
    fixed_t getMeasurementNoise() const;

    /**
     * @brief Get the current error covariance
     * @return Current error covariance value (fixed-point)
     */
    fixed_t getEstimatedError() const;

    /**
     * @brief Get the current state estimate
     * @return Current state estimate value (fixed-point)
     */
    fixed_t getStateEstimate() const;

    /**
     * @brief Convert float to fixed-point
     * @param value Float value to convert
     * @return Fixed-point representation
     */
    static fixed_t floatToFixed(float value);

    /**
     * @brief Convert fixed-point to float
     * @param value Fixed-point value to convert
     * @return Float representation
     */
    static float fixedToFloat(fixed_t value);

private:
    /**
     * @brief Fixed-point multiplication with overflow protection
     * @param a First operand
     * @param b Second operand
     * @return Product (a * b) >> 16
     */
    fixed_t fixedMul(fixed_t a, fixed_t b) const;

    /**
     * @brief Fixed-point division with overflow protection
     * @param a Numerator
     * @param b Denominator
     * @return Quotient (a << 16) / b
     */
    fixed_t fixedDiv(fixed_t a, fixed_t b) const;

    /**
     * @brief Clamp a fixed-point value within specified bounds
     * @param value The value to clamp
     * @param min Minimum allowed value
     * @param max Maximum allowed value
     * @return Clamped value
     */
    fixed_t clamp(fixed_t value, fixed_t min, fixed_t max) const;

    fixed_t _processNoise;     /**< Process noise covariance (Q) */
    fixed_t _measurementNoise; /**< Measurement noise covariance (R) */
    fixed_t _estimatedError;   /**< Error covariance (P) */
    fixed_t _value;            /**< Current state estimate (x) */
    bool _isInitialized;       /**< Initialization flag */

    // Constants for parameter validation
    static constexpr fixed_t MIN_NOISE = 1;           /**< Minimum allowed noise value */
    static constexpr fixed_t MAX_NOISE = 2147483647;  /**< Maximum allowed noise value */
    static constexpr fixed_t MIN_ERROR = 1;           /**< Minimum allowed error covariance */
    static constexpr fixed_t MAX_ERROR = 2147483647;  /**< Maximum allowed error covariance */

    // Fixed-point constants
    static constexpr int32_t FIXED_SCALE = 65536;     /**< Fixed-point scaling factor (2^16) */
    static constexpr int32_t FIXED_MASK = 0xFFFF;     /**< Lower 16 bits mask */
};

#endif // AUTOKALMANFIXED_H

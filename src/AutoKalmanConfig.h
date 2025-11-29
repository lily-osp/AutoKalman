/**
 * @file AutoKalmanConfig.h
 * @brief Configuration presets for AutoKalman library
 * @author 1999AZZAR
 * @version 1.0.0
 *
 * This file provides pre-configured parameter sets for common Kalman filter
 * applications, making it easier to get started with appropriate settings.
 */

#ifndef AUTOKALMANCONFIG_H
#define AUTOKALMANCONFIG_H

#include <Arduino.h>

/**
 * @namespace AutoKalmanConfig
 * @brief Namespace containing pre-configured Kalman filter parameters
 *
 * This namespace provides optimized parameter sets for different sensor types
 * and applications. These presets are based on typical noise characteristics
 * and can serve as good starting points for tuning.
 */
namespace AutoKalmanConfig {

    /**
     * @struct KalmanParams
     * @brief Structure containing Kalman filter parameters
     */
    struct KalmanParams {
        float processNoise;      /**< Process noise covariance (Q) */
        float measurementNoise;  /**< Measurement noise covariance (R) */
        float initialError;      /**< Initial error covariance (P) */
        float initialValue;      /**< Initial state estimate */

        KalmanParams(float q = 1.0f, float r = 1.0f, float p = 1.0f, float x = 0.0f)
            : processNoise(q), measurementNoise(r), initialError(p), initialValue(x) {}
    };

    /**
     * @struct Kalman2DParams
     * @brief Structure containing 2D Kalman filter parameters
     */
    struct Kalman2DParams {
        float processNoise;      /**< Process noise covariance (Q) */
        float measurementNoise;  /**< Measurement noise covariance (R) */
        float initialPosX;       /**< Initial X position */
        float initialPosY;       /**< Initial Y position */
        float initialVelX;       /**< Initial X velocity */
        float initialVelY;       /**< Initial Y velocity */

        Kalman2DParams(float q = 0.01f, float r = 1.0f,
                      float px = 0.0f, float py = 0.0f,
                      float vx = 0.0f, float vy = 0.0f)
            : processNoise(q), measurementNoise(r),
              initialPosX(px), initialPosY(py),
              initialVelX(vx), initialVelY(vy) {}
    };

    // ============================================================================
    // PRESETS FOR COMMON APPLICATIONS
    // ============================================================================

    /**
     * @brief Temperature sensor filtering
     * Slow-changing signal with moderate noise
     */
    const KalmanParams TEMPERATURE = {0.001f, 0.5f, 1.0f, 25.0f};

    /**
     * @brief Pressure/altitude sensor filtering
     * Atmospheric pressure measurements
     */
    const KalmanParams PRESSURE = {0.01f, 2.0f, 10.0f, 1013.25f};

    /**
     * @brief Distance/proximity sensor filtering
     * Ultrasonic or infrared distance measurements
     */
    const KalmanParams DISTANCE = {0.1f, 5.0f, 25.0f, 0.0f};

    /**
     * @brief Accelerometer filtering (single axis)
     * High-frequency vibration filtering
     */
    const KalmanParams ACCELEROMETER = {0.1f, 0.05f, 1.0f, 0.0f};

    /**
     * @brief Gyroscope filtering (single axis)
     * Angular rate measurements with drift
     */
    const KalmanParams GYROSCOPE = {0.001f, 0.01f, 0.1f, 0.0f};

    /**
     * @brief Battery voltage monitoring
     * Slow-changing DC voltage with ripple
     */
    const KalmanParams BATTERY_VOLTAGE = {0.0001f, 0.02f, 0.1f, 3.7f};

    /**
     * @brief Light sensor (LDR/photoresistor)
     * Variable lighting conditions
     */
    const KalmanParams LIGHT_SENSOR = {0.01f, 10.0f, 100.0f, 512.0f};

    /**
     * @brief GPS position filtering (1D component)
     * High noise, low-frequency position updates
     */
    const KalmanParams GPS_POSITION = {0.001f, 25.0f, 100.0f, 0.0f};

    /**
     * @brief Audio signal processing
     * Real-time audio filtering
     */
    const KalmanParams AUDIO_SIGNAL = {0.5f, 0.1f, 1.0f, 0.0f};

    /**
     * @brief Motor speed control feedback
     * Encoder or tachometer filtering
     */
    const KalmanParams MOTOR_SPEED = {0.05f, 1.0f, 5.0f, 0.0f};

    // ============================================================================
    // 2D KALMAN FILTER PRESETS
    // ============================================================================

    /**
     * @brief GPS position tracking (2D)
     * Latitude/longitude or local coordinates
     */
    const Kalman2DParams GPS_2D = {0.001f, 25.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    /**
     * @brief Robot navigation (2D)
     * Position tracking in robot coordinate system
     */
    const Kalman2DParams ROBOT_NAVIGATION = {0.01f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f};

    /**
     * @brief Touch screen position tracking
     * Finger or stylus position smoothing
     */
    const Kalman2DParams TOUCH_SCREEN = {0.1f, 2.0f, 320.0f, 240.0f, 0.0f, 0.0f};

    /**
     * @brief Object tracking (computer vision)
     * 2D object position in image coordinates
     */
    const Kalman2DParams OBJECT_TRACKING = {0.05f, 10.0f, 320.0f, 240.0f, 0.0f, 0.0f};

    /**
     * @brief IMU-based motion tracking
     * Accelerometer integration for position
     */
    const Kalman2DParams IMU_MOTION = {0.1f, 0.2f, 0.0f, 0.0f, 0.0f, 0.0f};

    // ============================================================================
    // UTILITY FUNCTIONS
    // ============================================================================

    /**
     * @brief Get Kalman parameters by application type
     * @param application String identifier for the application
     * @return KalmanParams structure with appropriate settings
     *
     * Example usage:
     * @code
     * AutoKalman filter(AutoKalmanConfig::getParams("temperature"));
     * @endcode
     */
    KalmanParams getParams(const char* application);

    /**
     * @brief Get 2D Kalman parameters by application type
     * @param application String identifier for the application
     * @return Kalman2DParams structure with appropriate settings
     */
    Kalman2DParams get2DParams(const char* application);

    /**
     * @brief Create custom parameters with noise scaling
     * @param base Base parameter set
     * @param noiseMultiplier Multiplier for noise parameters
     * @return Modified parameter set
     */
    KalmanParams scaleNoise(const KalmanParams& base, float noiseMultiplier);

    /**
     * @brief Validate parameter ranges
     * @param params Parameters to validate
     * @return true if parameters are within valid ranges
     */
    bool validateParams(const KalmanParams& params);

    /**
     * @brief Validate 2D parameter ranges
     * @param params Parameters to validate
     * @return true if parameters are within valid ranges
     */
    bool validate2DParams(const Kalman2DParams& params);

} // namespace AutoKalmanConfig

#endif // AUTOKALMANCONFIG_H

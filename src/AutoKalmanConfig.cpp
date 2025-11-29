/**
 * @file AutoKalmanConfig.cpp
 * @brief Implementation of AutoKalman configuration utilities
 * @author 1999AZZAR
 * @version 1.0.0
 */

#include "AutoKalmanConfig.h"

namespace AutoKalmanConfig {

    KalmanParams getParams(const char* application) {
        if (strcmp(application, "temperature") == 0) return TEMPERATURE;
        if (strcmp(application, "pressure") == 0) return PRESSURE;
        if (strcmp(application, "distance") == 0) return DISTANCE;
        if (strcmp(application, "accelerometer") == 0) return ACCELEROMETER;
        if (strcmp(application, "gyroscope") == 0) return GYROSCOPE;
        if (strcmp(application, "battery") == 0) return BATTERY_VOLTAGE;
        if (strcmp(application, "light") == 0) return LIGHT_SENSOR;
        if (strcmp(application, "gps") == 0) return GPS_POSITION;
        if (strcmp(application, "audio") == 0) return AUDIO_SIGNAL;
        if (strcmp(application, "motor") == 0) return MOTOR_SPEED;

        // Default parameters if application not recognized
        return KalmanParams(1.0f, 1.0f, 1.0f, 0.0f);
    }

    Kalman2DParams get2DParams(const char* application) {
        if (strcmp(application, "gps") == 0) return GPS_2D;
        if (strcmp(application, "robot") == 0) return ROBOT_NAVIGATION;
        if (strcmp(application, "touch") == 0) return TOUCH_SCREEN;
        if (strcmp(application, "object") == 0) return OBJECT_TRACKING;
        if (strcmp(application, "imu") == 0) return IMU_MOTION;

        // Default 2D parameters if application not recognized
        return Kalman2DParams(0.01f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }

    KalmanParams scaleNoise(const KalmanParams& base, float noiseMultiplier) {
        return KalmanParams(
            base.processNoise * noiseMultiplier,
            base.measurementNoise * noiseMultiplier,
            base.initialError,
            base.initialValue
        );
    }

    bool validateParams(const KalmanParams& params) {
        const float MIN_NOISE = 1e-6f;
        const float MAX_NOISE = 1e6f;

        return (params.processNoise >= MIN_NOISE && params.processNoise <= MAX_NOISE &&
                params.measurementNoise >= MIN_NOISE && params.measurementNoise <= MAX_NOISE &&
                params.initialError >= MIN_NOISE && params.initialError <= MAX_NOISE);
    }

    bool validate2DParams(const Kalman2DParams& params) {
        const float MIN_NOISE = 1e-6f;
        const float MAX_NOISE = 1e6f;

        return (params.processNoise >= MIN_NOISE && params.processNoise <= MAX_NOISE &&
                params.measurementNoise >= MIN_NOISE && params.measurementNoise <= MAX_NOISE);
    }

} // namespace AutoKalmanConfig

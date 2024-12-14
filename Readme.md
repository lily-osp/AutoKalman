# AutoKalman Library

The AutoKalman library provides an easy-to-use interface for implementing Kalman filters in Arduino projects. It simplifies the process of setting up and tuning a Kalman filter, making it ideal for applications involving sensor fusion, signal smoothing, and state estimation.

## Features

- **Ease of Use**: Simplifies the setup of a Kalman filter with default parameters.
- **Configurable**: Allows dynamic adjustment of process noise, measurement noise, and error covariance.
- **State Estimation**: Accurately predicts the system state based on noisy measurements.
- **Reset Capability**: Enables reinitialization of the filter for dynamic systems.
- **Getter Methods**: Provides access to current filter parameters and state estimates.

## Installation

1. Download the library as a ZIP file
2. In the Arduino IDE, go to Sketch > Include Library > Add .ZIP Library
3. Select the downloaded ZIP file
4. Restart the Arduino IDE

## API Reference

### Constructor
```cpp
AutoKalman(float processNoise = 1.0, float measurementNoise = 1.0, float estimatedError = 1.0, float initialValue = 0.0);
```
- `processNoise`: The process noise covariance (Q).
- `measurementNoise`: The measurement noise covariance (R).
- `estimatedError`: The initial estimate of error covariance (P).
- `initialValue`: The initial state estimate.

### Methods

#### `void setProcessNoise(float q)`
Sets the process noise covariance.

#### `void setMeasurementNoise(float r)`
Sets the measurement noise covariance.

#### `void setEstimatedError(float p)`
Sets the error covariance.

#### `void setInitialValue(float value)`
Sets the initial value of the state estimate.

#### `float filter(float measurement)`
Applies the Kalman filter to a new measurement and returns the updated state estimate.

#### `void reset()`
Reinitializes the filter, clearing its state.

#### `float getProcessNoise() const`
Returns the current process noise covariance.

#### `float getMeasurementNoise() const`
Returns the current measurement noise covariance.

#### `float getEstimatedError() const`
Returns the current error covariance.

#### `float getStateEstimate() const`
Returns the current state estimate.

## Applications

The AutoKalman library can be used in a variety of projects, including but not limited to:

- **Signal Smoothing**: Reducing noise in sensor readings (e.g., temperature, pressure, or distance sensors).
- **Sensor Fusion**: Combining data from multiple sensors for a more accurate system state (e.g., combining gyroscope and accelerometer data).
- **Robotics**: Estimating the position or velocity of a robot in noisy environments.
- **Navigation**: Tracking objects or vehicles with noisy GPS or IMU data.

## Examples

This library includes simple, intermediate, and advanced examples to help you get started. The examples cover:

- Basic signal smoothing.
- Dynamic noise adjustment.
- Multi-sensor data fusion.

Refer to the `examples` directory in the library for more details.

## License

This library is open-source and distributed under the MIT License. You are free to use, modify, and distribute it in your projects.

## Contributions

Contributions are welcome! If you encounter any issues or have suggestions for improvement, feel free to submit a pull request or open an issue in the repository.

---

With AutoKalman, integrating Kalman filters into your Arduino projects has never been easier. Start building smarter, more reliable systems today!


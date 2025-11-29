# AutoKalman Library

A comprehensive Kalman filtering library for Arduino that provides multiple implementations optimized for different use cases and hardware constraints. From simple 1D filtering to advanced 2D position tracking, with support for both floating-point and fixed-point arithmetic.

## Features

### Core Features
- **Multiple Filter Types**: 1D, 2D, and fixed-point Kalman filter implementations
- **Robust Error Handling**: Comprehensive parameter validation and numerical stability checks
- **Hardware Optimization**: Fixed-point arithmetic for memory-constrained microcontrollers
- **Easy Configuration**: Simple API with sensible defaults and runtime parameter adjustment
- **State Management**: Built-in initialization, reset, and state persistence capabilities

### Advanced Capabilities
- **2D Position Tracking**: Simultaneous position and velocity estimation in two dimensions
- **Sensor Fusion Ready**: Designed for combining multiple sensor inputs
- **Performance Optimized**: Efficient algorithms suitable for real-time applications
- **Memory Efficient**: Fixed-point version uses minimal RAM and processing power

### Quality Assurance
- **Professional Documentation**: Doxygen-style comments throughout
- **Comprehensive Examples**: Multiple example sketches demonstrating different use cases
- **Syntax Highlighting**: Arduino IDE keywords support for better code editing experience

## Installation

### Arduino IDE Installation
1. Download the library as a ZIP file from the [GitHub repository](https://github.com/1999AZZAR/AutoKalman)
2. Open Arduino IDE
3. Go to **Sketch > Include Library > Add .ZIP Library**
4. Select the downloaded ZIP file
5. Restart Arduino IDE if necessary

### Manual Installation
1. Extract the library files to: `~/Documents/Arduino/libraries/AutoKalman/`
2. Ensure the folder structure matches Arduino library standards
3. Restart Arduino IDE

### PlatformIO Installation
Add to your `platformio.ini`:
```ini
lib_deps =
    https://github.com/1999AZZAR/AutoKalman.git
```

## API Reference

### AutoKalman (1D Floating-Point Filter)

#### Constructor
```cpp
AutoKalman(float processNoise = 1.0, float measurementNoise = 1.0, float estimatedError = 1.0, float initialValue = 0.0);
```

#### Core Methods
```cpp
float filter(float measurement);           // Apply filter and get state estimate
void reset();                              // Reset filter state
void setProcessNoise(float q);             // Set process noise covariance (Q)
void setMeasurementNoise(float r);         // Set measurement noise covariance (R)
void setEstimatedError(float p);           // Set error covariance (P)
void setInitialValue(float value);         // Set initial state estimate

// Getters
float getProcessNoise() const;             // Get current process noise
float getMeasurementNoise() const;         // Get current measurement noise
float getEstimatedError() const;           // Get current error covariance
float getStateEstimate() const;            // Get current state estimate
```

### AutoKalman2D (2D Position/Velocity Filter)

#### Constructor
```cpp
AutoKalman2D(float processNoise = 0.01f, float measurementNoise = 1.0f,
             Vector2D initialPosition = Vector2D(0,0), Vector2D initialVelocity = Vector2D(0,0));
```

#### Core Methods
```cpp
Vector2D filterPosition(Vector2D measuredPosition, float dt);  // Filter position measurement
void reset();                                                  // Reset filter state
void setInitialState(Vector2D position, Vector2D velocity);    // Set initial state

// Getters
Vector2D getPosition() const;                                  // Get current position estimate
Vector2D getVelocity() const;                                  // Get current velocity estimate
float getProcessNoise() const;                                 // Get process noise
float getMeasurementNoise() const;                             // Get measurement noise
```

#### Vector2D Structure
```cpp
struct Vector2D {
    float x, y;
    Vector2D(float x = 0.0f, float y = 0.0f);
    Vector2D operator+(const Vector2D& other) const;
    Vector2D operator-(const Vector2D& other) const;
    Vector2D operator*(float scalar) const;
};
```

### AutoKalmanFixed (Fixed-Point Filter)

#### Constructor
```cpp
AutoKalmanFixed(fixed_t processNoise, fixed_t measurementNoise, fixed_t estimatedError, fixed_t initialValue);
```

#### Core Methods
```cpp
fixed_t filter(fixed_t measurement);        // Apply filter (fixed-point)
void reset();                               // Reset filter state
void setProcessNoise(fixed_t q);            // Set process noise (fixed-point)
void setMeasurementNoise(fixed_t r);        // Set measurement noise (fixed-point)
void setEstimatedError(fixed_t p);          // Set error covariance (fixed-point)
void setInitialValue(fixed_t value);        // Set initial state (fixed-point)

// Getters
fixed_t getProcessNoise() const;            // Get process noise (fixed-point)
fixed_t getMeasurementNoise() const;        // Get measurement noise (fixed-point)
fixed_t getEstimatedError() const;          // Get error covariance (fixed-point)
fixed_t getStateEstimate() const;           // Get state estimate (fixed-point)

// Conversion utilities
static fixed_t floatToFixed(float value);   // Convert float to fixed-point
static float fixedToFloat(fixed_t value);   // Convert fixed-point to float
```

#### Fixed-Point Format
- Uses 16.16 fixed-point arithmetic (16 integer bits, 16 fractional bits)
- Range: -32768.0 to 32767.999... with 1/65536 precision
- Ideal for Arduino boards without hardware floating-point unit

## Applications

### Perfect For:
- **IoT Sensor Networks**: Reliable data filtering for environmental monitoring
- **Robotics & Drones**: Position and velocity estimation for autonomous navigation
- **Automotive Projects**: Sensor fusion for vehicle tracking and stability control
- **Industrial Control**: Process variable estimation and noise reduction
- **Wearable Devices**: Motion tracking with accelerometer/gyroscope data
- **GPS Applications**: Position smoothing and accuracy improvement
- **Audio Processing**: Signal denoising and feature extraction

### Use Case Examples:
- **Temperature Monitoring**: Filter noisy thermocouple readings
- **Distance Sensing**: Improve ultrasonic or infrared sensor accuracy
- **Motion Detection**: Track object position and velocity in 2D space
- **IMU Fusion**: Combine accelerometer, gyroscope, and magnetometer data
- **Battery Monitoring**: Estimate SOC (State of Charge) with noise filtering
- **Pressure Sensing**: Filter barometric pressure for altitude estimation

## Examples

The library includes comprehensive examples demonstrating different implementations:

### Basic Examples
- **Simple**: Basic 1D signal filtering with default parameters
- **Intermediate**: Dynamic parameter adjustment and multiple sensors

### Advanced Examples
- **Advanced**: Multi-sensor fusion with the 1D Kalman filter
- **Kalman2D**: 2D position and velocity tracking for GPS-like applications
- **KalmanFixed**: Fixed-point filtering for resource-constrained Arduino boards

### Example Usage Patterns:
```cpp
// 1D Signal Filtering
AutoKalman filter(0.1, 0.5, 1.0, 0.0);
float filtered = filter.filter(rawSensorReading);

// 2D Position Tracking
AutoKalman2D tracker(0.01, 1.0);
Vector2D position = tracker.filterPosition(measuredPos, timeStep);

// Fixed-Point for Low-Memory Boards
AutoKalmanFixed fixedFilter(AutoKalmanFixed::floatToFixed(0.1),
                           AutoKalmanFixed::floatToFixed(0.5));
fixed_t result = fixedFilter.filter(AutoKalmanFixed::floatToFixed(sensorValue));
```

Each example includes detailed comments and can be used as a starting point for your projects.

## Performance & Compatibility

### Memory Usage (Approximate)
- **AutoKalman (1D)**: ~24 bytes RAM + stack usage
- **AutoKalman2D**: ~100 bytes RAM for state and covariance matrices
- **AutoKalmanFixed**: ~20 bytes RAM (more efficient than floating-point)

### Processing Time (Arduino Uno @ 16MHz)
- **AutoKalman**: ~50-100 μs per filter update
- **AutoKalman2D**: ~200-400 μs per filter update
- **AutoKalmanFixed**: ~30-60 μs per filter update (faster due to integer math)

### Hardware Compatibility
- **Full Support**: Arduino Uno, Mega, Nano, Leonardo, Micro
- **Recommended for Fixed-Point**: ATmega328-based boards, ATtiny series
- **All Architectures**: AVR, SAMD, ESP8266, ESP32, Teensy, etc.

### Limitations
- 2D filter assumes constant velocity model (no acceleration modeling)
- Fixed-point precision: ±0.000015 (1/65536) resolution
- Maximum covariance values limited by fixed_t range

## Library Architecture

```
AutoKalman/
├── src/                    # Source files
│   ├── AutoKalman.h       # Main 1D filter header
│   ├── AutoKalman.cpp     # 1D filter implementation
│   ├── AutoKalman2D.h     # 2D filter header
│   ├── AutoKalman2D.cpp   # 2D filter implementation
│   ├── AutoKalmanFixed.h  # Fixed-point filter header
│   └── AutoKalmanFixed.cpp # Fixed-point implementation
├── examples/              # Example sketches
├── keywords.txt           # Arduino IDE syntax highlighting
├── library.properties     # Arduino library metadata
└── README.md             # This documentation
```

## License

This library is open-source software distributed under the **MIT License**. You are free to use, modify, and distribute it in your projects, both commercial and non-commercial.

```
Copyright (c) 2025 1999AZZAR

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## Support & Contributions

### Getting Help
- 📖 **Documentation**: Comprehensive API docs and examples included
- 💬 **Issues**: Report bugs or request features on [GitHub Issues](https://github.com/1999AZZAR/AutoKalman/issues)
- 📧 **Contact**: azzar.mr.zs@gmail.com

### Contributing
We welcome contributions! Here's how you can help:

1. **Bug Reports**: Use the issue tracker to report problems
2. **Feature Requests**: Suggest new features or improvements
3. **Code Contributions**: Submit pull requests with enhancements
4. **Documentation**: Improve docs, add examples, or translate

### Development Guidelines
- Follow Arduino library standards and best practices
- Include comprehensive documentation for new features
- Add unit tests for new functionality
- Maintain backward compatibility where possible
- Test on multiple Arduino boards when possible

### Testing
The library has been tested on:
- Arduino Uno (ATmega328P)
- Arduino Mega (ATmega2560)
- ESP32 development boards
- Various AVR-based Arduino variants

---

**Ready to build more reliable Arduino projects?** The AutoKalman library provides the tools you need for professional-grade sensor fusion and state estimation. From simple signal filtering to complex 2D tracking, start building smarter embedded systems today!

⭐ **Star the repository** if you find this library useful!


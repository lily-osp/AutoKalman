# System Architecture

## Table of Contents

1. [Library Overview](#library-overview)
   - [Design Philosophy](#design-philosophy)
   - [Architecture Principles](#architecture-principles)
   - [Component Structure](#component-structure)
2. [Core Components](#core-components)
   - [AutoKalman Class](#autoklman-class)
   - [AutoKalman2D Class](#autoklman2d-class)
   - [AutoKalmanFixed Class](#autoklmanfixed-class)
   - [AutoKalmanConfig Namespace](#autoklmanconfig-namespace)
3. [Data Structures](#data-structures)
   - [State Representation](#state-representation)
   - [Configuration Structures](#configuration-structures)
   - [Vector Types](#vector-types)
4. [Algorithm Implementation](#algorithm-implementation)
   - [Filter Algorithms](#filter-algorithms)
   - [Numerical Methods](#numerical-methods)
   - [Optimization Techniques](#optimization-techniques)
5. [Memory Management](#memory-management)
   - [Static Allocation Strategy](#static-allocation-strategy)
   - [Memory Layout](#memory-layout)
   - [Resource Constraints](#resource-constraints)
6. [Interface Design](#interface-design)
   - [API Consistency](#api-consistency)
   - [Error Handling](#error-handling)
   - [Configuration Interface](#configuration-interface)
7. [Extensibility Mechanisms](#extensibility-mechanisms)
   - [Inheritance Patterns](#inheritance-patterns)
   - [Template Specializations](#template-specializations)
   - [Plugin Architecture](#plugin-architecture)

## Library Overview

### Design Philosophy

The AutoKalman library is designed with the following core principles:

#### Simplicity and Accessibility
- **Single-header includes** for easy integration
- **Default constructors** with sensible parameters
- **Preset configurations** for common applications
- **Consistent API** across all filter variants

#### Performance and Efficiency
- **Minimal memory footprint** for embedded systems
- **Optimized algorithms** for real-time operation
- **Fixed-point arithmetic** option for resource-constrained platforms
- **Deterministic execution times** for predictable behavior

#### Robustness and Reliability
- **Comprehensive error checking** and validation
- **Numerical stability** safeguards
- **Graceful degradation** under adverse conditions
- **Extensive testing** across multiple platforms

#### Modularity and Extensibility
- **Component separation** for independent use
- **Configuration-driven design** for customization
- **Inheritance-friendly architecture** for specialization
- **Template-based implementation** for type safety

### Architecture Principles

#### Separation of Concerns
- **Algorithm implementation** separated from configuration
- **Platform-specific code** isolated from core logic
- **Testing infrastructure** independent of production code
- **Documentation** maintained separately from implementation

#### Layered Architecture
```
┌─────────────────┐
│  Application    │  ← User code, examples, integration
├─────────────────┤
│ Configuration   │  ← Presets, parameter management
├─────────────────┤
│ Filter Variants │  ← AutoKalman, AutoKalman2D, AutoKalmanFixed
├─────────────────┤
│ Core Algorithms │  ← Kalman filtering mathematics
├─────────────────┤
│ Platform Layer  │  ← Arduino-specific adaptations
└─────────────────┘
```

#### Dependency Management
- **Minimal external dependencies** (only Arduino.h)
- **Self-contained implementations** with no external libraries
- **Forward-compatible design** for future Arduino versions
- **Platform abstraction** for cross-platform portability

### Component Structure

```
AutoKalman/
├── src/
│   ├── AutoKalman.h           # Main 1D filter interface
│   ├── AutoKalman.cpp         # 1D filter implementation
│   ├── AutoKalman2D.h         # 2D filter interface
│   ├── AutoKalman2D.cpp       # 2D filter implementation
│   ├── AutoKalmanFixed.h      # Fixed-point filter interface
│   ├── AutoKalmanFixed.cpp    # Fixed-point implementation
│   ├── AutoKalmanConfig.h     # Configuration presets
│   └── AutoKalmanConfig.cpp   # Configuration utilities
├── examples/                   # Usage examples
├── docs/                       # Documentation
├── keywords.txt               # Arduino IDE syntax highlighting
├── library.properties         # Arduino library metadata
└── README.md                  # Main documentation
```

## Core Components

### AutoKalman Class

#### Class Hierarchy
```
AutoKalman
├── Private Members
│   ├── State variables (processNoise, measurementNoise, etc.)
│   ├── Internal state (value, estimatedError, isInitialized)
│   └── Utility methods (clamp, validateParameters)
├── Public Interface
│   ├── Constructors (default, parameterized)
│   ├── Configuration methods (setProcessNoise, etc.)
│   ├── Processing methods (filter)
│   └── Accessor methods (getProcessNoise, etc.)
└── Static Constants
    └── Parameter limits and validation constants
```

#### Key Design Decisions
- **Single responsibility**: Focused on 1D state estimation
- **Immutable interface**: No mutable state exposure
- **Parameter validation**: All inputs checked and clamped
- **Memory efficiency**: Minimal member variables

### AutoKalman2D Class

#### Architecture Extensions
```
AutoKalman2D extends AutoKalman concepts to 2D space:
├── Vector2D nested class for 2D operations
├── 4-element state vector [pos_x, pos_y, vel_x, vel_y]
├── 4x4 covariance matrix for full state uncertainty
├── Time-based prediction with velocity integration
└── Position-only measurement updates
```

#### Implementation Considerations
- **Matrix operations**: Custom 4x4 matrix handling
- **Numerical stability**: Enhanced algorithms for 2D filtering
- **Memory management**: Efficient storage of covariance matrices
- **Performance optimization**: Vectorized operations where possible

### AutoKalmanFixed Class

#### Fixed-Point Architecture
```
AutoKalmanFixed uses 16.16 fixed-point arithmetic:
├── int32_t fixed_t type definition
├── Fixed-point arithmetic operations
│   ├── Multiplication (fixedMul)
│   ├── Division (fixedDiv)
│   └── Conversion utilities
├── AVR-optimized algorithms
└── Memory-efficient storage
```

#### Precision Management
- **16.16 format**: 16 integer + 16 fractional bits
- **Range**: -32768.0 to 32767.999...
- **Resolution**: 1/65536 ≈ 0.000015
- **Overflow protection**: Saturation arithmetic

### AutoKalmanConfig Namespace

#### Configuration Architecture
```
AutoKalmanConfig provides application-specific tuning:
├── Predefined parameter sets
│   ├── TEMPERATURE, PRESSURE, DISTANCE, etc.
│   └── GPS_2D, ROBOT_NAVIGATION, etc.
├── Utility functions
│   ├── getParams() - Retrieve by application name
│   ├── validateParams() - Parameter validation
│   └── scaleNoise() - Dynamic parameter adjustment
└── Type-safe interfaces
    └── KalmanParams and Kalman2DParams structures
```

## Data Structures

### State Representation

#### 1D Filter State
```cpp
class AutoKalman {
private:
    float _processNoise;      // Q: Process noise covariance
    float _measurementNoise;  // R: Measurement noise covariance
    float _estimatedError;    // P: Error covariance estimate
    float _value;            // x: State estimate
    bool _isInitialized;     // Initialization flag
};
```

#### 2D Filter State
```cpp
class AutoKalman2D {
private:
    float _state[4];         // [pos_x, pos_y, vel_x, vel_y]
    float _covariance[16];   // 4x4 covariance matrix (flattened)
    float _processNoise;     // Q: Process noise scalar
    float _measurementNoise; // R: Measurement noise scalar
    bool _isInitialized;    // Initialization flag
};
```

#### Fixed-Point State
```cpp
class AutoKalmanFixed {
private:
    fixed_t _processNoise;    // Q: Process noise (fixed-point)
    fixed_t _measurementNoise;// R: Measurement noise (fixed-point)
    fixed_t _estimatedError;  // P: Error covariance (fixed-point)
    fixed_t _value;          // x: State estimate (fixed-point)
    bool _isInitialized;     // Initialization flag
};
```

### Configuration Structures

#### Parameter Structures
```cpp
namespace AutoKalmanConfig {
    struct KalmanParams {
        float processNoise;
        float measurementNoise;
        float initialError;
        float initialValue;
    };

    struct Kalman2DParams {
        float processNoise;
        float measurementNoise;
        float initialPosX, initialPosY;
        float initialVelX, initialVelY;
    };
}
```

### Vector Types

#### 2D Vector Implementation
```cpp
class AutoKalman2D {
public:
    struct Vector2D {
        float x, y;

        Vector2D(float x = 0.0f, float y = 0.0f) : x(x), y(y) {}

        Vector2D operator+(const Vector2D& other) const {
            return Vector2D(x + other.x, y + other.y);
        }
        // Additional operators...
    };
};
```

## Algorithm Implementation

### Filter Algorithms

#### Prediction Phase
```cpp
// 1D Prediction
void predict() {
    // For 1D, prediction is implicit in the update step
    // Error covariance evolves with process noise
    _estimatedError += _processNoise;
}

// 2D Prediction
void predict2D(float dt) {
    // Position prediction: pos += vel * dt
    _state[0] += _state[2] * dt;  // pos_x += vel_x * dt
    _state[1] += _state[3] * dt;  // pos_y += vel_y * dt

    // Covariance prediction with process noise
    // Complex matrix operations for 4x4 covariance
}
```

#### Update Phase
```cpp
// Kalman Gain Calculation
float calculateGain() {
    float denominator = _estimatedError + _measurementNoise;
    return _estimatedError / denominator;
}

// State Update
float updateState(float measurement, float gain) {
    float innovation = measurement - _value;
    _value += gain * innovation;
    return _value;
}

// Covariance Update
void updateCovariance(float gain) {
    _estimatedError *= (1.0f - gain);
    _estimatedError += _processNoise;  // Process noise addition
}
```

### Numerical Methods

#### Stability Safeguards
```cpp
// Prevent division by zero
float safeDenominator(float value) {
    const float MIN_VALUE = 1e-12f;
    return (fabs(value) < MIN_VALUE) ? MIN_VALUE : value;
}

// Clamp parameters to valid ranges
float clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}
```

#### Fixed-Point Arithmetic
```cpp
// Fixed-point multiplication with overflow protection
fixed_t fixedMul(fixed_t a, fixed_t b) {
    int64_t result = (int64_t)a * (int64_t)b;
    return (fixed_t)(result >> 16);
}

// Fixed-point division
fixed_t fixedDiv(fixed_t a, fixed_t b) {
    if (b == 0) return 0;  // Error handling
    int64_t numerator = (int64_t)a << 16;
    return (fixed_t)(numerator / b);
}
```

### Optimization Techniques

#### Memory Optimization
- **Static allocation** of all data structures
- **No dynamic memory** allocation in normal operation
- **Minimal stack usage** for recursive operations
- **Compact data representation** for fixed-point types

#### Computational Optimization
- **Pre-computed constants** where possible
- **Loop unrolling** for small fixed-size operations
- **Early termination** conditions for convergence
- **Platform-specific optimizations** for AVR vs ARM

## Memory Management

### Static Allocation Strategy

#### Object Lifetime Management
```cpp
// Preferred: Static allocation
void loop() {
    static AutoKalman filter(AutoKalmanConfig::TEMPERATURE);
    float measurement = readSensor();
    float filtered = filter.filter(measurement);
}

// Avoid: Dynamic allocation in time-critical code
AutoKalman* filter = new AutoKalman();  // Not recommended
```

#### Memory Pool Pattern
```cpp
class FilterPool {
private:
    static const int MAX_FILTERS = 4;
    AutoKalman filters[MAX_FILTERS];

public:
    AutoKalman* allocate() {
        for (int i = 0; i < MAX_FILTERS; i++) {
            if (!allocated[i]) {
                allocated[i] = true;
                return &filters[i];
            }
        }
        return nullptr;
    }
};
```

### Memory Layout

#### SRAM Usage Breakdown
| Component | Float (bytes) | Fixed (bytes) | 2D Float (bytes) |
|-----------|---------------|---------------|------------------|
| State variables | 16 | 16 | 64 |
| Covariance | 4 | 4 | 64 |
| Parameters | 12 | 16 | 8 |
| Flags | 1 | 1 | 1 |
| **Total** | **33** | **37** | **137** |

#### Flash Memory Usage
| Component | Flash (bytes) |
|-----------|---------------|
| Core algorithms | ~2KB |
| 2D extensions | ~3KB |
| Fixed-point math | ~1KB |
| Configuration | ~4KB |
| **Total** | **~10KB** |

### Resource Constraints

#### Arduino Uno (ATmega328P) Limits
- **SRAM**: 2KB total, leaving ~1.5KB for user code with Kalman filters
- **Flash**: 32KB total, ~22KB available after bootloader
- **EEPROM**: 1KB available for persistent storage
- **Stack**: ~1KB available, careful with deep recursion

#### Optimization Strategies
- **Single precision**: Use float instead of double
- **Fixed-point**: For memory-critical applications
- **Selective compilation**: Include only needed components
- **Memory pools**: Reuse filter instances

## Interface Design

### API Consistency

#### Method Naming Convention
```cpp
// Configuration methods
void setProcessNoise(float q);      // Setter pattern
void setMeasurementNoise(float r);
void setEstimatedError(float p);
void setInitialValue(float value);

// Processing methods
float filter(float measurement);    // Main processing method

// Accessor methods
float getProcessNoise() const;      // Getter pattern
float getMeasurementNoise() const;
float getEstimatedError() const;
float getStateEstimate() const;

// Control methods
void reset();                       // Reset to initial state
```

#### Parameter Order Consistency
```cpp
// Constructor parameters follow logical order
AutoKalman(float processNoise,     // Q first
           float measurementNoise, // R second
           float estimatedError,   // P third
           float initialValue);    // x fourth
```

### Error Handling

#### Parameter Validation
```cpp
bool validateParameters(float q, float r, float p) {
    const float MIN_NOISE = 1e-6f;
    const float MAX_NOISE = 1e6f;

    return (q >= MIN_NOISE && q <= MAX_NOISE &&
            r >= MIN_NOISE && r <= MAX_NOISE &&
            p >= MIN_NOISE && p <= MAX_NOISE);
}
```

#### Runtime Error Handling
```cpp
float AutoKalman::filter(float measurement) {
    // Handle invalid inputs
    if (isnan(measurement) || isinf(measurement)) {
        return _value;  // Return last valid estimate
    }

    // Ensure initialization
    if (!_isInitialized) {
        _value = measurement;
        _isInitialized = true;
        return _value;
    }

    // Continue with normal processing...
}
```

### Configuration Interface

#### Preset-Based Configuration
```cpp
// Method 1: Direct preset usage
AutoKalman tempFilter(AutoKalmanConfig::TEMPERATURE);

// Method 2: Named lookup
AutoKalman customFilter(AutoKalmanConfig::getParams("temperature"));

// Method 3: Parameter modification
AutoKalman noisyFilter(AutoKalmanConfig::scaleNoise(
    AutoKalmanConfig::DISTANCE, 2.0f));
```

## Extensibility Mechanisms

### Inheritance Patterns

#### Base Class Design
```cpp
class KalmanFilterBase {
protected:
    virtual void predict() = 0;
    virtual void update(float measurement) = 0;

public:
    virtual float filter(float measurement) = 0;
    virtual void reset() = 0;
};

// Implementation classes
class AutoKalman : public KalmanFilterBase {
    // 1D implementation
};

class AutoKalman2D : public KalmanFilterBase {
    // 2D implementation (extended interface)
};
```

### Template Specializations

#### Type-Safe Implementations
```cpp
// Template for different numeric types
template<typename T>
class KalmanFilter {
    T processNoise;
    T measurementNoise;
    // ... implementation
};

// Specializations for different precisions
template<> class KalmanFilter<float>;     // Standard precision
template<> class KalmanFilter<double>;    // High precision
template<> class KalmanFilter<fixed_t>;   // Fixed-point
```

### Plugin Architecture

#### Modular Component System
```cpp
// Interface for pluggable components
class KalmanComponent {
public:
    virtual bool initialize() = 0;
    virtual void process(float input, float& output) = 0;
    virtual void configure(const char* config) = 0;
};

// Example: Adaptive noise estimator plugin
class AdaptiveNoiseEstimator : public KalmanComponent {
    float estimatedNoise;
    float adaptationRate;

public:
    void process(float input, float& output) override {
        // Adaptive noise estimation logic
        estimatedNoise = estimatedNoise * (1 - adaptationRate) +
                        fabs(input - output) * adaptationRate;
    }
};
```

This system architecture documentation provides a comprehensive view of the AutoKalman library's design principles, implementation details, and extensibility mechanisms. The modular architecture ensures maintainability while the performance optimizations make it suitable for resource-constrained embedded systems.

# Mechanism

## Table of Contents

1. [Algorithm Overview](#algorithm-overview)
   - [Kalman Filter Mathematics](#kalman-filter-mathematics)
   - [Implementation Variants](#implementation-variants)
   - [Computational Flow](#computational-flow)
2. [1D Kalman Filter Implementation](#1d-kalman-filter-implementation)
   - [State Prediction](#state-prediction)
   - [Measurement Update](#measurement-update)
   - [Numerical Considerations](#numerical-considerations)
3. [2D Kalman Filter Implementation](#2d-kalman-filter-implementation)
   - [State Space Model](#state-space-model)
   - [Matrix Operations](#matrix-operations)
   - [Time Integration](#time-integration)
4. [Fixed-Point Arithmetic](#fixed-point-arithmetic)
   - [16.16 Fixed-Point Format](#1616-fixed-point-format)
   - [Arithmetic Operations](#arithmetic-operations)
   - [Precision Analysis](#precision-analysis)
5. [Numerical Stability](#numerical-stability)
   - [Covariance Matrix Handling](#covariance-matrix-handling)
   - [Singularity Prevention](#singularity-prevention)
   - [Overflow Protection](#overflow-protection)
6. [Optimization Techniques](#optimization-techniques)
   - [Memory Optimization](#memory-optimization)
   - [Computational Optimization](#computational-optimization)
   - [Platform-Specific Optimizations](#platform-specific-optimizations)

## Algorithm Overview

### Kalman Filter Mathematics

The Kalman filter implements optimal recursive state estimation using the following mathematical framework:

#### State Space Representation
```
Process model:    x[k+1] = A·x[k] + w[k]
Measurement model: z[k] = H·x[k] + v[k]
```

Where:
- `x[k]`: State vector at time step k
- `z[k]`: Measurement vector
- `w[k]`: Process noise (Q covariance)
- `v[k]`: Measurement noise (R covariance)
- `A`: State transition matrix
- `H`: Measurement matrix

#### Recursive Estimation
```
Prediction:  x̂[k|k-1] = A·x̂[k-1|k-1]
             P[k|k-1] = A·P[k-1|k-1]·A^T + Q

Update:      K[k] = P[k|k-1]·H^T·(H·P[k|k-1]·H^T + R)^(-1)
             x̂[k|k] = x̂[k|k-1] + K[k]·(z[k] - H·x̂[k|k-1])
             P[k|k] = (I - K[k]·H)·P[k|k-1]
```

### Implementation Variants

#### Linear Kalman Filter (AutoKalman)
- Single state variable estimation
- Simplest implementation with minimal computational requirements
- Suitable for scalar sensor measurements

#### 2D Kalman Filter (AutoKalman2D)
- Four-dimensional state vector [position_x, position_y, velocity_x, velocity_y]
- Constant velocity motion model
- Position-only measurements with time-based prediction

#### Fixed-Point Kalman Filter (AutoKalmanFixed)
- Integer-based arithmetic using 16.16 fixed-point representation
- Optimized for microcontrollers without hardware floating-point
- Deterministic execution time and memory usage

### Computational Flow

#### Initialization Phase
```cpp
// Set initial conditions
x[0] = initial_estimate
P[0] = initial_covariance
initialized = true
```

#### Prediction Phase
```cpp
// Time update (prediction)
predicted_state = state_transition * current_state
predicted_covariance = state_transition * current_covariance * state_transition^T + process_noise
```

#### Update Phase
```cpp
// Measurement update (correction)
innovation = measurement - measurement_matrix * predicted_state
innovation_covariance = measurement_matrix * predicted_covariance * measurement_matrix^T + measurement_noise
kalman_gain = predicted_covariance * measurement_matrix^T * innovation_covariance^(-1)

updated_state = predicted_state + kalman_gain * innovation
updated_covariance = (I - kalman_gain * measurement_matrix) * predicted_covariance
```

## 1D Kalman Filter Implementation

### State Prediction

For the 1D case, the state prediction simplifies to maintaining the current state estimate:

```cpp
void predict() {
    // In 1D case, state prediction is implicit
    // The state remains constant until measurement update
    // Covariance evolves with process noise
    _estimatedError += fabs(_value) * _processNoise;
}
```

The process noise is added proportionally to the absolute value of the state estimate to account for state-dependent uncertainty.

### Measurement Update

The measurement update implements the core Kalman filter equations:

```cpp
float AutoKalman::filter(float measurement) {
    // Input validation
    if (isnan(measurement) || isinf(measurement)) {
        return _value;  // Return last valid estimate
    }

    // Initialization on first call
    if (!_isInitialized) {
        _value = measurement;
        _isInitialized = true;
        return _value;
    }

    // Kalman gain calculation: K = P / (P + R)
    float denominator = _estimatedError + _measurementNoise;
    if (denominator < 1e-12f) {
        denominator = 1e-12f;  // Prevent division by near-zero
    }
    float kalmanGain = _estimatedError / denominator;

    // State update: x = x + K * (measurement - x)
    float innovation = measurement - _value;
    _value = _value + kalmanGain * innovation;

    // Covariance update: P = (1 - K) * P + Q
    _estimatedError = (1.0f - kalmanGain) * _estimatedError;
    _estimatedError += fabs(_value) * _processNoise;

    // Clamp covariance to prevent numerical issues
    _estimatedError = clamp(_estimatedError, MIN_ERROR, MAX_ERROR);

    return _value;
}
```

### Numerical Considerations

#### Stability Bounds
- **Process noise**: Clamped between 1e-6 and 1e6
- **Measurement noise**: Clamped between 1e-6 and 1e6
- **Error covariance**: Clamped between 1e-6 and 1e6

#### Singularity Prevention
```cpp
// Prevent division by zero or near-zero values
float safeDivide(float numerator, float denominator) {
    const float MIN_DENOMINATOR = 1e-12f;
    if (fabs(denominator) < MIN_DENOMINATOR) {
        denominator = (denominator >= 0) ? MIN_DENOMINATOR : -MIN_DENOMINATOR;
    }
    return numerator / denominator;
}
```

## 2D Kalman Filter Implementation

### State Space Model

The 2D Kalman filter uses a constant velocity model:

#### State Vector
```
x = [position_x, position_y, velocity_x, velocity_y]^T
```

#### State Transition Matrix A
```
A = | 1  0  dt  0  |    (position_x += velocity_x * dt)
    | 0  1  0   dt |    (position_y += velocity_y * dt)
    | 0  0  1   0  |    (velocity_x constant)
    | 0  0  0   1  |    (velocity_y constant)
```

#### Measurement Matrix H
```
H = | 1  0  0  0  |    (measure position_x)
    | 0  1  0  0  |    (measure position_y)
```

### Matrix Operations

#### Matrix Multiplication (4x4 * 4x4)
```cpp
void matrixMultiply4x4(const float a[16], const float b[16], float result[16]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            result[i * 4 + j] = 0;
            for (int k = 0; k < 4; k++) {
                result[i * 4 + j] += a[i * 4 + k] * b[k * 4 + j];
            }
        }
    }
}
```

#### Matrix Transpose
```cpp
void matrixTranspose4x4(const float matrix[16], float result[16]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            result[i * 4 + j] = matrix[j * 4 + i];
        }
    }
}
```

#### Matrix Inversion (2x2 for measurement covariance)
```cpp
bool matrixInverse2x2(const float matrix[4], float result[4]) {
    float det = matrix[0] * matrix[3] - matrix[1] * matrix[2];

    if (fabs(det) < 1e-12f) {
        // Matrix is singular, use pseudoinverse approximation
        det = 1e-12f;
    }

    float invDet = 1.0f / det;

    result[0] =  matrix[3] * invDet;
    result[1] = -matrix[1] * invDet;
    result[2] = -matrix[2] * invDet;
    result[3] =  matrix[0] * invDet;

    return true;
}
```

### Time Integration

The 2D filter incorporates time-based prediction:

```cpp
void AutoKalman2D::filterPosition(Vector2D measuredPosition, float dt) {
    // Clamp time step to reasonable range
    dt = clamp(dt, 0.001f, 10.0f);

    // Prediction step
    predictState(dt);

    // Update step
    updatePosition(measuredPosition);
}

void predictState(float dt) {
    // Position prediction using velocity
    _state[0] += _state[2] * dt;  // pos_x += vel_x * dt
    _state[1] += _state[3] * dt;  // pos_y += vel_y * dt
    // Velocity remains constant (constant velocity model)

    // Covariance prediction (simplified)
    // Add process noise to position and velocity variances
    _covariance[0] += _processNoise * dt * dt;     // pos_x variance
    _covariance[5] += _processNoise * dt * dt;     // pos_y variance
    _covariance[10] += _processNoise;              // vel_x variance
    _covariance[15] += _processNoise;              // vel_y variance
}
```

## Fixed-Point Arithmetic

### 16.16 Fixed-Point Format

#### Representation
- **Format**: 16 integer bits + 16 fractional bits
- **Range**: -32768.0 to 32767.9999847412109375
- **Resolution**: 1/65536 ≈ 0.0000152587890625
- **Type**: int32_t (32-bit signed integer)

#### Conversion Functions
```cpp
// Float to fixed-point
AutoKalmanFixed::fixed_t floatToFixed(float value) {
    return (fixed_t)(value * 65536.0f);
}

// Fixed-point to float
float fixedToFloat(AutoKalmanFixed::fixed_t value) {
    return (float)value / 65536.0f;
}
```

### Arithmetic Operations

#### Multiplication
```cpp
AutoKalmanFixed::fixed_t fixedMul(fixed_t a, fixed_t b) {
    // Use 64-bit intermediate to prevent overflow
    int64_t result = (int64_t)a * (int64_t)b;
    // Shift right by 16 to maintain fixed-point format
    return (fixed_t)(result >> 16);
}
```

#### Division
```cpp
AutoKalmanFixed::fixed_t fixedDiv(fixed_t a, fixed_t b) {
    // Handle division by zero
    if (b == 0) {
        return (a >= 0) ? MAX_FIXED : MIN_FIXED;
    }

    // Shift numerator left by 16, then divide
    int64_t numerator = (int64_t)a << 16;
    return (fixed_t)(numerator / b);
}
```

#### Addition and Subtraction
```cpp
// Standard integer operations work directly
fixed_t fixedAdd(fixed_t a, fixed_t b) { return a + b; }
fixed_t fixedSub(fixed_t a, fixed_t b) { return a - b; }
```

### Precision Analysis

#### Error Sources
1. **Quantization error**: Limited fractional precision
2. **Rounding error**: Integer division truncation
3. **Overflow**: Exceeding 32-bit signed range

#### Precision Bounds
- **Addition/Subtraction**: Exact within representable range
- **Multiplication**: Error ≤ 0.5 ULP (Unit in Last Place)
- **Division**: Error ≤ 1 ULP

#### Optimal Ranges
| Operation | Input Range | Output Range | Precision |
|-----------|-------------|--------------|-----------|
| Addition | ±16384 | ±16384 | Exact |
| Multiplication | ±181.0 | ±32768 | 1/65536 |
| Division | ±32768 | ±32768 | 1/65536 |

## Numerical Stability

### Covariance Matrix Handling

#### Positive Definiteness Maintenance
```cpp
void ensurePositiveDefinite(float covariance[16]) {
    // Ensure diagonal elements are positive
    for (int i = 0; i < 4; i++) {
        if (covariance[i * 4 + i] < MIN_COVARIANCE) {
            covariance[i * 4 + i] = MIN_COVARIANCE;
        }
    }

    // Symmetrize matrix (ensure symmetry)
    for (int i = 0; i < 4; i++) {
        for (int j = i + 1; j < 4; j++) {
            float avg = (covariance[i * 4 + j] + covariance[j * 4 + i]) * 0.5f;
            covariance[i * 4 + j] = avg;
            covariance[j * 4 + i] = avg;
        }
    }
}
```

#### Joseph Form Covariance Update
```cpp
void updateCovarianceJoseph(float P[16], float K[8], float H[8], float R[4]) {
    // Joseph form: P = (I - K*H) * P * (I - K*H)^T + K*R*K^T
    // More numerically stable than simple form

    float I_minus_KH[16];
    // Compute I - K*H matrix
    matrixSubtractIdentity(I_minus_KH, K, H);

    float temp[16];
    // P_temp = (I - K*H) * P
    matrixMultiply4x4(I_minus_KH, P, temp);

    // P = P_temp * (I - K*H)^T
    float I_minus_KH_T[16];
    matrixTranspose4x4(I_minus_KH, I_minus_KH_T);
    matrixMultiply4x4(temp, I_minus_KH_T, P);

    // Add K*R*K^T term
    // (simplified for this implementation)
}
```

### Singularity Prevention

#### Innovation Covariance Regularization
```cpp
float regularizeInnovationCovariance(float S, float minValue = 1e-8f) {
    if (fabs(S) < minValue) {
        S = (S >= 0) ? minValue : -minValue;
    }
    return S;
}
```

#### Kalman Gain Limiting
```cpp
float limitKalmanGain(float K, float maxGain = 0.99f) {
    if (K > maxGain) return maxGain;
    if (K < -maxGain) return -maxGain;
    return K;
}
```

### Overflow Protection

#### Saturation Arithmetic
```cpp
fixed_t saturatedAdd(fixed_t a, fixed_t b) {
    fixed_t result = a + b;
    // Check for overflow (simplified)
    if ((a > 0 && b > 0 && result < 0) || (a < 0 && b < 0 && result > 0)) {
        return (result > 0) ? MAX_FIXED : MIN_FIXED;
    }
    return result;
}
```

#### Range Checking
```cpp
bool isValidFixedPoint(fixed_t value) {
    return (value >= MIN_FIXED && value <= MAX_FIXED);
}
```

## Optimization Techniques

### Memory Optimization

#### Compact State Storage
```cpp
// Use unions for type punning (when appropriate)
union StateStorage {
    struct {
        float processNoise;
        float measurementNoise;
        float estimatedError;
        float value;
        bool initialized;
    } floatingPoint;

    struct {
        fixed_t processNoise;
        fixed_t measurementNoise;
        fixed_t estimatedError;
        fixed_t value;
        bool initialized;
    } fixedPoint;
};
```

#### Bit-Packed Flags
```cpp
// Pack multiple boolean flags into a single byte
struct FilterFlags {
    bool initialized : 1;
    bool validMeasurement : 1;
    bool overflowDetected : 1;
    bool reserved : 5;  // For future use
};
```

### Computational Optimization

#### Loop Unrolling
```cpp
// Unroll small fixed-size matrix operations
void matrixMultiply4x4Unrolled(const float a[16], const float b[16], float result[16]) {
    // Manually unroll the 4x4x4 = 64 operations
    result[0] = a[0]*b[0] + a[1]*b[4] + a[2]*b[8] + a[3]*b[12];
    result[1] = a[0]*b[1] + a[1]*b[5] + a[2]*b[9] + a[3]*b[13];
    // ... continue for all 16 elements
}
```

#### Early Termination
```cpp
// Skip unnecessary calculations when filter is converged
bool isConverged(float covariance, float threshold = 1e-6f) {
    return fabs(covariance - previousCovariance) < threshold;
}
```

### Platform-Specific Optimizations

#### AVR Optimizations
```cpp
// Use PROGMEM for constant data on AVR
const float kalmanConstants[] PROGMEM = {
    1.0f, 0.0f, 1.0f, 0.0f,  // State transition matrix
    // ... other constants
};

// AVR-specific multiplication (if hardware multiplier available)
fixed_t avrMultiply(fixed_t a, fixed_t b) {
    // Use AVR-specific assembly for faster multiplication
    __asm__ volatile (
        "fmuls %A1, %A0" "\n\t"
        "movw %C0, r0" "\n\t"
        : "=r" (result)
        : "r" (a), "r" (b)
    );
    return result;
}
```

#### ARM Cortex-M Optimizations
```cpp
// Use CMSIS-DSP library if available
#include "arm_math.h"

void armMatrixMultiply(const arm_matrix_instance_f32* pSrcA,
                      const arm_matrix_instance_f32* pSrcB,
                      arm_matrix_instance_f32* pDst) {
    arm_mat_mult_f32(pSrcA, pSrcB, pDst);
}
```

This mechanism documentation provides detailed insight into the algorithmic implementations, numerical methods, and optimization techniques used in the AutoKalman library. Understanding these mechanisms enables users to effectively utilize the library and potentially extend it for specialized applications.

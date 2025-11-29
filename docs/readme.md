# AutoKalman Library Documentation

## Overview

The AutoKalman library provides a comprehensive implementation of Kalman filtering algorithms for Arduino platforms. It includes multiple filter variants optimized for different computational requirements and application domains, supporting one-dimensional, two-dimensional, and fixed-point arithmetic implementations.

## Documentation Structure

This documentation is organized into the following sections:

- **[Usage Guide](usage.md)**: Installation, setup, and basic usage instructions
- **[Examples](examples.md)**: Complete code examples for different use cases
- **[Explanation](explanation.md)**: Theoretical background and conceptual explanations
- **[System Architecture](system-architecture.md)**: Library design and implementation details
- **[Mechanism](mechanism.md)**: Algorithm implementations and technical details
- **[Troubleshooting](troubleshoot.md)**: Common issues and debugging guidance

## Library Versions

| Component | Version | Description |
|-----------|---------|-------------|
| AutoKalman | 2.0.0 | Core 1D floating-point Kalman filter |
| AutoKalman2D | 2.0.0 | 2D position/velocity Kalman filter |
| AutoKalmanFixed | 2.0.0 | Fixed-point arithmetic Kalman filter |
| AutoKalmanConfig | 2.0.0 | Configuration presets and utilities |

## Key Features

- **Multiple Implementation Variants**: Floating-point, fixed-point, and 2D filtering
- **Configuration Presets**: Pre-tuned parameters for common sensor types
- **Robust Error Handling**: Parameter validation and numerical stability
- **Memory Optimization**: Efficient implementations for constrained environments
- **Comprehensive Examples**: Production-ready code samples
- **Professional Documentation**: Complete API reference and guides

## Supported Platforms

- Arduino AVR (Uno, Mega, Nano, etc.)
- Arduino SAMD (Zero, MKR series)
- ESP8266 and ESP32
- Teensy (3.x, 4.x)
- Other Arduino-compatible platforms

## Quick Start

```cpp
#include <AutoKalman.h>

// Create a filter with preset configuration
AutoKalman filter(AutoKalmanConfig::TEMPERATURE);

// Use in your main loop
float measurement = analogRead(A0) * 3.3 / 1024.0;
float filteredValue = filter.filter(measurement);
```

For detailed installation and usage instructions, see the [Usage Guide](usage.md).

## License

This library is distributed under the MIT License. See the main library directory for complete license text.

## Support

For technical support, bug reports, or feature requests:
- Documentation: This documentation folder
- Examples: [examples/](examples.md) section
- Issues: GitHub repository issues
- Troubleshooting: [troubleshoot.md](troubleshoot.md)

---

*This documentation is maintained as part of the AutoKalman library project.*

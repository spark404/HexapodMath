# HexapodMath

HexapodMath is a C library providing mathematical functions and routines for robotics and kinematics, with a particular focus on hexapod (six-legged robot) motion and transformation tasks. The library builds on ARM's CMSIS-DSP for efficient vector and matrix operations and is suitable for use on embedded systems (e.g., ARM Cortex-M microcontrollers) as well as desktop development.

---

## Features

- **Forward and inverse kinematics** for legged robots
- Matrix and vector utilities (addition, subtraction, normalization, cross-product, etc.)
- 2D and 3D coordinate transformations
- Pose and transformation matrix manipulation
- Trigonometric functions (via CMSIS-DSP)
- Path interpolation and planning functions for leg movement

---

## Dependencies

- [CMSIS-DSP library](https://github.com/ARM-software/CMSIS-DSP)
- Standard C library

---

## Building

This library uses CMake for configuration. CMSIS-DSP is fetched automatically via CMake's FetchContent.

### Steps:

1. **Clone the repository**
2. **Create a build directory and configure:**
   ```sh
   mkdir build
   cd build
   cmake ..
   make
   ```

3. **Include HexapodMath in your project:**
    - Link against the `HexapodMath` static library
    - Add the `include/` directory to your compiler's include path

---

## Example Usage

### Forward Kinematics

```c
#include "hexapodmath/forward_kinematics.h"

float angles[3] = {/* joint angles in radians */};
float coordinates[3];
forward_kinematics(angles, coordinates);
// coordinates now holds the X, Y, Z position of the leg tip
```

### Inverse Kinematics

```c
#include "hexapodmath/inverse_kinematics.h"

float origin[3] = {0, 0, 0};
float tip[3] = {100, 20, -50};
float angles[3];
inverse_kinematics(origin, tip, angles);
// angles now holds the required joint angles
```

---

## API Overview

- **Vector routines:** see `include/hexapodmath/additional_functions.h`
- **2D/3D conversions:** `include/hexapodmath/conversion_2d.h`, `matrix_3d.h`
- **Kinematics:** `include/hexapodmath/forward_kinematics.h`, `inverse_kinematics.h`
- **Pose handling:** `include/hexapodmath/pose.h`
- **Path/interpolation:** `include/hexapodmath/hexapod.h`

---

## Integration Notes

- For STM32 and ARM targets, the library integrates with CMSIS-DSP and can be built as a static library.
- For desktop, standard CMake will build HexapodMath and fetch CMSIS-DSP automatically.
- The code is compiled with `-Wall -Wextra -pedantic -Werror` flags to ensure code quality.

---

## License

Apache 2.0

---

## Contributing

Pull requests and bug reports are welcome!

---

## Authors

Created by Hugo Trippaers, 2024–2025.

---

## Acknowledgements

- ARM CMSIS-DSP library
- Community robotics projects

---

For questions or support, please open an issue or contact the maintainer.
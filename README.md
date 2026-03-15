# elevated_control

Pure C++ control interface for Elevate Robotics 7-DOF manipulators. Communicates
with Synapticon drives over EtherCAT (via the bundled SOEM library) and exposes a
simple `ArmInterface` class.

## Dependencies

| Dependency | Purpose |
|---|---|
| **GCC 13+** (or any C++23 compiler) | `std::expected`, `std::jthread` |
| **spdlog** | Logging (`libspdlog-dev`) |
| **yaml-cpp** | Config file parsing (`libyaml-cpp-dev`) |
| **pthread / rt** | Real-time threading (provided by glibc) |

Install on Ubuntu 24.04:

```bash
sudo apt install build-essential cmake libspdlog-dev libyaml-cpp-dev
```

## Build

```bash
cd elevated_control
mkdir build && cd build
cmake ..
make -j$(nproc)
```

The build produces two artifacts:

- `libelevated_soem.a` -- static EtherCAT master library (SOEM/OSAL/OSHW)
- `libelevated_control.so` -- shared library with the `ArmInterface` class

## Usage

Link against `elevated_control` in your CMake project:

```cmake
find_package(spdlog REQUIRED)
add_subdirectory(path/to/elevated_control)
target_link_libraries(my_app PRIVATE elevated_control)
```

See [`examples/basic_usage.cpp`](examples/basic_usage.cpp) for a minimal
program that initializes the arm, reads joint positions, then stops.

EtherCAT requires root privileges for raw socket access:

```bash
sudo ./my_app
```

## Architecture

Two `std::jthread`s run inside `ArmInterface`:

1. **EtherCAT control loop** -- cyclic PDO exchange at the configured rate,
   per-joint state machine, gravity compensation, joint-limit enforcement.
2. **Dynamic simulation** -- currently a stub (produces zero feedforward torque).
   Will be wired to Drake for gravity compensation and collision checking.

Both threads share state through `DynamicSimState` using lock-free atomics and
are cleanly stopped via cooperative `std::stop_token` cancellation.

## Control Modes

| Mode | Enum | Description |
|---|---|---|
| Position | `kPosition` | Cyclic position (OpMode 8) |
| Velocity | `kVelocity` | Cyclic velocity (OpMode 9) |
| Torque | `kTorque` | Profile torque (OpMode 4) |
| Hand-guided | `kHandGuided` | Zero-torque with friction comp, dial-controlled wrist |
| Quick stop | `kQuickStop` | Motor brakes engaged |
| Spring adjust | `kSpringAdjust` | PD control of internal spring via potentiometer |

## Project Structure

```
elevated_control/
  CMakeLists.txt
  examples/
    basic_usage.cpp         Minimal usage example (builds as executable)
  include/elevated_control/
    arm_interface.hpp       Main class
    types.hpp               JointName, ControlLevel, ErrorCode, Error
    constants.hpp           Joint indices, EtherCAT constants
    somanet_pdo.hpp         Packed PDO structs
    unit_conversions.hpp    Ticks <-> radians, torque conversions
    velocity_filter.hpp     Low-pass filter
    joint_admittance.hpp    Single-joint admittance controller
    joint_limits.hpp        Soft joint-limit enforcement
    config_parsing.hpp      YAML config parsing
    state_machine.hpp       EtherCAT state machine helpers
    dynamic_sim.hpp         Dynamic simulation stub
  src/
    arm_interface.cpp       Full implementation
    joint_admittance.cpp
    velocity_filter.cpp
    config_parsing.cpp
    dynamic_sim.cpp
    osal/                   OS abstraction (from SOEM)
    oshw/                   Hardware abstraction (from SOEM)
    soem/                   Simple Open EtherCAT Master
```

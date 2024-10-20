# Multicopter Attitude Control 
## Overview

- `MulticopterAttitudeControl.hpp` defines `MulticopterAttitudeControl` class & it's implementation is in `MulticopterAttitudeControl.cpp`. 

- This implements the multicopter attitude controller. It takes attitude setpoints (`vehicle_attitude_setpoint`) as inputs and outputs a rate setpoint.
  
## Class Definition

### Initialization

```cpp
bool MulticopterAttitudeControl::init();
```

- **init()**: Registers the callback for vehicle attitude updates. Returns true if successful, otherwise false.

### Parameter Management

```cpp
void MulticopterAttitudeControl::parameters_updated();
```

- **parameters_updated()**: Updates internal parameters and precomputes frequently used values like proportional gains and angular rate limits based on the configured parameters.

### Throttle Control

```cpp
float MulticopterAttitudeControl::throttle_curve(float throttle_stick_input);
```

- **throttle_curve()**: Calculates the thrust based on the throttle stick input. Depending on the throttle curve parameter, it can either apply no rescaling or map the input to hover thrust. The resulting thrust is limited to the maximum allowed value.

### Attitude Setpoint Generation

```cpp
void MulticopterAttitudeControl::generate_attitude_setpoint(const Quatf &q, float dt, bool reset_yaw_sp);
```

- **generate_attitude_setpoint()**: Generates the attitude setpoint based on the current vehicle orientation, user manual inputs, and time delta. This function:
  - Calculates the desired yaw setpoint.
  - Updates roll and pitch inputs, ensuring they are within the maximum tilt limits.
  - Aligns the desired tilt with the yaw setpoint.
  - Publishes the calculated setpoint to the appropriate topic.

### Main Run Loop

```cpp
void MulticopterAttitudeControl::Run();
```

- **Run()**: The main execution loop of the controller that:
  - Checks if the controller should exit and unregisters callbacks if needed.
  - Begins performance monitoring.
  - Handles parameter updates and recalibrates internal parameters.
  - Processes updates from the vehicle's attitude, control mode, and status subscriptions.
  - Generates the attitude setpoint if manual control is enabled and other conditions are met.
  - Checks for new attitude setpoints and updates control inputs accordingly.
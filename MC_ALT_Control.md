> `MulticopterAttitudeControl.hpp`

- Defines `MulticopterAttitudeControl` class & the function impl is in `MulticopterAttitudeControl.cpp`
> This implements the multicopter attitude controller. It takes attitude
setpoints (`vehicle_attitude_setpoint`) as inputs and outputs a rate setpoint.
- Important function
```cpp
	/**
	 * Generate & publish an attitude setpoint from stick inputs
	 */
	void generate_attitude_setpoint(const matrix::Quatf &q, float dt, bool reset_yaw_sp);
```
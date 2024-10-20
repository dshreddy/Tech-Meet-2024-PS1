> `MulticopterPositionControl.hpp`
- This files defines the `MulticopterPositionControl` class which inherits from `ModuleBase`, `ModuleParams`, and `px4::ScheduledWorkItem`. 

- The class only defines the function sigmatures with actual implementation in `MulticopterPositionControl.cpp`
- The class has the following objects created inside it

```cpp
    TakeoffHandling _takeoff; /**< state machine and ramp to bring the vehicle off the ground without jumps */
	GotoControl _goto_control; ///< class for handling smooth goto position setpoints
	PositionControl _control; ///< class for core PID position control
```

- Description is given for following functions

```cpp
	/**
	 * Update our local parameter cache.
	 * Parameter update can be forced when argument is true.
	 * @param force forces parameter update.
	 */
	void parameters_update(bool force);

	/**
	 * Check for validity of positon/velocity states.
	 */
	PositionControlStates set_vehicle_states(const vehicle_local_position_s &local_pos, const float dt_s);

	/**
	 * Generate setpoint to bridge no executable setpoint being available.
	 * Used to handle transitions where no proper setpoint was generated yet and when the received setpoint is invalid.
	 * This should only happen briefly when transitioning and never during mode operation or by design.
	 */
	trajectory_setpoint_s generateFailsafeSetpoint(const hrt_abstime &now, const PositionControlStates &states, bool warn);

	/**
	 * @brief adjust existing (or older) setpoint with any EKF reset deltas and update the local counters
	 *
	 * @param[in] vehicle_local_position struct containing EKF reset deltas and counters
	 * @param[out] setpoint trajectory setpoint struct to be adjusted
	 */
	void adjustSetpointForEKFResets(const vehicle_local_position_s &vehicle_local_position,
					trajectory_setpoint_s &setpoint);
```

> `Takeoff.hpp`

- This files defines the `TakeoffHandling` class.
- The class only defines the function sigmatures with actual implementation in `Takeoff.cpp`
- The following functions have description 

```cpp
	// initialize parameters
	void setSpoolupTime(const float seconds) { _spoolup_time_hysteresis.set_hysteresis_time_from(false, seconds * 1_s); }
	void setTakeoffRampTime(const float seconds) { _takeoff_ramp_time = seconds; }

	/**
	 * Calculate a vertical velocity to initialize the takeoff ramp
	 * that when passed to the velocity controller results in a zero throttle setpoint.
	 * @param hover_thrust normalized thrsut value with which the vehicle hovers
	 * @param velocity_p_gain proportional gain of the velocity controller to calculate the thrust
	 */
	void generateInitialRampValue(const float velocity_p_gain);

	/**
	 * Update the state for the takeoff.
	 * Has to be called also when not flying altitude controlled to skip the takeoff and not do it in flight when switching mode.
	 */
	void updateTakeoffState(const bool armed, const bool landed, const bool want_takeoff,
				const float takeoff_desired_vz, const bool skip_takeoff, const hrt_abstime &now_us);

	/**
	 * Update and return the velocity constraint ramp value during takeoff.
	 * By ramping up _takeoff_ramp_vz during the takeoff and using it to constain the maximum climb rate a smooth takeoff behavior is achieved.
	 * Returns zero on the ground and takeoff_desired_vz in flight.
	 * @param dt time in seconds since the last call/loop iteration
	 * @param takeoff_desired_vz end value for the velocity ramp
	 * @return true if setpoint has updated correctly
	 */
	float updateRamp(const float dt, const float takeoff_desired_vz);
```

> `PositionControl.hpp`

- This class defines `PositionControl` class & it's implementation is in `PositionControl.cpp` which imports (includes) `ControlMath.hpp`

> `GotoControl.hpp`

- Defines `GotoControl` class & it's implementation in `GotoControl.cpp`

```cpp
/**
 * @file GotoControl.hpp
 *
 * A class which smooths position and heading references from "go-to" setpoints
 * for planar multicopters.
 *
 * Be sure to set constraints with setGotoConstraints() before calling the update() method for the first time
 */
```
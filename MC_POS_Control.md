# Multicopter Position Control 
- `MulticopterPositionControl.hpp` defines the `MulticopterPositionControl` class which inherits from `ModuleBase`, `ModuleParams`, and `px4::ScheduledWorkItem`. 
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

# TakeoffHandling
- `Takeoff.hpp` defines the `TakeoffHandling` class.
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

# PositionControl
- `PositionControl.hpp` defines `PositionControl` class & it's implementation is in `PositionControl.cpp`

1. **Constants and Structures**: 
   - It defines an empty trajectory setpoint with all values set to `NAN`.

3. **Gain and Limit Settings**:
   - Methods for setting velocity gains (`P`, `I`, `D`), velocity limits (horizontal and vertical), thrust limits, and horizontal thrust margins are provided. 

4. **Hover Thrust Adjustment**:
   - The `updateHoverThrust` method calculates the necessary adjustment to the acceleration setpoint when the hover thrust is changed.

5. **State Management**:
   - The `setState` method updates the position, velocity, and yaw states of the drone, while the `setInputSetpoint` method takes in a trajectory setpoint, including position, velocity, acceleration, yaw, and yaw speed.

6. **Control Update Logic**:
   - The `update` method checks for valid inputs and processes position and velocity control through `_positionControl` and `_velocityControl` methods, respectively. It ensures that the output acceleration and thrust setpoints are valid.

7. **Position Control**:
   - In `_positionControl`, a proportional controller calculates the desired velocity based on position errors. It also applies constraints to ensure the velocity remains within limits.

8. **Velocity Control**:
   - The `_velocityControl` method handles the PID control for velocity, calculates the acceleration setpoint based on velocity errors, and implements anti-windup mechanisms for thrust and velocity.

9. **Acceleration Control**:
   - The `_accelerationControl` method computes thrust based on specific forces, converts these to thrust values, and limits tilt angles to maintain stable flight.

10. **Input Validation**:
    - The `_inputValid` method checks the validity of input states and ensures that setpoints are finite and correspond appropriately across axes.

11. **Setpoint Retrieval**:
    - The methods `getLocalPositionSetpoint` and `getAttitudeSetpoint` retrieve the current local position and attitude setpoints, respectively, populating structures with the necessary data for flight control.

# GotoControl
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
1. **Check for Setpoint**:
   - The `checkForSetpoint` method verifies whether a valid setpoint is available and whether the system should process it based on the current timestamp and the enabled state.

2. **Update Method**:
   - The `update` method:
     - Initializes position and heading smoothers if not already done.
     - Retrieves the desired position setpoint.
     - Validates the setpoint and current position for finiteness.
     - Generates smoothed position setpoints using the `PositionSmoothing` class.
     - Updates the trajectory setpoint, including position, velocity, and acceleration.
     - Optionally controls the heading, applying similar smoothing techniques and updates the trajectory accordingly.
     - Publishes the updated trajectory and vehicle constraints.

3. **Reset Functions**:
   - `resetPositionSmoother`: Resets the position smoother using the current position, ensuring that it is valid.
   - `resetHeadingSmoother`: Resets the heading smoother with the current heading.

4. **Setting Constraints**:
   - `setPositionSmootherLimits`: Sets constraints on the maximum horizontal and vertical speeds and accelerations based on the current setpoint, ensuring that the vehicle does not exceed these limits.
   - `setHeadingSmootherLimits`: Sets constraints on the maximum heading rate and acceleration.

5. **Mathematical Operations**:
   - The file frequently uses mathematical constraints to ensure values remain within defined limits, using the `math::constrain` function for safety.
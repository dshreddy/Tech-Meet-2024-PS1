# POSITION & ALTITUDE CONTROL OF A QUADCOPTER WITH SINGLE MOTOR FAILURE


## Introduction & Motivation

Quadrotors are a popular choice for small UAVs due to multiple reasons. First, they have a fairly straightforward design. Second, their symmetric design simplifies the control algorithms needed to operate these UAVs and provides inherent stability. Their capabilities like vertical takeoff and landing (VTOL), stable hover allow space efficient deployment and maneuverability. Quadrotor based UAVs are widely used today across multiple applications including aerial surveillance, surveying and mapping, inspection, aerial photography, search and rescue, agriculture, delivery logistics, etc.

However, the simple and minimal design of quadrotors also makes them vulnerable to the **failure of a single motor**, which can lead to loss of control and potential crashes. **Motor failure could occur due to mechanical causes such as bearing, shaft or propeller issues, or electrical causes such as overheating, short circuits, winding damage, etc. Operating in extreme environmental conditions such as heat, cold or rain can also lead to failure.**

**As UAV platforms increase in size and weight and as the deployment of UAVs in urban, populated areas keeps growing, any potential UAV crash can cause harm to human life or property**. Developing a robust control algorithm to handle motor failures is therefore crucial for enhancing the safety and reliability of quadrotor systems.

## Problem Statement

Given the potential damage that a UAV crash due to a single motor failure can cause, the problem is to design and implement a control algorithm that enables a standard quadrotor to recover with minimum loss of stability in the event of a single motor failure. Solutions could iteratively build on smaller sub-problems of increasing complexity

- Implicit detection of single motor failure
- Control adjustments to remaining three motors for an immediate controlled landing (for example spinning and landing)
- Control adjustments to remaining three motors to achieve stable hover (altitude and position hold)
- Control adjustments to remaining three motors to achieve basic navigation and altitude control to allow safe return home / landing.

## System Setup
The solution should be designed in the context of the following system configuration:
- Standard quadrotor configuration
- Onboard sensors may include Accelerometer, Gyroscope, Magnetometer, Barometer and GPS. No additional sensors should be assumed
- No reversible motor/prop or tilt/servo mechanisms
- All solutions must be developed using the open source PX4 flight controller stack
- Teams should use the Gazebo simulator using the IRIS model (default gazebo quadrotor model)
- **Teams able to achieve a successful simulation can attempt to demonstrate the solution in an actual flight for bonus points – links to the PX4 compatible development kits are given below.**

## Terminology 
- **Quadcopter** : A quadcopter, also called quadrocopter, or quadrotor is a type of helicopter or multicopter that has four rotors.  
- **Rotor** : On a helicopter, the main rotor or rotor system is the combination of several rotary wings (rotor blades) with a control system, that generates the aerodynamic lift force that supports the weight of the helicopter, and the thrust that counteracts aerodynamic drag in forward flight.
- **Quadcopter vs Drone** : Once airborne, quadcopters have the ability to hover in place whereas fixed wing aerial drones must be constantly moving. Quadcopters are also capable of much more precise aerial maneuvers while fixed wing aerial drones can only make much less precise flyby runs.
- **UAV** : Unmanned Aerial Vehicle
- **VTOL**v : Vertical takeoff and landing 
- **Position** : 
- **Altitude** : 

## My Task
- [multi copter att control](https://docs.px4.io/main/en/modules/modules_controller.html#mc-att-control) : This implements the multicopter attitude controller. It takes attitude setpoints (vehicle_attitude_setpoint) as inputs and outputs a rate setpoint. The controller has a P loop for angular error.

```
Usage - 

mc_att_control <command> [arguments...]

 Commands:
 
   start
     [vtol]      VTOL mode

   stop

   status        print status info
```

- [multi copter pos control](https://docs.px4.io/main/en/modules/modules_controller.html#mc-pos-control) : The controller has two loops: a P loop for position error and a PID loop for velocity error. Output of the velocity controller is thrust vector that is split to thrust direction (i.e. rotation matrix for multicopter orientation) and thrust scalar (i.e. multicopter thrust itself).

The controller doesn't use Euler angles for its work, they are generated only for more human-friendly control and logging.

```
Usage - 

mc_pos_control <command> [arguments...]

 Commands:

   start
     [vtol]      VTOL mode

   stop

   status        print status info
```
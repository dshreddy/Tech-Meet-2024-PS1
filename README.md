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

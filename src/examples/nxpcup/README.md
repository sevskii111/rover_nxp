# NXP Cup example code for RDDRONE-FMUK66
This is an application to run a rover on a white track which is borderd by black lines.
This is a submodule which should be integrated in the PX4 firmware

This example can be used with a PX4 firmware with version v1.11.0-beta1 or newer.
The firmware is available here: https://github.com/PX4/Firmware.git

### Including this module into the PX4 firmware
Clone the PX4 repository: https://github.com/PX4/Firmware.git
Then go to the folder `~/Firmware/src/examples/` and copy the folder from this download page https://nxp.gitbook.io/nxp-cup/downloads-and-links#example-code into the examples folder.
Go back to the `Firmware` folder. Open the file `boards/nxp/fmuk66-v3/default.cmake` and add `nxpcup`in the section `EXAMPLES`. If the application should start automatically open the file `ROMFS/px4fmu_common/init.d/rcS` and add the following code at the end of the autostart condition. This may be somewhere in line 524 (depends on the Firmware version).
```
# Rover controlling with an track algorithm
nxpcup start
```

This example is intended to be used without an RC control. Therefore a few changes have to be made. First arming with the Safety Swich should be enabled. This enables stating all driving functionallities by pressing the Safety Switch.
> **ATTENTION!**
> Changing this funtioncallity is on own risk. Do not use this in combination with a drone. This starts the motors after pressing Safety Switch and in case of using a drone propellers can start and can cut you. So please just use this in for Rovers.

In the file src/modules/commander/Commander.cpp a few lines of code have to be added. Search for the following code:
```
/* update safety topic */
if (_safety_sub.updated()) {
	const bool previous_safety_off = _safety.safety_off;

	if (_safety_sub.copy(&_safety)) {
		// disarm if safety is now on and still armed
		if (armed.armed && _safety.safety_switch_available && !_safety.safety_off) {

			bool safety_disarm_allowed = (status.hil_state == vehicle_status_s::HIL_STATE_OFF);

			// if land detector is available then prevent disarming via safety button if not landed
			if (hrt_elapsed_time(&_land_detector.timestamp) < 1_s) {
...
```
Now add the following code after the line `if (_safety_sub.copy(&_safety)) {`
```
// arming only with safety switch if vehicle type is rover and RC control mode is "Joystick/no RC checks"
if (_safety.safety_switch_available && _safety.safety_off && (status.rc_input_mode == vehicle_status_s::RC_IN_MODE_OFF)) {
	bool safety_arm_allowed = (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROVER);
	if (safety_arm_allowed) {
		if (TRANSITION_CHANGED == arm_disarm(true, true, &mavlink_log_pub, "Safety button")) {
			_status_changed = true;
		}
	}
}
```
The code should follow with
```
// disarm if safety is now on and still armed
if (armed.armed && _safety.safety_switch_available && !_safety.safety_off) {
```

Last but not least run a `make nxp_fmuk66-v3_default`. And run `make nxp_fmuk66-v3_default upload` to flash your FMUK66.

A last change has to be made in QGroundControl. Open this application and connect the rover to it. Now go to settings and then to parameters. Search for "COM_RC_IN_MODE" and set value to "Joystick/No RC Checks" and disable the parameter "NAV_RCL_ACT".

### Handling of the module
This module is designed as a thread application. So this application allows to use all other features of the Firmware. The application can be started via a MAVLink console (or the screen function from Linux) with the command `nxpcup start` and be stopped with `nxpcup stop`. The actual status of the application can be called with `nxpcup status`. If the autostart is considered the `nxpcup start` command is not needed to start the application after startup.

### Functionality of this module
The main function of the nxpcup creates a instanze of the Pixy 2 and initializes it. The uORB topics for safety and RC input are subscribed and an instance to publish actuator controls is made.
If the initialization of the pixy way successful, the version is printed. And a endless-while loop starts. Within this loop the information about the detected lines are called from the Pixy2. Here all line are requested. Only the main vector can be requested, but since you want to drive between two lines, all lines are requried. For starting the function the safety switch must be activated. Then the rover is set to attitude control, other control modes are disabled now and the function `roverControl raceTrack(Pixy2 &pixy)` (the algorithm) is called. This function returns a struct with the values for steering and speed of the motors. In the default version steering is statically set to 30 degree and speed is set to 10% forward. If the Safety Switch is deactivated the values of speed and steering are set to zero and manual control ist enabled again. The struct with the motor values are passed to the function `void roverSteerSpeed(roverControl control, vehicle_attitude_setpoint_s &_att_sp)`. This function converts the motorControl values to attitude setpoints. The attitude setpoints are published within the loop in the main function. The loop always proofs if this thread should be exited.

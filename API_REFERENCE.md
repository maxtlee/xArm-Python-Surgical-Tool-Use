# xArm Python SDK — API Reference

## Import

```python
from xarm.wrapper import XArmAPI
arm = XArmAPI('COM5')
arm = XArmAPI('192.168.1.113')
arm = XArmAPI('192.168.1.113', do_not_open=False)
arm = XArmAPI('192.168.1.113', is_radian=False)
```

## Connect/Disconnect

```python
arm.connect(...)
arm.disconnect()
```

## Move

```python
arm.reset(...)
arm.set_position(...)
arm.set_servo_angle(...)
arm.set_servo_angle_j(...)
arm.set_servo_cartesian(...)
arm.move_gohome(...)
arm.move_circle(...)
arm.emergency_stop()
arm.set_position_aa(...)
arm.set_servo_cartesian_aa(...)
arm.vc_set_joint_velocity(...)
arm.vc_set_cartesian_velocity(...)
```

## Set

```python
arm.set_servo_attach(...)
arm.set_servo_detach(...)
arm.set_state(...)
arm.set_mode(...)
arm.motion_enable(...)
arm.set_pause_time(...)
```

## Get

```python
arm.get_version()
arm.get_state()
arm.get_is_moving()
arm.get_cmdnum()
arm.get_err_warn_code()
arm.get_position(...)
arm.get_servo_angle(...)
arm.get_position_aa(...)
arm.get_pose_offset(...)
```

## Setting

```python
arm.set_tcp_offset(...)
arm.set_tcp_jerk(...)
arm.set_tcp_maxacc(...)
arm.set_joint_jerk(...)
arm.set_joint_maxacc(...)
arm.set_tcp_load(...)
arm.set_collision_sensitivity(...)
arm.set_teach_sensitivity(...)
arm.set_gravity_direction(...)
arm.config_tgpio_reset_when_stop(...)
arm.config_cgpio_reset_when_stop(...)
arm.set_report_tau_or_i(...)
arm.set_self_collision_detection(...)
arm.set_collision_tool_model(...)
arm.clean_conf()
arm.save_conf()
```

## Gripper / Gripper G2

```python
arm.set_gripper_enable(...)
arm.set_gripper_mode(...)
arm.set_gripper_speed(...)
arm.set_gripper_position(...)
arm.get_gripper_position()
arm.get_gripper_err_code()
arm.clean_gripper_error()

# only Gripper G2
arm.get_gripper_g2_position()
arm.set_gripper_g2_position(...)
```

## BIO Gripper

```python
arm.set_bio_gripper_enable(...)
arm.set_bio_gripper_speed(...)
arm.open_bio_grippe(...)
arm.close_bio_gripper(...)
arm.get_bio_gripper_status()
arm.get_bio_gripper_error()
arm.clean_bio_gripper_error()

# only BIO Gripper G2
arm.get_bio_gripper_g2_position()
arm.set_bio_gripper_g2_position(...)
```

## RobotIQ Gripper

```python
arm.robotiq_reset()
arm.robotiq_set_activate(...)
arm.robotiq_set_position(...)
arm.robotiq_open(...)
arm.robotiq_close(...)
arm.robotiq_get_status(...)
```

## Modbus / RS485

```python
arm.set_rs485_timeout(...)
arm.get_rs485_timeout(...)
arm.set_rs485_baudrate(...)
arm.get_rs485_baudrate(...)
arm.set_rs485_data(...)
```

## GPIO

```python
# Tool GPIO
arm.get_tgpio_digital(...)
arm.set_tgpio_digital(...)
arm.get_tgpio_analog(...)
arm.set_tgpio_digital_with_xyz(...)
# Controller GPIO
arm.get_cgpio_digital(...)
arm.get_cgpio_analog(...)
arm.set_cgpio_digital(...)
arm.set_cgpio_analog(...)
arm.set_cgpio_digital_input_function(...)
arm.set_cgpio_digital_output_function(...)
arm.get_cgpio_state()
arm.set_cgpio_digital_with_xyz(...)
arm.set_cgpio_analog_with_xyz(...)
```

## Linear Motor

```python
arm.get_linear_motor_pos()
arm.get_linear_motor_status()
arm.get_linear_motor_error()
arm.get_linear_motor_is_enabled()
arm.get_linear_motor_on_zero()
arm.get_linear_motor_sci()
arm.get_linear_motor_sco()

arm.clean_linear_motor_error(...)
arm.set_linear_motor_enable(...)
arm.set_linear_motor_speed(...)
arm.set_linear_motor_back_origin(...)
arm.set_linear_motor_pos(...)
arm.set_linear_motor_stop(...)
```

## FT Sensor

```python
arm.set_ft_sensor_enable(...)
arm.set_ft_sensor_mode(...)
arm.get_ft_sensor_mode(...)
arm.set_ft_sensor_zero(...)
arm.iden_ft_sensor_load_offset(...)
arm.set_ft_sensor_load_offset(...)
arm.set_ft_sensor_admittance_parameters(...)
arm.set_ft_sensor_force_parameters(...)
arm.get_ft_sensor_data(...)
arm.get_ft_senfor_config(...)
arm.get_ft_sensor_error(...)
```

## Other

```python
arm.set_pause_time(...)
arm.system_control(...)
arm.clean_error()
arm.clean_warn()
arm.set_counter_reset()
arm.set_counter_increase(...)
```

## Register/Release Callbacks

```python
arm.register_report_callback(...)
arm.register_report_location_callback(...)
arm.register_connect_changed_callback(callback)
arm.register_state_changed_callback(callback)
arm.register_mode_changed_callback(callback)
arm.register_mtable_mtbrake_changed_callback(callback)
arm.register_error_warn_changed_callback(callback)
arm.register_cmdnum_changed_callback(callback)
arm.register_temperature_changed_callback(callback)
arm.register_count_changed_callback(callback)
arm.release_report_callback(callback)
arm.release_report_location_callback(callback)
arm.release_connect_changed_callback(callback)
arm.release_state_changed_callback(callback)
arm.release_mode_changed_callback(callback)
arm.release_mtable_mtbrake_changed_callback(callback)
arm.release_error_warn_changed_callback(callback)
arm.release_cmdnum_changed_callback(callback)
arm.release_temperature_changed_callback(callback)
arm.release_count_changed_callback(callback)
```

## Properties

```python
arm.connected
arm.default_is_radian
arm.version
arm.position
arm.last_used_position
arm.tcp_speed_limit
arm.tcp_acc_limit
arm.last_used_tcp_speed
arm.last_used_tcp_acc
arm.angles
arm.joint_speed_limit
arm.joint_acc_limit
arm.last_used_angles
arm.last_used_joint_speed
arm.last_used_joint_acc
arm.tcp_offset
arm.state
arm.mode
arm.joints_torque
arm.tcp_load
arm.collision_sensitivity
arm.teach_sensitivity
arm.motor_brake_states
arm.motor_enable_states
arm.has_err_warn
arm.has_error
arm.has_warn
arm.error_code
arm.warn_code
arm.cmd_num
arm.device_type
arm.axis
arm.gravity_direction
arm.gpio_reset_config
arm.count
arm.temperatures
arm.voltages
arm.currents
arm.cgpio_states
```

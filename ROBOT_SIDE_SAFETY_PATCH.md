# Raspberry Pi forward-obstacle safety patch

Apply this to the host copy of `unity_master_team2.py` used by
`start_robot_team2.sh`. The startup script copies that host file into the
`xiao_ros_brain` container, so changing only the temporary container copy will
be lost on restart.

Back up the file first. In `vel_callback`, immediately after the existing
`lin_x` clamp, add:

```python
    # Fail-safe local stop. Keep rotation and reverse available so the robot can
    # escape, but never accept a forward component inside the safety distance.
    if lin_x > 0.0 and 0.0 < filtered_cm < SAFETY_STOP_CM:
        rospy.logwarn_throttle(
            1.0,
            "LOCAL SAFETY STOP: ultrasonic %.1f cm < %d cm",
            filtered_cm,
            SAFETY_STOP_CM,
        )
        lin_x = 0.0
```

The relevant section must then be equivalent to:

```python
def vel_callback(data):
    global filtered_cm, last_cmd_vel_time, prev_ang_z
    last_cmd_vel_time = time.time()

    TURN_K = 0.25
    MAX_LINEAR = 0.25
    lin_x = max(min(data.linear.x, MAX_LINEAR), -MAX_LINEAR)

    if lin_x > 0.0 and 0.0 < filtered_cm < SAFETY_STOP_CM:
        rospy.logwarn_throttle(
            1.0,
            "LOCAL SAFETY STOP: ultrasonic %.1f cm < %d cm",
            filtered_cm,
            SAFETY_STOP_CM,
        )
        lin_x = 0.0

    EMA_STEER = 0.40
    ang_z = EMA_STEER * data.angular.z + (1.0 - EMA_STEER) * prev_ang_z
    prev_ang_z = ang_z

    v_left = lin_x + (ang_z * TURN_K)
    v_right = lin_x - (ang_z * TURN_K)
    pwm_left = clamp_pwm(v_left * PWM_CONVERSION_FACTOR)
    pwm_right = clamp_pwm(v_right * PWM_CONVERSION_FACTOR)
    set_motors_pwm(pwm_left, pwm_right)
```

After editing, restart `start_robot_team2.sh`, keep the tracks lifted, and send
a small forward test while placing a large obstacle in front of the ultrasonic
sensor. Confirm `LOCAL SAFETY STOP` in the robot log and zero forward motion.

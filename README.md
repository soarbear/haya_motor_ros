# haya_motor_ros

Using 9axis-imu haya_imu in this project to evaluate whether the A/B phase pulses of the encoder, which is the basis for calculating the rotation speed of motor output shaft, are correctly measured or not, and to evaluate the accuracy and accuracy of DC motor PID control.

# Required ROS packages or LIBs

rosserial
haya_motor_ros
haya_imu_ros
ros_lib by rosserail_arduino with Arduino IDE
Adafruit-Motor-Shield-library-leonardo
arduino_node(ino) on Leonardo with Arduino IDE 

# ROS node interaction status

<img src="https://github.com/soarbear/haya_motor_ros/blob/master/haya_rqt_graph_haya_motor.png" alt="haya_motor_ros ROS node interaction status" title="haya_motor_ros ROS node interaction status" />

# Launch

roslaunch haya_imu_ros haya_imu.launch

roslaunch haya_motor_ros haya_motor.launch

# Evaluation environment

DC motor PID control evaluation environment using haya_imu.

<img src="https://github.com/soarbear/haya_motor_ros/blob/master/haya_motor_pid_control_evaluation_environment.jpg" alt="motor_pid_control_evaluation_environment" title="motor_pid_control_evaluation_environment" />

# PID control curve

Target value of PID control, rotation speed by encoder, rotation speed curve measured by haya_imu.

<img src="https://github.com/soarbear/haya_motor_ros/blob/master/haya_pid_anguler_velocity_by_haya_imu_encoder_counter.png" alt="pid_anguler_velocity_by_haya_imu_encoder_counter" title="pid_anguler_velocity_by_haya_imu_encoder_counter" />

# PWM pulse

PWM pulse by oscilloscope.

<img src="https://github.com/soarbear/haya_motor_ros/blob/master/haya_pid_pwm_pulse_by_oscilloscope.jpg" alt="pid_pwm_pulse_by_oscilloscope" title="pid_pwm_pulse_by_oscilloscope" />

# For more in Japanese

<a href="https://memo.soarcloud.com/encoder-motor-pid-control-with-haya_imu/">https://memo.soarcloud.com/encoder-motor-pid-control-with-haya_imu/</a>

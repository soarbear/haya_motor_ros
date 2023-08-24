/*
 * arduino_haya_motor.ino
 *
 * License: BSD-3-Clause
 *
 * Copyright (c) 2012-2023 Shoun Corporation <research.robosho@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of RT Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#define USE_USBCON // For Leonardo
#include <ros.h>
#include <std_msgs/Float32.h>
#include <haya_motor_ros/ArduinoMsgs.h>
#include <haya_motor_ros/PidMsgs.h>
#include <AFMotor.h>

ros::NodeHandle nh; // ROS node handler
haya_motor_ros::ArduinoMsgs arduino_msg;
ros::Publisher pub_arduino("arduino_msg", &arduino_msg);
void sub_pid_cb(const haya_motor_ros::PidMsgs& pid);
ros::Subscriber<haya_motor_ros::PidMsgs> sub_pid("pid_params", &sub_pid_cb);

#define ENCODER_A 2 // Leonardo D2
#define ENCODER_B 3 // Leonardo D3

// Run the PID loop at 100 times per second
#define PID_RATE_HZ 10 // 10Hz

// Convert the rate into an interval
#define PID_INTERVAL_MS (1000 / PID_RATE_HZ) // 10ms
#define PID_INTERVAL_S (float)(1.0 / PID_RATE_HZ) // 0.1s

#define PPR 12 // Encoder pulse / round
#define COUNTER_FACTOR 4 // 1*encoder-pulse = 4*counter
#define REDUCTION_RATIO 100 // Gear reduction ratio
#define CPR (PPR * COUNTER_FACTOR * REDUCTION_RATIO) // Counters of encoder / round

// Track the next time we make a PID control
volatile uint64_t next_pid = PID_INTERVAL_MS;

// Encoder counter updated while interruts from encoder A or B phase
volatile int64_t current_counter = 0;

// Current velocity
volatile float current_velocity = 0.0;

// Target velocity
volatile float target_velocity=0.0, Kp=0.0, Ki=0.0, Kd=0.0; 

AF_DCMotor motor(4);

/*
 * Callback for subscribing pwm
 */
void sub_pid_cb(const haya_motor_ros::PidMsgs& pid){
  Kp = pid.Kp;
  Ki = pid.Ki;
  Kd = pid.Kd;  
  //target_velocity = pid.target_velocity * 180.0 / PI;
  target_velocity = pid.target_velocity; // DEG
}

/*
 * Control motor with pwm
 */
void control_motor(int32_t pwm_value) {  
  if (pwm_value > 0) {
    // Direction Control
    motor.run(FORWARD);
    // PWM Speed Control
    motor.setSpeed(pwm_value);
  }
  else if (pwm_value < 0) {
    // Direction Control
    motor.run(BACKWARD);
    // PWM Speed Control
    motor.setSpeed(-pwm_value);
  }
  else { // ==0
    // STOP
    motor.run(RELEASE);
  }
}

/*
 * Read A-phase of encoder
 */
void a_handler(void) {
  if (digitalRead(ENCODER_A) != digitalRead(ENCODER_B)) {
    current_counter++;
  }
  else {
    current_counter--;
  }
}

/*
 * Read B-phase of encoder
 */
void b_handler(void) {
  if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) {
    current_counter++;
  }
  else {
    current_counter--;
  }
}

/*
 * Compuete pwm with PID algorithm
 */
int32_t pid(void) {
// Positional PID
  static float previous_error = 0.0, integral = 0.0;
  static int32_t pwm = 0;
  float error, differential;

  error = target_velocity - current_velocity;
  integral += error * PID_INTERVAL_S;
  differential = (error - previous_error) / PID_INTERVAL_S;
  previous_error = error;
  pwm = (int32_t)(Kp * error + Ki * integral + Kd * differential);
/*
// Incremental PID
  static float previous_error = 0.0, previous_error2 = 0.0;
  static int32_t pwm = 0;
  float error, proportion, integral, differential;

  error = target_velocity - current_velocity;
  proportion = Kp * (error - previous_error);
  integral = Ki * error * PID_INTERVAL_S;
  differential = Kd * (error - 2 * previous_error + previous_error2) / PID_INTERVAL_S;
  pwm += (int32_t)(proportion + integral + differential);
  previous_error2 = previous_error;
  previous_error = error;
*/
  if (pwm > 255) {
    pwm = 255;
  }
  else if (pwm < -255) {
    pwm = -255;
  }
  return pwm;
}

/*
 * setup for arduino
 */
void setup() {

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), a_handler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), b_handler, CHANGE);

  nh.initNode();
  nh.advertise(pub_arduino);
  nh.subscribe(sub_pid);
  nh.loginfo("[arduino-node] arduino_control_motor started up");
  //digitalWrite(13, HIGH);
}

/*
 * loop for arduino
 */
void loop() {
  static int64_t previous_counter = 0;
  
  // Publish current_velocity each PID_INTERVAL_MS by millis timer
  if (millis() > next_pid) {
    next_pid += PID_INTERVAL_MS;
    //current_velocity = (current_counter - previous_counter) * 2 * PI / (CPR * PID_INTERVAL_S);
    current_velocity = (current_counter - previous_counter) * 360.0 / (CPR * PID_INTERVAL_S);
    int32_t pwm_value = pid();
    control_motor(pwm_value);
    arduino_msg.pwm = pwm_value;
    //arduino_msg.current_velocity = current_velocity * PI / 180.0;
    arduino_msg.current_velocity = current_velocity;
    previous_counter = current_counter;
    pub_arduino.publish(&arduino_msg);
  }
  nh.spinOnce();
  //delay(1);
}

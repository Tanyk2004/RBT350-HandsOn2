#include "Arduino.h"
#include "C610Bus.h"
#include "HandsOn2Util.hpp"

long last_command = 0; // To keep track of when we last commanded the motors
C610Bus<CAN2> bus;     // Initialize the Teensy's CAN bus to talk to the motors

const int LOOP_DELAY_MILLIS = 5; // Wait for 0.005s between motor updates.

// Implement your own PD controller here.
float pd_control(float pos,
                 float vel,
                 float target_position,
                 float Kp,
                 float Kd)
{
  float error = target_position - pos;
  float d_error = LOOP_DELAY_MILLIS * vel;
  return Kp * error - Kd * d_error;
}

/* Sanitize current command to make it safer.

Clips current command between bounds. Reduces command if actuator outside of position or D bounds.
Max current defaults to 1000mA. Max position defaults to +-180degs. Max velocity defaults to +-5rotations/s.
*/
void sanitize_current_command(float &command,
                              float pos,
                              float vel,
                              float max_current = 2000,
                              float max_pos = 3.141,
                              float max_vel = 30,
                              float reduction_factor = 0.1)
{
  command = command > max_current ? max_current : command;
  command = command < -max_current ? -max_current : command;
  if (pos > max_pos || pos < -max_pos)
  {
    command = 0;
    Serial.println("ERROR: Actuator position outside of allowed bounds.");
  }
  if (vel > max_vel || vel < -max_vel)
  {
    command = 0;
    Serial.println("ERROR: Actuactor velocity outside of allowed bounds. Setting torque to 0.");
  }
}

void setup()
{
  // Clean up the Serial line at the start of the program
  purgeSerial();

  // Wait for the student to press 's' in the serial monitor
  waitForStart();
}

void loop()
{
  bus.PollCAN(); // Check for messages from the motors.

  long now = millis();

  // Check if the student has pressed 's' to stop the program
  checkForStop();

  if (now - last_command >= LOOP_DELAY_MILLIS)
  {
    float m0_pos = bus.Get(0).Position(); // Get the shaft position of motor 0 in radians.
    float m0_vel = bus.Get(0).Velocity(); // Get the shaft velocity of motor 0 in radians/sec.
    Serial.print("m0_pos: ");
    Serial.print(m0_pos);
    Serial.print("\tm0_vel: ");
    Serial.print(m0_vel);

    float m0_current = 0.0;

    // Your PD controller is run here.
    float Kp = 1000.0;
    float Kd = 0;
    float target_position = 0.0; // modify in step 8
    m0_current = pd_control(m0_pos, m0_vel, target_position, Kp, Kd);

    // Uncomment for bang-bang control
    // if(m0_pos < 0) {
    //   m0_current = 800;
    // } else {
    //   m0_current = -800;
    // }

    // Sanitizes your computed current commands to make the robot safer.
    sanitize_current_command(m0_current, m0_pos, m0_vel);

    // Only call CommandTorques once per loop! Calling it multiple times will override the last command.
    bus.CommandTorques(m0_current, 0, 0, 0, C610Subbus::kIDZeroToThree);

    last_command = now;
    Serial.println();
  }
}
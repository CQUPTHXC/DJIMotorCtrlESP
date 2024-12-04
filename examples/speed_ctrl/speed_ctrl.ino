#include "../../include/DJIMotorCtrlESP.hpp"

/* 速度控制 */

// 3508电机
M3508_P19 MOTOR1(/*ID=*/1);

// 2006电机
M2006_P19 MOTOR2(/*ID=*/2);

// GM6020电机
GM6020 MOTOR3(/*ID=*/3);

void setup()
{
  // CAN初始化
  can_init(8, 18, 500);
  
  MOTOR1.setup();
  MOTOR2.setup();
  MOTOR3.setup();
}
void loop()
{
  MOTOR1.set_speed(200);
  MOTOR2.set_speed(200);
  MOTOR3.set_speed(200);
  delay(5000);
  MOTOR1.set_speed(0);
  MOTOR2.set_speed(0);
  MOTOR3.set_speed(0);
  delay(5000);

}



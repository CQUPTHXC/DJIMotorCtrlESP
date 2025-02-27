#include "DJIMotorCtrlESP.hpp"
#include "HXC_TWAI.hpp"
/* 速度控制 */


HXC_TWAI CAN_BUS(/*TX=*/8, /*RX=*/18);

// 3508电机
M3508_P19 MOTOR1(&CAN_BUS,/*ID=*/1);

// 2006电机
M2006_P19 MOTOR2(&CAN_BUS,/*ID=*/2);

// GM6020电机
GM6020 MOTOR3(&CAN_BUS,/*ID=*/3);

void setup()
{
  // CAN初始化
  CAN_BUS.setup();
  
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



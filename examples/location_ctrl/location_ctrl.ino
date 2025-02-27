#include "DJIMotorCtrlESP.hpp"
#include "HXC_TWAI.hpp"
/* 位置控制 */
HXC_TWAI CAN_BUS(/*TX=*/8, /*RX=*/18);
// 3508电机
M3508_P19 MOTOR(&CAN_BUS,1);
void setup()
{
    CAN_BUS.setup();

    MOTOR.setup();
}
void loop()
{
    MOTOR.set_location(8192 * 20);
    delay(5000);
    MOTOR.set_location(0);
    delay(5000);
}


# DJIMotorCtrlESP
重庆邮电大学HXC战队开源


[![Compile Examples](https://github.com/qingmeijiupiao/DJIMotorCtrlESP/workflows/Compile%20Examples/badge.svg)](https://github.com/qingmeijiupiao/DJIMotorCtrlESP/actions?workflow=Compile+Examples)
[![Check Arduino](https://github.com/qingmeijiupiao/DJIMotorCtrlESP/actions/workflows/check-arduino.yml/badge.svg)](https://github.com/qingmeijiupiao/DJIMotorCtrlESP/actions/workflows/check-arduino.yml)
[![Check keywords.txt](https://github.com/qingmeijiupiao/DJIMotorCtrlESP/actions/workflows/check-keywords-txt.yml/badge.svg)](https://github.com/qingmeijiupiao/DJIMotorCtrlESP/actions/workflows/check-keywords-txt.yml)

## 概述
该模块可用于esp32芯片arduino框架的大疆电机控制。
esp32系列只需要外挂一颗CAN收发器，即可实现大疆电机控制。

本文档提供了用于控制大疆电机的DJIMotorCtrlESP模块的详细API说明。

## ESP-IDF兼容
IDFCompatible文件夹中的文件扔到IDF里面并添加编译就能使用了

## 使用流程
创建电机对象->can初始化->设置PID参数(可选)->电机初始化->电机控制
```cpp
MOTOR motor1(1);/*←ID*/
GM6020 motor2(2);/*←ID*/
void setup(){
  can_init();//必须在使用电机之前调用can初始化

  //motor1.set_speed_pid(1,0,0,0,1000);//设置位置闭环控制参数
  //motor1.set_location_pid(1,0,0,0,1000);//设置位置闭环控制参数

  motor1.setup();
  motor2.setup();

  motor1.set_speed(1000);//设置电机转速
  motor2.set_angle(180.0);//设置电机转向角度
}
```
## 包含文件

```cpp
//以下3个文件包含在esp32 Arduino SDK中无需手动安装
#include <Arduino.h>
#include <map>
#include <functional>
//can通信库，本模块附带
#include "ESP_CAN.hpp"
//PID控制器文件，本模块附带
#include "PID_CONTROL.hpp"
```

## 类和函数API

### CAN初始化函数

```cpp
void can_init(uint8_t TX_PIN=8, uint8_t RX_PIN=18, int current_update_hz=200);
```
初始化CAN总线通信，必须在使用电机之前调用此函数。

- **参数**:
  - `TX_PIN`: CAN总线TX引脚，默认为8。
  - `RX_PIN`: CAN总线RX引脚，默认为18。
  - `current_update_hz`: 电流更新频率，默认为200Hz。
---
### 添加用户自定义CAN消息接收回调函数

```cpp
void add_user_can_func(uint16_t addr,std::function<void(twai_message_t* can_message)> func);
```
添加用户自定义的CAN消息接收函数，用于处理非电机的数据。

- **参数**:
  - `addr`: 要处理的消息的地址。
  - `func`: 回调函数,可以是:lamda表达式,函数指针,静态类成员函数,std::function。
- **示例**:
```cpp
void func2(twai_message_t* can_message){
    //打印收到的数据
    for(int i=0;i<can_message->data_length_code;i++){
      Serial.print(can_message->data[i],HEX);
      Serial.print(" ");
    }
    Serial.println();
}
void setup() {
  can_init();//必须初始化CAN总线才可以使用
  add_user_can_func(0x255,
    [](twai_message_t* can_message){//lamda表达式
      //打印收到的数据
      for(int i=0;i<can_message->data_length_code;i++){
        Serial.print(can_message->data[i],HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
  );//收到0x255地址消息时自动调用这个lamda表达式

  add_user_can_func(0x245,func2);//收到0x245地址消息时自动调用func2
}
```
---



### MOTOR

电机基类，没有减速箱的电机也可以使用这个类。

- **构造函数**:
  - `MOTOR(int id)`: 构造函数，初始化电机ID。
    - **参数**: `int id` - 电机的ID。
---
  - `MOTOR(int id, pid_param location_pid, pid_param speed_pid)`: 构造函数，初始化电机ID和PID参数。
    - **参数**:
      - `int id` - 电机的ID。
      - `pid_param location_pid` - 位置PID参数。
      - `pid_param speed_pid` - 速度PID参数。
---
- **成员函数**:
  - `void setup()`: 初始化电机，位置闭环，使能。
---
  - `bool is_online()`: 判断电机是否在线。
    - **返回值**: `bool` - 如果电机在线返回`true`，否则返回`false`。
---
  - `void stop(bool need_unload=true)`: 停止电机。
    - **参数**: `bool need_unload` - 是否需要卸载使能，默认为`true`。
---
  - `void set_location_pid(float _location_Kp, float _location_Ki, float _location_Kd, float __dead_zone, float _max_speed)`: 设置位置闭环控制参数。
    - **参数**:
      - `float _location_Kp` - PID的比例系数。
      - `float _location_Ki` - PID的积分系数。
      - `float _location_Kd` - PID的微分系数。
      - `float __dead_zone` - 死区。
      - `float _max_speed` - 最大速度。
---
  - `void set_speed_pid(float _speed_Kp, float _speed_Ki, float _speed_Kd, float __dead_zone, float _max_curunt)`: 设置速度闭环控制参数。
    - **参数**:
      - `float _speed_Kp` - PID的比例系数。
      - `float _speed_Ki` - PID的积分系数。
      - `float _speed_Kd` - PID的微分系数。
      - `float __dead_zone` - 死区。
      - `float _max_curunt` - 最大电流。
---
  - `void set_location(int64_t _location)`: 设置多圈目标位置。
    - **参数**: `int64_t _location` - 目标位置值。
---
  - `void reset_location(int64_t _location=0)`: 重置当前多圈位置。
    - **参数**: `int64_t _location` - 重置到的位置值，默认为0。
---
  - `int64_t get_location()`: 获取当前多圈位置。
    - **返回值**: `int64_t` - 当前位置值。
---
  - `int get_current_raw()`: 获取当前电流原始值。
    - **返回值**: `int` - 电流原始值。
---
  - `void set_max_curunt(float _max_curunt)`: 设置最大电流。
    - **参数**: `float _max_curunt` - 最大电流值。
---
  - `void unload()`: 卸载使能。
---
  - `void load()`: 使能。
---
  - `bool get_is_load()`: 获取是否使能。
    - **返回值**: `bool` - 如果电机使能返回`true`，否则返回`false`。
---
  - `float get_now_speed()`: 获取当前速度。
    - **返回值**: `float` - 当前速度值。
---
  - `void set_speed(float speed, float acce=0)`: 设置目标速度。
    - **参数**:
      - `float speed` - 目标速度值。
      - `float acce` - 加速度，默认为0。
---
  - `float get_taget_speed()`: 获取转子目标速度。
    - **返回值**: `float` - 目标速度值。
---
  - `void set_acceleration(float acce=0)`: 设置电机加速度。
    - **参数**: `float acce` - 加速度，默认为0。
---
  - `void set_speed_location_K(float _K)`: 设置速度环位置误差系数。
    - **参数**: `float _K` - 速度环位置误差系数。
---
  - `void add_location_to_current_func(std::function<int(int64_t)> func)`: 添加位置到电流映射函数。
    - **参数**: `std::function<int(int64_t)> func` - 映射函数。
---
  - `void set_control_frequency(int _control_frequency=200)`: 设置闭环控制频率。
  - 此频率影响速度闭环和位置闭环控制的频率,建议不要超过电流更新的频率。
    - **参数**: `int _control_frequency` - 闭环控制频率,默认为200HZ。
---
  - `int get_control_frequency()`: 获取闭环控制频率。
    - **返回值**: `int` - 闭环控制频率。

### M3508_P19

3508电机类,MOTOR基类相同的API不重复列出。

- **构造函数**:
  - `M3508_P19(int id)`: 构造函数，初始化电机ID。
    - **参数**: `int id` - 电机的ID。
---
  - `M3508_P19(int id, pid_param location_pid, pid_param speed_pid)`: 构造函数，初始化电机ID和PID参数。
    - **参数**:
      - `int id` - 电机的ID。
      - `pid_param location_pid` - 位置PID参数。
      - `pid_param speed_pid` - 速度PID参数。
---
- **成员函数**:
  - `void set_speed(float speed, float acce=0)`: 设置减速箱输出速度。
    - **参数**:
      - `float speed` - 目标速度值。
      - `float acce` - 加速度，默认为0。
---
  - `float get_curunt_ma()`: 获取当前电流，单位mA。
    - **返回值**: `float` - 当前电流值，单位mA。
---
  - `float get_now_speed()`: 获取减速箱输出速度。
    - **返回值**: `float` - 减速箱输出速度值。

### M2006_P36

2006电机类,MOTOR基类相同的API不重复列出。

- **构造函数**:
  - `M2006_P36(int id)`: 构造函数，初始化电机ID。
    - **参数**: `int id` - 电机的ID。
---
  - `M2006_P36(int id, pid_param location_pid, pid_param speed_pid)`: 构造函数，初始化电机ID和PID参数。
    - **参数**:
      - `int id` - 电机的ID。
      - `pid_param location_pid` - 位置PID参数。
      - `pid_param speed_pid` - 速度PID参数。
---
- **成员函数**:
  - `void set_speed(float speed, float acce=0)`: 设置减速箱输出速度。
    - **参数**:
      - `float speed` - 目标速度值。
      - `float acce` - 加速度，默认为0。
---
  - `float get_curunt_ma()`: 获取当前电流，单位mA。
    - **返回值**: `float` - 当前电流值，单位mA。
---
  - `float get_now_speed()`: 获取减速箱输出速度。
    - **返回值**: `float` - 减速箱输出速度值。

### GM6020

GM6020电机类。

- **构造函数**:
  - `GM6020(int id)`: 构造函数，初始化电机ID。
    - **参数**: `int id` - 电机的ID。
---
  - `GM6020(int id, pid_param location_pid, pid_param speed_pid)`: 构造函数，初始化电机ID和PID参数。
   - **参数**:
      - `int id` - 电机的ID。
      - `pid_param location_pid` - 位置PID参数。
      - `pid_param speed_pid` - 速度PID参数。
---
- **成员函数**:
  - `float get_curunt_ma()`: 获取实际电流，单位mA。
    - **返回值**: `float` - 当前电流值，单位mA。
---
  - `void set_angle(float angle, int8_t dir=0)`: 设置转向角度。
    - **参数**:
      - `float angle` - 目标角度值。
      - `int8_t dir` - 旋转方向(1,0,-1)，默认为0（最近方向）。
---
  - `void set_angle_offset(float offset)`: 设置角度偏移量。
    - **参数**: `float offset` - 角度偏移量(0-180)。
---
  - `float get_angle()`: 获取角度。
    - **返回值**: `float` - 当前角度值(0-360)。


## 内部调用的类和函数
在外部调用以下的类和函数是不安全的。
### C600_DATA

电调接收数据相关类，用户无需创建对象。

- **成员函数**:
  - `float get_angle()`: 返回电机的角度。
    - **返回值**: `float` - 角度值，范围0-360度。
  - `int get_speed()`: 返回电机的转速。
    - **返回值**: `int` - 转速值，单位RPM。
  - `int get_current()`: 返回电机的电流。
    - **返回值**: `int` - 电流值。
  - `int get_tempertrue()`: 获取电机的温度。
    - **返回值**: `int` - 温度值，单位摄氏度。
  - `int64_t get_location()`: 获取电机的多圈位置。
    - **返回值**: `int64_t` - 位置值，每8192为转子1圈。
  - `void reset_location(int l=0)`: 重置电机的多圈位置。
    - **参数**: `int l` - 重置到的位置值，默认为0。
  - `bool is_online()`: 判断电机是否在线。
    - **返回值**: `bool` - 如果电机在线返回`true`，否则返回`false`。


### 位置闭环控制任务

```cpp
void location_contral_task(void* n);
```
位置闭环控制任务函数，每个电机都有自己的位置闭环控制任务，用户无需调用。

### 速度闭环控制任务

```cpp
void speed_contral_task(void* n);
```
速度闭环控制任务函数，每个电机都有自己的速度闭环控制任务，用户无需调用。

### 更新电流任务

```cpp
void update_current_task(void* p);
```
更新电流任务，全局只有一个任务，用户无需调用。

### 代码示例

```c
// 速度控制
M3508_P19 MOTOR(1);
void setup()
{
  can_init(8, 18, 100);
}
void loop()
{
  MOTOR.set_speed(50, 10);
}

// 位置控制
M3508_P19 MOTOR(1);
void setup()
{
    can_init(8, 18, 100);
    MOTOR.setup();
}
void loop()
{
    MOTOR.set_location(8192 * 20);
}
```
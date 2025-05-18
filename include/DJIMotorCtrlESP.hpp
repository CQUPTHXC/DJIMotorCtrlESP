/*
 * @version: 2.2.0
 * @Description: 用于控制大疆电机
 * @Author: qingmeijiupiao
 * @Date: 2024-04-13 21:00:21
 * @LastEditTime: 2025-05-18 21:24:49
 * @LastEditors: qingmeijiupiao
 * @rely:PID_CONTROL.hpp,ESP_CAN.hpp
 */

#ifndef _DJIMotorCtrlESP_HPP_
#define _DJIMotorCtrlESP_HPP_

#include <Arduino.h>//通用库，包含freertos相关文件
#include <map> //std::map
#include <list>//std::list
#include <functional>//std::function
#include "HXC_CAN.hpp"//can通信库
#include "PID_CONTROL.hpp"//PID控制器文件


// 电机基类，支持多种类型电机（包括没有减速箱的电机）
class DJI_MOTOR {
public:
    // 禁止拷贝构造和拷贝赋值操作
    DJI_MOTOR(const DJI_MOTOR&) = delete;
    DJI_MOTOR& operator=(const DJI_MOTOR&) = delete;

    // 电机构造函数：指定CAN总线和电机ID（1-8）
    DJI_MOTOR(HXC_CAN* can_bus, int id);
    
    // 电机构造函数：指定CAN总线、ID、位置PID参数和速度PID参数
    DJI_MOTOR(HXC_CAN* can_bus, int id, pid_param location_pid, pid_param speed_pid);
    

    /**
     * @description: 初始化,必须先调用setup()才能正常控制电机
     * @return {*}
     * @Author: qingmeijiupiao
     * @param {bool} is_enable 是否使能
     */
    void setup(bool is_enable = true);

    // 判断电机是否在线
    bool is_online();

    // 停止电机，并根据需要卸载使能
    void stop(bool need_unload = true);

    // 设置位置闭环控制参数
    void set_location_pid(float _location_Kp = 0, float _location_Ki = 0, float _location_Kd = 0, float __dead_zone = 0, float _max_speed = 0);
    
    // 设置位置闭环控制参数（传入pid_param结构体）
    void set_location_pid(pid_param pid);

    // 设置速度闭环控制参数
    void set_speed_pid(float _speed_Kp = 0, float _speed_Ki = 0, float _speed_Kd = 0, float __dead_zone = 0, float _max_curunt = 0);

    // 设置速度闭环控制参数（传入pid_param结构体）
    void set_speed_pid(pid_param pid);

    // 设置多圈目标位置
    void set_location(int64_t _location);

    // 重置当前多圈位置
    void reset_location(int64_t _location = 0);

    // 获取当前多圈位置
    int64_t get_location();

    // 获取当前电流原始值
    int get_current_raw();

    // 设置最大电流（0-16384）
    void set_max_curunt(float _max_curunt);

    // 卸载使能
    void unload();

    // 使能电机
    void load();

    // 获取电机是否使能
    bool get_is_load();

    // 获取当前速度（单位：RPM）
    virtual float get_now_speed();

    // 设置目标速度，单位：RPM，加速度控制
    virtual void set_speed(float speed, float acce = 0);

    // 获取转子目标速度
    float get_target_speed();

    // 设置电机加速度，单位：RPM/s
    void set_acceleration(float acce = 0);

    // 获取减速箱减速比
    float get_reduction_ratio();

    // 设置速度环位置误差系数
    void set_speed_location_K(float _K = 1000);

    // 获取闭环控制频率
    int get_control_frequency();

    // 设置闭环控制频率 0~1000
    void set_control_frequency(int _control_frequency = 1000);

    /**
     * @description: 当电机的输出电流需要根据位置进行映射的时候，可以传入位置到电流映射函数的指针
     * 例如电机做摇臂控制电机时，电流需要做三角函数映射，那么可以传入经过三角函数的位置到电流映射函数的指针
     * @return {*}
     * @Author: qingmeijiupiao
     * @Date: 2024-04-23 14:00:29
     * @param 返回值为电流原始值(int)，输入为位置(int64_t)的映射函数或者lambda
     */
    void add_location_to_current_func(std::function<int(int64_t)> func);

protected:
    class DJI_MOTOR_DATA {
        // 内部类，处理电机的实时数据更新和获取
        friend DJI_MOTOR;

    public:
        DJI_MOTOR_DATA();
        ~DJI_MOTOR_DATA();

        // 获取电机角度（单位：度）
        float get_angle();

        // 获取转子速度（单位：RPM）
        int get_speed();

        // 获取电流值
        int get_current();

        // 获取电机温度（单位：摄氏度）
        int get_tempertrue();

        // 获取多圈位置
        int64_t get_location();

        // 重置位置
        void reset_location(int l = 0);

        // 判断电机是否在线
        bool is_online();

        // 更新电机位置
        void update_location();

        // 更新电机数据（CAN消息接收后调用）
        void update_data(HXC_CAN_message_t* can_message);

        uint16_t angle = 0;
        int16_t speed = 0;
        int16_t current = 0;
        uint8_t tempertrue = 0;
        int16_t set_current = 0;
        int64_t location = 0;
        bool enable = false;
        int64_t last_location_update_time = 0;
        uint16_t last_angle = 0;
        bool is_GM6020 = false;
    };

    //默认构造函数放这里是为了解决GM6020的初始化问题
    DJI_MOTOR(){};

    HXC_CAN* can;               // CAN总线对象
    uint8_t ID;                 // 电机ID（1-8）

    //位置到电流的映射函数，默认返回0,当电流非线性时需要重写
    std::function<int(int64_t)> location_to_current_func=[](int64_t location){return 0;};

    int64_t location_taget = 0;        // 目标位置
    int64_t speed_location_taget = 0;  // 速度目标位置
    pid_param default_location_pid_parmater={0.1,0.1,0,2000,3000};  // 默认位置PID参数
    PID_CONTROL location_pid_contraler;      // 位置PID控制器
    pid_param default_speed_pid_parmater={5,1,0.01,1,10000};    // 默认速度PID参数
    PID_CONTROL speed_pid_contraler;         // 速度PID控制器

    float max_curunt = 10000;  // 最大电流值
    DJI_MOTOR_DATA can_data;   // 电机数据对象

    float target_speed = 0;     // 目标速度
    TaskHandle_t location_func_handle = nullptr;  // 位置闭环控制任务句柄
    TaskHandle_t speed_func_handle = nullptr;     // 速度闭环控制任务句柄

    float reduction_ratio = 1; // 减速比
    float acceleration = 0;    // 电机加速度（单位：RPM/s）
    int speed_location_K = 1000;   // 速度环位置误差系数
    int control_frequency = 1000;  // 控制频率（单位：Hz）

    // CAN总线和电机对象的映射关系
    struct can_bus_to_motor {
        DJI_MOTOR* motor[8] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
        DJI_MOTOR* GM6020_motor[7] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
        uint16_t send_frequency = 1000;//发送频率 HZ
    };

    static std::map<HXC_CAN*, can_bus_to_motor> can_bus_map;  // CAN总线与电机的映射表

    // 电流控制任务的优先级
    constexpr static int current_task_Priority = 5;
    // 电流控制任务的堆栈大小  
    constexpr static int current_task_stack_size = 4096;
    // 电流控制更新任务
    static void update_current_task(void* p);

    // 速度控制任务的优先级
    constexpr static int speed_task_Priority = 5;  
    // 速度控制任务的堆栈大小
    constexpr static int speed_task_stack_size = 4096;
    // 速度闭环控制任务
    static void speed_contral_task(void* n);

    // 位置控制任务的优先级
    constexpr static int location_task_Priority = 5;  
    // 位置控制任务的堆栈大小
    constexpr static int location_task_stack_size = 4096;
    // 位置闭环控制任务
    static void location_contral_task(void* n);
};

class M3508_P19:public DJI_MOTOR{
    public:
        M3508_P19(HXC_CAN*can_bus,int id);

        M3508_P19(HXC_CAN*can_bus,int id,pid_param location_pid,pid_param speed_pid);

        //设置减速箱输出速度，单位RPM
        void set_speed(float speed,float acce=0) override;

        //获取当前电流,单位mA
        float get_curunt_ma();

        //获取减速箱输出速度，单位RPM
        float get_now_speed() override;
};


//2006电机类
class M2006_P36:public DJI_MOTOR{
    public:
        M2006_P36(HXC_CAN*can_bus,int id);

        M2006_P36(HXC_CAN*can_bus,int id,pid_param location_pid,pid_param speed_pid);

        //设置减速箱输出速度，单位RPM
        void set_speed(float speed,float acce=0) override;

        //获取当前电流,单位mA
        float get_curunt_ma();

        //获取减速箱输出速度，单位RPM
        float get_now_speed() override;
};

//GM6020类
class GM6020:public DJI_MOTOR{
    public:
        GM6020(HXC_CAN*can_bus,int id);

        GM6020(HXC_CAN*can_bus,int id,pid_param location_pid,pid_param speed_pid);

        //返回实际电流,单位mA
        float get_curunt_ma();

        //转向角度，范围：0-360，单位：度
        void set_angle(float angle,int8_t dir =0);

        //设置角度偏移量，范围：-180_180，单位：度
        void set_angle_offset(float offset);

        //返回角度，范围：0-360，单位：度
        float get_angle();

    protected:
        float angle_offset = 0;//转子角度偏移量
        
};
#endif
/*
                                              .=%@#=.                                               
                                            -*@@@@@@@#=.                                            
                                         .+%@@@@@@@@@@@@#=                                          
                                       -#@@@@@@@* =@@@@@@@@*:                                       
                                     =%@@@@@@@@=   -@@@@@@@@@#-                                     
                                  .+@@@@@@@@@@-     .@@@@@@@@@@%=                                   
                                .+@@@@@@@@@@@@-     +@@@@@@@@@@@@@+.                                
                               +@@@@@@@@@@@@@@@    .@@@@@@@@@@@@@@@@+.                              
                             =@@@@@@@@@@@@@@@%-     =%@@%@@@@@@@@@@@@@=                             
                           -%@@@@@@@@@@@@+..     .       -@@@@@@@@@@@@@%-                           
                         .#@@@@@@@@@@@@@#       -@+       +@@@@@@@@@@@@@@#:                         
                        +@@@@@@@@@@@@@@@@+     +@@@+     =@@@@@@@@@@@@@@@@@+                        
                      :%@@@@@@@@@@@@@@@@@+    *@@@@*     =@@@@@@@@@@@@@@@@@@%-                      
                     +@@@@@@@@@@@@@@#+*+-   .#@@@@+       :+*+*@@@@@@@@@@@@@@@*                     
                   :%@@@@@@@@@@@@@@+       :%@@@@-    .-       -@@@@@@@@@@@@@@@%:                   
                  =@@@@@@@@@@@@@@@@-      -@@@@%:    .%@+      =@@@@@@@@@@@@@@@@@=                  
                 *@@@@@@@@@@@@@@@@@@.    =@@@@#.    -@@@@+    =@@@@@@@@@@@@@@@@@@@#                 
               .%@@@@@@@@@@@@@@@@@@+    +@@@@*     =@@@@%:    .#@@@@@@@@@@@@@@@@@@@%.               
              :@@@@@@@@@@@@@@@%:::.    #@@@@+     +@@@@#        .::.*@@@@@@@@@@@@@@@@-              
             -@@@@@@@@@@@@@@@%       .%@@@@=     *@@@@*     +-       *@@@@@@@@@@@@@@@@=             
            =@@@@@@@@@@@@@@@@@#.    -@@@@@-    :%@@@@=    .#@@+     +@@@@@@@@@@@@@@@@@@=            
           =@@@@@@@@@@@@@@@@@@@:    =====.     -+===:     :====     @@@@@@@@@@@@@@@@@@@@+           
          +@@@@@@@@@@@@@@@#%%#-                                     :*%%#%@@@@@@@@@@@@@@@+          
         =@@@@@@@@@@@@@@%.       ...........................              *@@@@@@@@@@@@@@@=         
        =@@@@@@@@@@@@@@@+      .#@@@@@@@@@@@@@@@@@@@@@@@@@@#     .*:      =@@@@@@@@@@@@@@@@-        
       -@@@@@@@@@@@@@@@@@=    .%@@@@@@@@@@@@@@@@@@@@@@@@@@#     :@@@-    =@@@@@@@@@@@@@@@@@@:       
      :@@@@@@@@@@@@@@@@@%.   -@@@@%+=====================:     -@@@@%    :%@@@@@@@@@@@@@@@@@@.      
      %@@@@@@@@@@@@@=-=:    =@@@@#.                           +@@@@#.      -=--%@@@@@@@@@@@@@%      
     #@@@@@@@@@@@@@:       +@@@@*      ............. .       *@@@@*             %@@@@@@@@@@@@@+     
    =@@@@@@@@@@@@@@#.     #@@@@+     +@@@@@@@@@@@@@@@#.    .#@@@@+     +#.     +@@@@@@@@@@@@@@@:    
   .@@@@@@@@@@@@@@@@-   .%@@@@=     *@@@@@@@@@@@@@@@#     :%@@@@-     *@@%:    @@@@@@@@@@@@@@@@%    
   %@@@@@@@@@@@%%%#=   :@@@@@:    .#@@@@+-----------     -@@@@@:     #@@@@=    :#%%%@@@@@@@@@@@@*   
  =@@@@@@@@@@@=       -@@@@%.    :%@@@@-                =@@@@%.    .%@@@@=          :%@@@@@@@@@@@:  
  @@@@@@@@@@@%.      =@@@@#     -@@@@%:    .:::-:      +@@@@#     :@@@@@:    .       +@@@@@@@@@@@#  
 +@@@@@@@@@@@@@.    *@@@@*     =@@@@#.    -@@@@@:     #@@@@+     =@@@@%.    -@#     +@@@@@@@@@@@@@- 
.@@@@@@@@@@@@@#    *@%@%=     +@@@@*     =@@@@#.    .#@@@%=     +@@@@#     =@@@%.   =@@@@@@@@@@@@@% 
+@@@@@@@@*-==-                .          .           . ..       .....      .....     .=+=+@@@@@@@@@-
%@@@@@@@+                                                                                 -@@@@@@@@#
@@@@@@@-       =#%#=     -#%%#-     -#%%*.     +%%%*.    .*%%#=     :#%%#-     =%%%*.      .#@@@@@@@
@@@@@@=.::::::*@@@@@*:::-@@@@@@-:::=@@@@@%::::*@@@@@#::::%@@@@@+:---@@@@@@=---+@@@@@%------:=@@@@@@@
=@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@+
 *@@@@@@@@@@@@@@@@@@@@@@@@@@@%%##**++===----:::::------===++***##%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@* 
  -#@@@@@@@@@@@@@@@@%#*+=-:.                                        ..::-=+*##%@@@@@@@@@@@@@@@@@#-  
    :=*%@@@@@%#*=-:                                                             .:-=+*#%%%%##+-.    
                                                                                        
        K####      #####     ###    ###  ######.   ##########     K##    ### ###    ##W    ####W    
       #######    #######    ###    ###  ########  ##########     ###    ### ###   ###   W######    
      W###G####  ###W ####   ###    ###  ######### ##########     ###    ###  ##   ###   ###W####   
      ###   ###  ###   ###   ###    ##  ###    ###    ###         ###    ###  ### t##   ###   ###   
     G##    #   ###    ###   ##     ##  ###    ###    ###         ###    ###  ### ###   ##W         
     ###        ###    ###   ##    ###  ###    ###    ###         ##L    ##   ### ##   ###          
     ###        ###    ###  K##    ###  ###    ###    ###         ##     ##    #####   ###          
     ###       ,##     ###  ###    ###  ###   ###,    ##         G##    ###    ####    ###          
    W##        ###     ###  ###    ###  #########     ##         ##########    ####    ###          
    ###        ###     ###  ###    ###  ########     ###         ##########    ###i   K##           
    ###        ###     ###  ###    ##  #######       ###         ###    ###    ####   ###           
    ###        ###     ###  ##     ##  ###           ###         ###    ###   ##W##   ###           
    ###        ###     ##i  ##    ###  ###           ###         ###    ##    ## ##   ###           
    ###        ###    ###  ,##    ###  ###           ###         ##     ##   ### ##   ###           
    ###    ### ###    ###  K##    ###  ###           ##         t##    ###   ##  ###  ###    ###    
    ###   G##i ###   ###   .##   ###.  ##t           ##         ###    ###  ###  ###  W##,   ###    
     ########  W##W#####    ########   ##           ###         ###    ###  ##    ##   ####W###     
     #######    #######     #######   ###           ###         ###    ### ###    ##.  #######      
      #####      #####       #####    ###           ###         ###    ### ##W    ###   #####       
                   ###                                                                              
                   ###                                                                              
                   #####                                                                            
                    ####                                                                            
                      K                                                                             
*/
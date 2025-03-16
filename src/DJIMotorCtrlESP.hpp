/*
 * @version: 2.1.0
 * @Description: 用于控制大疆电机
 * @Author: qingmeijiupiao
 * @Date: 2024-04-13 21:00:21
 * @LastEditTime: 2025-03-16 20:07:48
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
    float get_taget_speed();

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

    float taget_speed = 0;     // 目标速度
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


DJI_MOTOR::DJI_MOTOR(HXC_CAN*can_bus,int id){
    //ID超过范围
    if(id<1 || id>8){
        return;
    }
    can=can_bus;
    ID=id;
    //设置CAN接收回调
    std::function<void(HXC_CAN_message_t*)> motor_data_call_back=std::bind(&DJI_MOTOR_DATA::update_data,&can_data,std::placeholders::_1);
    //注册CAN接收回调
    can->add_can_receive_callback_func(0x200+ID,motor_data_call_back);
    //设置默认PID参数
    location_pid_contraler.setPram(default_location_pid_parmater);
    speed_pid_contraler.setPram(default_speed_pid_parmater);
};

DJI_MOTOR::DJI_MOTOR(HXC_CAN*can_bus,int id,pid_param location_pid,pid_param speed_pid){
    //ID超过范围
    if(id<1 || id>8){
        return;
    }
    can=can_bus;
    ID=id;
    //设置CAN接收回调
    std::function<void(HXC_CAN_message_t*)> motor_data_call_back=std::bind(&DJI_MOTOR_DATA::update_data,&can_data,std::placeholders::_1);
    //注册CAN接收回调
    can->add_can_receive_callback_func(0x200+ID,motor_data_call_back);
    //设置默认PID参数
    location_pid_contraler.setPram(location_pid);
    speed_pid_contraler.setPram(speed_pid);
};



void DJI_MOTOR::setup(bool is_enable){
    if(!can->get_setup_flag()){
        can->setup(CAN_RATE_1MBIT);
    }
    if(can_bus_map.find(can)==can_bus_map.end()){
        can_bus_map[can]=can_bus_to_motor();
        xTaskCreate(update_current_task,"can_bus_task",current_task_stack_size,can,current_task_Priority,NULL);
    }

    if(this->can_data.is_GM6020){
        can_bus_map[can].GM6020_motor[ID-1]=this;
    }else{
        can_bus_map[can].motor[ID-1]=this;
    }
    
    if(is_enable){
        can_data.enable=true;
        if(speed_func_handle==nullptr){
            xTaskCreate(speed_contral_task,"speed_contral_task",speed_task_stack_size,this,speed_task_Priority,&speed_func_handle);
        }
    }
};

bool DJI_MOTOR::is_online(){
    return can_data.is_online();
};

void DJI_MOTOR::stop(bool need_unload){
    if(need_unload){
        unload();
    }
    location_taget=can_data.get_location();
    speed_location_taget=can_data.get_location();
    taget_speed=0;
    can_data.set_current=0;
};

void DJI_MOTOR::set_location_pid(pid_param location_pid){
    location_pid_contraler.setPram(location_pid);
};

void DJI_MOTOR::set_speed_pid(pid_param speed_pid){
    speed_pid_contraler.setPram(speed_pid);
};

void DJI_MOTOR::set_location_pid(float _location_Kp, float _location_Ki, float _location_Kd, float __dead_zone, float _max_speed){
    location_pid_contraler.setPram(_location_Kp, _location_Ki, _location_Kd, __dead_zone, _max_speed);
}

void DJI_MOTOR::set_speed_pid(float _speed_Kp, float _speed_Ki, float _speed_Kd, float __dead_zone, float _max_curunt){
    if(max_curunt>16384||max_curunt<=0){//最大电流限制
        max_curunt=16384;
    }
    speed_pid_contraler.setPram(_speed_Kp, _speed_Ki, _speed_Kd, __dead_zone, _max_curunt);
    max_curunt=_max_curunt;
    speed_pid_contraler.reset();
    location_pid_contraler.reset();
}

void DJI_MOTOR::set_max_curunt(float _max_curunt){
    if(_max_curunt>16384||max_curunt<=0){
        _max_curunt=16384;
    }
    speed_pid_contraler.setPram(0, 0, 0, 0, _max_curunt);
    max_curunt=_max_curunt;
}

void DJI_MOTOR::set_location(int64_t _location){
    //开启位置闭环控制任务
    if(location_func_handle==nullptr){
        xTaskCreate(location_contral_task,"location_contral_task",location_task_stack_size,this,location_task_Priority,&location_func_handle);
    }
    location_taget=_location;
}

void DJI_MOTOR::reset_location(int64_t _location){
    can_data.reset_location(_location);
}

int64_t DJI_MOTOR::get_location(){
    return can_data.location;
}


int DJI_MOTOR::get_current_raw(){
    return can_data.get_current();
}

//卸载使能
void DJI_MOTOR::unload(){
    if(speed_func_handle!=nullptr){
        vTaskDelete(speed_func_handle);
        speed_func_handle=nullptr;
    }
    if(location_func_handle!=nullptr){
        vTaskDelete(location_func_handle);
        location_func_handle=nullptr;
    }
    this->taget_speed=0;
    this->can_data.set_current=0;
    delay(30);//等待电流更新到电调
    this->can_data.enable=false;
}

//使能
void DJI_MOTOR::load(){
    taget_speed = 0;
    this->can_data.enable=true;
    if(speed_func_handle==nullptr){
        xTaskCreate(speed_contral_task,"speed_contral_task",speed_task_stack_size,this,speed_task_Priority,&speed_func_handle);
    }
}

//获取是否使能
bool DJI_MOTOR::get_is_load(){
    return speed_func_handle!=nullptr;
}

//获取当前速度
float DJI_MOTOR::get_now_speed(){
    return can_data.speed;
}

//设置目标速度,acce为电机加速度，当acce为0的时候，不启用加速度控制
void DJI_MOTOR::set_speed(float speed,float acce){
    acce=acce>0?acce:0;

    this->can_data.enable=true;
    if(location_func_handle!=nullptr){
        vTaskDelete(location_func_handle);
        location_func_handle=nullptr;
    }
    if(speed_func_handle==nullptr){
        xTaskCreate(speed_contral_task,"speed_cspeed_func_handleontral_task",speed_task_stack_size,this,speed_task_Priority,&speed_func_handle);
    }
    taget_speed = speed;
    acceleration=acce;
}

//获取转子目标速度
float DJI_MOTOR::get_taget_speed(){
    return taget_speed;
}

//设置电机加速度,当acce为0的时候,不启用加速度控制
void DJI_MOTOR::set_acceleration(float acce){
    acce=acce>0?acce:0;
    acceleration=acce;
}

//获取减速箱减速比
float DJI_MOTOR::get_reduction_ratio(){
    return reduction_ratio;
}

//设置速度环位置误差系数,可以理解为转子每相差一圈位置误差就加 Kp RPM速度补偿
void DJI_MOTOR::set_speed_location_K(float _K){//可以简单理解为K越大转子越"硬",默认值为1000
    _K=_K>0?_K:-_K;
    speed_location_K=_K;
}

//获取闭环控制频率
int DJI_MOTOR::get_control_frequency(){
    return control_frequency;
}

//设置闭环控制频率 1-1000 默认为200
void DJI_MOTOR::set_control_frequency(int _control_frequency){
    if(_control_frequency<1){
        _control_frequency=50;
    }else if(_control_frequency>1000){
        _control_frequency=1000;
    }
    control_frequency=_control_frequency;
}

//预加载电流函数
void DJI_MOTOR::add_location_to_current_func(std::function<int(int64_t)> func){// 建议传入std::function或者lambda
    location_to_current_func=func;
}

//速度闭环控制任务
void DJI_MOTOR::speed_contral_task(void* n){
    DJI_MOTOR* moto = (DJI_MOTOR*) n;
    //上次更新时间
    int last_update_speed_time=micros();
    moto->speed_pid_contraler.reset();
    //在unload过后出现扰动，再次load之后不会回到扰动前的位置
    moto->speed_location_taget = moto->can_data.location;

    float taget_control_speed = moto->taget_speed;
    float last_taget_control_speed = moto->taget_speed;
    auto xLastWakeTime = xTaskGetTickCount();
    while (1){
        float delta_time=1e-6*(micros()-last_update_speed_time); 
        
        if(moto->acceleration==0){
            taget_control_speed = moto->taget_speed;
        }else{
            //如果启用了加速度控制
            if(abs(taget_control_speed-last_taget_control_speed)>delta_time*moto->acceleration){
                //根据加速度控制速度
                taget_control_speed = last_taget_control_speed+(taget_control_speed-last_taget_control_speed)>0?delta_time*moto->acceleration:-delta_time*moto->acceleration;
            }else{
                taget_control_speed = taget_control_speed;
            }
        }
        //更新上次的设定速度
        last_taget_control_speed = taget_control_speed;

        //由速度计算得到的目标位置
        moto->speed_location_taget+=moto->can_data.is_online()*8192*taget_control_speed*delta_time/60;//位置误差,只有电机在线才计算累计位置

        //更新上次更新时间
        last_update_speed_time=micros();


        //由速度误差和位置误差一同计算电流
        double err = 
        /*速度环的误差=*/(taget_control_speed - moto->can_data.speed)
        +
        /*速度环位置误差比例系数=*/moto->speed_location_K/*这里的比例系数需要根据实际情况调整,比例系数speed_location_K可以理解为转子每相差一圈加 speed_location_K RPM速度补偿*/
        * 
        /*由速度计算得到的目标位置的误差*/(moto->speed_location_taget-moto->can_data.location)/8192;

        
        //计算控制电流
        int16_t cru = 
        /*位置映射的电流=*/moto->location_to_current_func(moto->can_data.location)
        +
        /*PID控制器的计算电流=*/moto->speed_pid_contraler.control(moto->can_data.is_online()*err);//电机在线才计算电流

        moto->can_data.set_current = cru;//设置电
        // 根据数据频率设置延时 
        xTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / moto->control_frequency);
    }
}
//位置闭环控制任务
void DJI_MOTOR::location_contral_task(void* n){
    DJI_MOTOR* moto = (DJI_MOTOR*) n;
    moto->location_pid_contraler.reset();//重置位置闭环控制器
    float speed=0;
    auto xLastWakeTime = xTaskGetTickCount();
    while (1){
        //位置闭环控制,由位置误差决定速度,再由速度误差决定电流
        speed = moto->location_pid_contraler.control(moto->location_taget - moto->can_data.location);
        moto->taget_speed = speed;
        // 根据数据频率设置延时 
        xTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / moto->control_frequency);
    }
};

//更新电流控制任务,每个can总线对应一个电流任务
void DJI_MOTOR::update_current_task(void* p){
    //获取can总线
    HXC_CAN* can_bus = (HXC_CAN*)p;

    //获取can总线对应的电机
    can_bus_to_motor& motors = can_bus_map[can_bus];

    auto xLastWakeTime = xTaskGetTickCount();
    while(1){
        int16_t current_data[8]={0,0,0,0,0,0,0,0};
        int16_t GM6020_current_data[7]={0,0,0,0,0,0,0};
        for(int i=0;i<7;i++){
            if(motors.motor[i] != nullptr){
                if(motors.motor[i]->can_data.enable){
                    current_data[i] = motors.motor[i]->can_data.set_current;
                }
            }
            if(motors.GM6020_motor[i] != nullptr){
                if(motors.GM6020_motor[i]->can_data.enable){
                    GM6020_current_data[i] = motors.GM6020_motor[i]->can_data.set_current;
                }
            }
        }
        if(motors.motor[7] != nullptr){
            if(motors.motor[7]->can_data.enable){
                current_data[7] = motors.motor[7]->can_data.set_current;
            }
        }
        
        if(current_data[0]!=0 || current_data[1]!=0 || current_data[2]!=0 || current_data[3]!=0){
            HXC_CAN_message_t tx_msg;
            tx_msg.data_length_code=8;
            tx_msg.identifier = 0x200;
            tx_msg.self=0;
            tx_msg.extd=0;
            tx_msg.data[0] = current_data[0] >> 8;
            tx_msg.data[1] = current_data[0]&0xff;
            tx_msg.data[2] = current_data[1] >> 8;
            tx_msg.data[3] = current_data[1]&0xff;
            tx_msg.data[4] = current_data[2] >> 8;
            tx_msg.data[5] = current_data[2]&0xff;
            tx_msg.data[6] = current_data[3] >> 8;
            tx_msg.data[7] = current_data[3]&0xff;
            can_bus->send(&tx_msg);
        }

        //如果启用了(C620 C610)5-8号任意一个电机就更新电流
        if(current_data[4]!=0 || current_data[5]!=0 || current_data[6]!=0 || current_data[7]!=0){
            HXC_CAN_message_t tx_msg;
            tx_msg.data_length_code=8;
            tx_msg.identifier = 0x1FF;
            tx_msg.self=0;
            tx_msg.extd=0;
            tx_msg.data[0] = current_data[4] >> 8;
            tx_msg.data[1] = current_data[4]&0xff;
            tx_msg.data[2] = current_data[5] >> 8;
            tx_msg.data[3] = current_data[5]&0xff;
            tx_msg.data[4] = current_data[6] >> 8;
            tx_msg.data[5] = current_data[6]&0xff;
            tx_msg.data[6] = current_data[7] >> 8;
            tx_msg.data[7] = current_data[7]&0xff;
            can_bus->send(&tx_msg);
        }
        //如果启用了GM6020 1-4号任意一个电机就更新电流
        if(GM6020_current_data[0]!=0 || GM6020_current_data[1]!=0 || GM6020_current_data[2]!=0 || GM6020_current_data[3]!=0){
            HXC_CAN_message_t tx_msg;
            tx_msg.data_length_code=8;
            tx_msg.identifier = 0x1FE;
            tx_msg.self=0;
            tx_msg.extd=0;
            tx_msg.data[0] = GM6020_current_data[0] >> 8;
            tx_msg.data[1] = GM6020_current_data[0]&0xff;
            tx_msg.data[2] = GM6020_current_data[1] >> 8;
            tx_msg.data[3] = GM6020_current_data[1]&0xff;
            tx_msg.data[4] = GM6020_current_data[2] >> 8;
            tx_msg.data[5] = GM6020_current_data[2]&0xff;
            tx_msg.data[6] = GM6020_current_data[3] >> 8;
            tx_msg.data[7] = GM6020_current_data[3]&0xff;
            can_bus->send(&tx_msg);
        }
        //如果启用了GM6020 5-8号任意一个电机就更新电流
        if(GM6020_current_data[4]!=0 || GM6020_current_data[5]!=0 || GM6020_current_data[6]!=0){
            HXC_CAN_message_t tx_msg;
            tx_msg.data_length_code=8;
            tx_msg.identifier = 0x2FE;
            tx_msg.self=0;
            tx_msg.extd=0;
            tx_msg.data[0] = GM6020_current_data[4] >> 8;
            tx_msg.data[1] = GM6020_current_data[4]&0xff;
            tx_msg.data[2] = GM6020_current_data[5] >> 8;
            tx_msg.data[3] = GM6020_current_data[5]&0xff;
            tx_msg.data[4] = GM6020_current_data[6] >> 8;
            tx_msg.data[5] = GM6020_current_data[6]&0xff;
            tx_msg.data[6] = 0;
            tx_msg.data[7] = 0;
            can_bus->send(&tx_msg);
        }

        // 根据数据频率设置延时 
        xTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / motors.send_frequency);
    }
}

std::map<HXC_CAN*, DJI_MOTOR::can_bus_to_motor> DJI_MOTOR::can_bus_map={};

/*↓↓↓↓↓内部类DJI_MOTOR_DATA的实现↓↓↓↓↓*/


DJI_MOTOR::DJI_MOTOR_DATA::DJI_MOTOR_DATA(){};

DJI_MOTOR::DJI_MOTOR_DATA::~DJI_MOTOR_DATA(){};

//返回角度，范围：0-360，单位：度
float DJI_MOTOR::DJI_MOTOR_DATA::get_angle(){
    return 360.f*angle/8192.0;    
}
//返回转子速度，单位：RPM
int DJI_MOTOR::DJI_MOTOR_DATA::get_speed(){
    return speed;
}
//返回电流
int DJI_MOTOR::DJI_MOTOR_DATA::get_current(){
    return current;
}
//获取电机温度，单位：摄氏度
int DJI_MOTOR::DJI_MOTOR_DATA::get_tempertrue(){
    return tempertrue;
}
//获取多圈位置,每8192为转子1圈
int64_t DJI_MOTOR::DJI_MOTOR_DATA::get_location(){
    return location;
}
//重置当前多圈位置
void DJI_MOTOR::DJI_MOTOR_DATA::reset_location(int l){
    location = l;
}
//判断电机是否在线
bool DJI_MOTOR::DJI_MOTOR_DATA::is_online(){
    //超过100ms没有更新，就认为电机不在线
    return micros()-last_location_update_time<100000;//100ms
}
//多圈位置获取
void DJI_MOTOR::DJI_MOTOR_DATA::update_location(){
    int16_t now_angle=angle;
    if (last_location_update_time==0){
        last_location_update_time=micros();
    }

    int now = micros();

    int delta=0;

    if((now_angle+8192-last_angle)%8192<4096){//正转
        delta=now_angle-last_angle;
        if (delta<0){
            delta+=8192;
        }
    }else{
        delta=now_angle-last_angle;
        if (delta>0){
            delta-=8192;
        }
    }

    location += delta;
    last_location_update_time=now;
    last_angle=now_angle;   
}



//将CAN数据更新到变量
void DJI_MOTOR::DJI_MOTOR_DATA::update_data(HXC_CAN_message_t* can_message){
    angle = can_message->data[0]<<8 | can_message->data[1];
    speed = can_message->data[2]<<8 | can_message->data[3];
    current = can_message->data[4]<<8 | can_message->data[5];
    tempertrue = can_message->data[6];
    update_location();
}




/*M3508_P19的实现*/

M3508_P19::M3508_P19(HXC_CAN* can_bus,int id):DJI_MOTOR::DJI_MOTOR(can_bus,id){
    reduction_ratio=19.0;//默认19的减速比
};

M3508_P19::M3508_P19(HXC_CAN*can_bus,int id,pid_param location_pid,pid_param speed_pid):DJI_MOTOR::DJI_MOTOR(can_bus,id,location_pid,speed_pid){
    reduction_ratio=19.0;//默认19的减速比
};

//设置减速箱输出速度，单位RPM
void M3508_P19::set_speed(float speed,float acce){
    acce=acce>0?acce:0;
    this->can_data.enable=true;
    if(location_func_handle!=nullptr){
        vTaskDelete(location_func_handle);
        location_func_handle=nullptr;
    }
    if(speed_func_handle==nullptr){
        xTaskCreate(speed_contral_task,"speed_cspeed_func_handleontral_task",speed_task_stack_size,this,speed_task_Priority,&speed_func_handle);
    }
    taget_speed = speed*19.0;
    acceleration=acce;
}
//获取当前电流,单位mA
float M3508_P19::get_curunt_ma(){
    return 2e4*can_data.current/16384;
}
//获取减速箱输出速度，单位RPM
float M3508_P19::get_now_speed(){
    return can_data.speed/19.0;
}




/*M2006_P36的实现*/

M2006_P36::M2006_P36(HXC_CAN*can_bus,int id):DJI_MOTOR(can_bus,id){
    reduction_ratio=36.0;
};
M2006_P36::M2006_P36(HXC_CAN*can_bus,int id,pid_param location_pid,pid_param speed_pid):DJI_MOTOR(can_bus,id,location_pid,speed_pid){
    reduction_ratio=36.0;
};
//设置减速箱输出速度，单位RPM
void M2006_P36::set_speed(float speed,float acce){
    acce=acce>0?acce:0;
    this->can_data.enable=true;
    if(location_func_handle!=nullptr){
        vTaskDelete(location_func_handle);
        location_func_handle=nullptr;
    }
    if(speed_func_handle==nullptr){
        xTaskCreate(speed_contral_task,"speed_contral_task",speed_task_stack_size,this,speed_task_Priority,&speed_func_handle);
    }
    taget_speed = speed*36.0;
    acceleration=acce;
}
//获取当前电流,单位mA
float M2006_P36::get_curunt_ma(){
    return 1e4*can_data.current/16384;
}
//获取减速箱输出速度，单位RPM
float M2006_P36::get_now_speed(){
    return can_data.speed/36.0;
}




/*GM6020的实现*/

GM6020::GM6020(HXC_CAN*can_bus,int id){
    if(id<1 || id>7){
        return;
    }
    can=can_bus;
    ID=id;
    //设置CAN接收回调
    std::function<void(HXC_CAN_message_t*)> motor_data_call_back=std::bind(&DJI_MOTOR_DATA::update_data,&can_data,std::placeholders::_1);
    //注册CAN接收回调
    can->add_can_receive_callback_func(0x204+ID,motor_data_call_back);

    can_data.enable=true;
    can_data.is_GM6020=true;
    //设置默认PID参数
    pid_param speed_pid_parmater(10,0,0,1,16384);
    pid_param location_pid_parmater(0.2,0.1,0,5,350);
    location_pid_contraler.setPram(location_pid_parmater);
    speed_pid_contraler.setPram(speed_pid_parmater);
};

GM6020::GM6020(HXC_CAN*can_bus,int id,pid_param location_pid,pid_param speed_pid){
    if(id<1 || id>7){
        return;
    }
    can=can_bus;
    ID=id;
    //设置CAN接收回调
    std::function<void(HXC_CAN_message_t*)> motor_data_call_back=std::bind(&DJI_MOTOR_DATA::update_data,&can_data,std::placeholders::_1);
    //注册CAN接收回调
    can->add_can_receive_callback_func(0x204+ID,motor_data_call_back);
    
    can_data.enable=true;
    can_data.is_GM6020=true;
    //设置PID参数
    location_pid_contraler.setPram(location_pid);
    speed_pid_contraler.setPram(speed_pid);
};

//返回实际电流,单位mA
float GM6020::get_curunt_ma(){
    return 3e3*can_data.current/16384;
}

//转向角度，范围：0-360，单位：度
void GM6020::set_angle(float angle,int8_t dir){//dir:0为最近方向,1为正方向,-1为负方向
    angle-=angle_offset;
    reset_location(can_data.angle);
    float now_angle = get_angle();
    // 确保 dir 为 -1、0 或 1
    dir = dir > 0 ? 1 : (dir < 0 ? -1 : 0);
    //确保angle在0-360度之间
    angle=fmodf(angle,360.f);
    angle=angle>=0?angle:360.f+angle;

    //电机需要旋转的角度
    float delta = angle - now_angle;


    while(dir*delta<0){//当dir不为0时向指定方向绕圈
        delta+=dir*360;
    }
    if(abs(delta)>180&&dir==0){//找到最近方向
        delta+=delta>0?-360:360;
    }
    set_location(can_data.angle+delta*8192.f/360.f);
    
}

//设置角度偏移量，范围：-180_180，单位：度
void GM6020::set_angle_offset(float offset){
    if(offset<-180.f){
        while(offset<-180.f){
            offset+=360.f;
        }
    }
    if(offset>180.f){
        while(offset>180.f){
            offset-=360.f;
        }
    }
    angle_offset = offset;
}

//返回角度，范围：0-360，单位：度
float GM6020::get_angle(){
    float angle_data=can_data.get_angle();//获取原始角度
    angle_data+=angle_offset;//加上角度偏移
    angle_data=fmodf(angle_data,360.f);//确保在0-360度之间
    angle_data=angle_data>=0?angle_data:360.f+angle_data;
    return angle_data;   
}

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
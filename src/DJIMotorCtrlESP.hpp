/*
 * @Description: 用于控制大疆电机
 * @Author: qingmeijiupiao
 * @Date: 2024-04-13 21:00:21
 * @LastEditTime: 2024-10-27 19:55:56
 * @LastEditors: qingmeijiupiao
 * @rely:PID_CONTROL.hpp
*/
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
#ifndef _DJIMotorCtrlESP_HPP_
#define _DJIMotorCtrlESP_HPP_


#include <Arduino.h>//通用库，包含freertos相关文件
#include <map> //std::map
#include <functional>//std::function
#include "ESP_CAN.hpp"//can通信库
#include "PID_CONTROL.hpp"//PID控制器文件
/*↓↓↓本文件的类和函数↓↓↓*/

//CAN初始化函数，要使用电机，必须先调用此函数初始化CAN通信
void can_init(uint8_t TX_PIN=8,uint8_t RX_PIN=18,/*电流更新频率=*/int current_update_hz=200);

//由于接受电调数据的类，用户无需创建对象
class C600_DATA;

//电机基类,电机无减速箱时使用
class MOTOR;

//3508电机类
class M3508_P19;

//2006电机类
class M2006_P36;

//GM6020电机类
class GM6020;

//位置闭环控制任务函数,使用位置控制函数时创建此任务,基于速度闭环控制任务函数,每个电机都有自己的位置闭环控制任务，用户无需调用
void location_contral_task(void* n);

//速度闭环控制任务函数，每个电机都有自己的速度闭环控制任务，用户无需调用
void speed_contral_task(void* n);

//更新电流任务，全局只有一个任务，用户无需调用
void update_current_task(void* p);

/*↑↑↑本文件的类和函数↑↑↑*/


//电调接收数据相关,用户无需创建对象
class C600_DATA{
    //定义为友元，方便调用
    friend void moto_fb_fun(twai_message_t* can_message);//电机反馈函数
    friend void update_current_task(void* n); //更新电流任务，全局只有一个任务
    friend void location_contral_task(void* n);
    friend void speed_contral_task(void* n);//速度闭环控制任务线程，每个电机都有自己的速度闭环控制任务，并且与位置闭环控制任务不同时存在
    friend MOTOR;
    friend M3508_P19;
    friend M2006_P36;
    friend GM6020;

public:
    C600_DATA(){}
    ~C600_DATA(){}
    //返回角度，范围：0-360，单位：度
    float get_angle(){
        return 360.f*angle/8192.0;    
    }
    //返回转子速度，单位：RPM
    int get_speed(){
        return speed;
    }
    //返回电流
    int get_current(){
        return current;
    }
    //获取电机温度，单位：摄氏度
    int get_tempertrue(){
        return tempertrue;
    }
    //获取多圈位置,每8192为转子1圈
    int64_t get_location(){
        return location;
    }
    //重置当前多圈位置
    void reset_location(int l=0){
        location = l;
    }
    //判断电机是否在线
    bool is_online(){
        //超过100ms没有更新，就认为电机不在线
        return micros()-last_location_update_time<100000;//100ms
    }

protected:
    //多圈位置获取
    void update_location(){
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
    void update_data(twai_message_t can_message){
        angle = can_message.data[0]<<8 | can_message.data[1];
        speed = can_message.data[2]<<8 | can_message.data[3];
        current = can_message.data[4]<<8 | can_message.data[5];
        tempertrue = can_message.data[6];
        update_location();
    }
    
    uint16_t angle=0;
    int16_t speed=0;
    int16_t current=0;
    uint8_t tempertrue=0;
    int16_t set_current=0;
    int64_t location =0;
    bool enable=false;
    int64_t last_location_update_time=0;
    uint16_t last_angle=0;
    bool is_GM6020 = false;

};


//1-8号电机数据接收对象,用户无需访问
C600_DATA motor_201,motor_202,motor_203,motor_204,motor_205,motor_206,motor_207,motor_208/*后面是GM6020专属地址→*/,motor_209,motor_20A,motor_20B;
C600_DATA* motors[]={&motor_201,&motor_202,&motor_203,&motor_204,&motor_205,&motor_206,&motor_207,&motor_208,&motor_209,&motor_20A,&motor_20B};








//电机基类，没有减速箱的电机也可以使用这个类
class MOTOR{
        friend void location_contral_task(void* n);
        friend void speed_contral_task(void* n);
        friend void update_current_task(void* n);
    public:
        //禁止电机拷贝传递
        MOTOR(const MOTOR&) = delete;
        MOTOR& operator=(const MOTOR&) = delete;

        //id从1-8
        MOTOR(int id){
            //ID超过范围
            if(id<1 || id>8){
                return;
            }
            ID=id;
            data = motors[id-1];
            data->enable=true;
            //设置默认PID参数
            location_pid_contraler.setPram(default_location_pid_parmater);
            speed_pid_contraler.setPram(default_speed_pid_parmater);
            
        }
        MOTOR(int id,pid_param location_pid,pid_param speed_pid){
            //ID超过范围
            if(id<1 || id>8){
                return;
            }
            ID=id;
            data = motors[id-1];
            data->enable=true;
            //设置PID参数
            location_pid_contraler.setPram(location_pid);
            speed_pid_contraler.setPram(speed_pid);
        }
        //初始化,位置闭环,使能
        void setup(){
            data->enable=true;
            if(speed_func_handle==nullptr){
                xTaskCreate(speed_contral_task,"speed_contral_task",4096,this,2,&speed_func_handle);
            }
        };
        //判断电机是否在线
        bool is_online(){
            return data->is_online();
        }
        //停止电机 need_unload:是否卸载使能
        void stop(bool need_unload=true){
            if(need_unload){
                unload();
            }
            location_taget=data->get_location();
            speed_location_taget=data->get_location();
            taget_speed=0;
            data->set_current=0;
        }
        //设置位置闭环控制参数
        void set_location_pid(float _location_Kp=0,float _location_Ki=0,float _location_Kd=0,float __dead_zone=0,float _max_speed=0){
                location_pid_contraler.Kp=_location_Kp;
                location_pid_contraler.Ki=_location_Ki;
                location_pid_contraler.Kd=_location_Kd;
                location_pid_contraler._dead_zone=__dead_zone;
                location_pid_contraler._max_value=_max_speed;
        }
        //设置位置闭环控制参数
        void set_location_pid(pid_param pid){
            set_location_pid(pid.Kp,pid.Ki,pid.Kd,pid._dead_zone,pid._max_value);
        }
        //设置速度闭环控制参数
        void set_speed_pid(float _speed_Kp=0,float _speed_Ki=0,float _speed_Kd=0,float __dead_zone=0,float _max_curunt=0){ 
                speed_pid_contraler.Kp=_speed_Kp;
                speed_pid_contraler.Ki=_speed_Ki;
                speed_pid_contraler.Kd=_speed_Kd;
                speed_pid_contraler._dead_zone=__dead_zone;
                max_curunt=_max_curunt;
                if(max_curunt>16384||max_curunt<=0){//最大电流限制
                    max_curunt=16384;
                }
                speed_pid_contraler._max_value=_max_curunt;
                speed_pid_contraler.reset();
                location_pid_contraler.reset();
        }
        //设置速度闭环控制参数
        void set_speed_pid(pid_param pid){
            set_speed_pid(pid.Kp,pid.Ki,pid.Kd,pid._dead_zone,pid._max_value);
        }
        //设置多圈目标位置
        void set_location(int64_t _location){
            //开启位置闭环控制任务
            if(location_func_handle==nullptr){
                xTaskCreate(location_contral_task,"location_contral_task",4096,this,2,&location_func_handle);
            }
            location_taget=_location;
        }
        //重置当前多圈位置
        void reset_location(int64_t _location=0){
            data->reset_location(_location);
        }
        //获取当前多圈位置
        int64_t get_location(){
            return data->location;
        }
        //获取当前电流原始值
        int get_current_raw(){
            return data->get_current();
        }
        //设置最大电流0-16384
        void set_max_curunt(float _max_curunt){
            if(_max_curunt>16384||max_curunt<=0)
                _max_curunt=16384;
            max_curunt=_max_curunt;
        }
        //卸载使能
        void unload(){
            if(speed_func_handle!=nullptr){
                vTaskDelete(speed_func_handle);
                speed_func_handle=nullptr;
            }
            if(location_func_handle!=nullptr){
                vTaskDelete(location_func_handle);
                location_func_handle=nullptr;
            }
            this->taget_speed=0;
            this->data->set_current=0;
            delay(30);//等待电流更新到电调
            this->data->enable=false;
        }
        //使能
        void load(){
            taget_speed = 0;
            this->data->enable=true;
            if(speed_func_handle==nullptr){
                xTaskCreate(speed_contral_task,"speed_contral_task",4096,this,2,&speed_func_handle);
            }
        }
        //获取是否使能
        bool get_is_load(){
            return speed_func_handle!=nullptr;
        }
        //获取当前速度
        virtual float get_now_speed(){
            return data->speed;
        }
        //设置目标速度,acce为电机加速度，当acce为0的时候，不启用加速度控制
        virtual void set_speed(float speed,float acce=0){
            acce=acce>0?acce:0;

            this->data->enable=true;
            if(location_func_handle!=nullptr){
                vTaskDelete(location_func_handle);
                location_func_handle=nullptr;
            }
            if(speed_func_handle==nullptr){
                xTaskCreate(speed_contral_task,"speed_cspeed_func_handleontral_task",4096,this,2,&speed_func_handle);
            }
            taget_speed = speed;
            acceleration=acce;
        }
        //获取转子目标速度
        float get_taget_speed(){
            return taget_speed;
        }
        //设置电机加速度,当acce为0的时候,不启用加速度控制
        void set_acceleration(float acce=0){
            acce=acce>0?acce:0;
            acceleration=acce;
        }
        //获取减速箱减速比
        float get_reduction_ratio(){
            return reduction_ratio;
        }
        //设置速度环位置误差系数,可以理解为转子每相差一圈位置误差就加 Kp RPM速度补偿
        void set_speed_location_K(float _K=1000){//可以简单理解为K越大转子越"硬",默认值为1000
            _K=_K>0?_K:-_K;
            speed_location_K=_K;
        }
        //获取闭环控制频率
        int get_control_frequency(){
            return control_frequency;
        }
        //设置闭环控制频率 1-1000 默认为200
        void set_control_frequency(int _control_frequency=200){
            if(_control_frequency<1){
                _control_frequency=50;
            }else if(_control_frequency>1000){
                _control_frequency=1000;
            }
            control_frequency=_control_frequency;
        }
        /**
         * @description: 当电机的输出电流需要根据位置进行映射的时候，可以传入位置到电流映射函数的指针
         * 例如电机做摇臂控制电机时，电流需要做三角函数映射，那么可以传入经过三角函数的位置到电流映射函数的指针
         * @return {*}
         * @Author: qingmeijiupiao
         * @Date: 2024-04-23 14:00:29
         * @param 返回值为电流值(int)，输入为位置(int64_t)的映射函数或者lambda
         */
        void add_location_to_current_func(std::function<int(int64_t)> func){// 建议传入std::function或者lambda
            location_to_current_func=func;
        }
        void add_location_to_current_func(int (*func_ptr)(int64_t location)){// 传入函数指针的重载
            location_to_current_func=func_ptr;
        }
    protected:
        MOTOR(){};
        //位置到电流的映射函数，默认返回0,当电流非线性时需要重写
        std::function<int(int64_t)> location_to_current_func=null_location_to_current;
        uint8_t ID;
        static int null_location_to_current(int64_t location){
            return 0;
        }
        int64_t location_taget=0;//位置环多圈目标位置
        int64_t speed_location_taget=0;//速度环多圈目标位置
        pid_param default_location_pid_parmater={0.1,0.1,0,2000,3000};//位置闭环默认控制参数   
        PID_CONTROL location_pid_contraler;//位置闭环控制PID计算对象
        pid_param default_speed_pid_parmater={5,1,0.01,1,10000};//速度闭环默认控制参数
        PID_CONTROL speed_pid_contraler;//速度闭环控制PID计算对象
        float max_curunt=10000;//电流值范围0-16384
        C600_DATA* data;//数据对象
        float taget_speed = 0;//单位RPM
        TaskHandle_t location_func_handle = nullptr;//位置闭环控制任务句柄
        TaskHandle_t speed_func_handle = nullptr;//速度闭环控制任务句柄
        float reduction_ratio=1;//减速比
        float acceleration=0;//电机加速度,0为不启用加速度控制
        int speed_location_K=1000;//速度环位置误差系数,这里的比例系数需要根据实际情况调整,比例系数speed_location_K可以理解为转子每相差一圈加 speed_location_K RPM速度补偿
        int control_frequency=200;//速度闭环以及位置闭环控制频率，单位Hz
};








//3508类
class M3508_P19:public MOTOR{
    public:
        M3508_P19(int id):MOTOR(id){
            reduction_ratio=19.0;
        };
        M3508_P19(int id,pid_param location_pid,pid_param speed_pid):MOTOR(id,location_pid,speed_pid){
            reduction_ratio=19.0;
        };
        //设置减速箱输出速度，单位RPM
        void set_speed(float speed,float acce=0) override{
            acce=acce>0?acce:0;
            this->data->enable=true;
            if(location_func_handle!=nullptr){
                vTaskDelete(location_func_handle);
                location_func_handle=nullptr;
            }
            if(speed_func_handle==nullptr){
                xTaskCreate(speed_contral_task,"speed_cspeed_func_handleontral_task",4096,this,2,&speed_func_handle);
            }
            taget_speed = speed*19.0;
            acceleration=acce;
        }
        //获取当前电流,单位mA
        float get_curunt_ma(){
            return 2e4*data->current/16384;
        }
        //获取减速箱输出速度，单位RPM
        float get_now_speed() override{
            return data->speed/19.0;
        }
};





//2006电机类
class M2006_P36:public MOTOR{
    public:
        M2006_P36(int id):MOTOR(id){
            reduction_ratio=36.0;
        };
        M2006_P36(int id,pid_param location_pid,pid_param speed_pid):MOTOR(id,location_pid,speed_pid){
            reduction_ratio=36.0;
        };
        //设置减速箱输出速度，单位RPM
        void set_speed(float speed,float acce=0) override{
            acce=acce>0?acce:0;
            this->data->enable=true;
            if(location_func_handle!=nullptr){
                vTaskDelete(location_func_handle);
                location_func_handle=nullptr;
            }
            if(speed_func_handle==nullptr){
                xTaskCreate(speed_contral_task,"speed_contral_task",4096,this,2,&speed_func_handle);
            }
            taget_speed = speed*36.0;
            acceleration=acce;
        }
        //获取当前电流,单位mA
        float get_curunt_ma(){
            return 1e4*data->current/16384;
        }
        //获取减速箱输出速度，单位RPM
        float get_now_speed() override{
            return data->speed/36.0;
        }
};

//GM6020类
class GM6020:public MOTOR{
    public:
        GM6020(int id){
            if(id<1 || id>7){
                return;
            }
            data=motors[id+3];
            data->enable=true;
            data->is_GM6020=true;
            //设置默认PID参数
            pid_param speed_pid_parmater(10,0,0,1,16384);
            pid_param location_pid_parmater(0.2,0.1,0,5,350);
            location_pid_contraler.setPram(location_pid_parmater);
            speed_pid_contraler.setPram(speed_pid_parmater);
        };
        GM6020(int id,pid_param location_pid,pid_param speed_pid){
            if(id<1 || id>7){
                return;
            }
            data=motors[id+3];
            data->enable=true;
            data->is_GM6020=true;
            //设置PID参数
            location_pid_contraler.setPram(location_pid);
            speed_pid_contraler.setPram(speed_pid);
        };
        //返回实际电流,单位mA
        float get_curunt_ma(){
            return 3e3*data->current/16384;
        }
        //转向角度，范围：0-360，单位：度
        void set_angle(float angle,int8_t dir =0){//dir:0为最近方向,1为正方向,-1为负方向
            angle-=angle_offset;
            reset_location(data->angle);
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
            set_location(data->angle+delta*8192.f/360.f);
            
        }
        //设置角度偏移量，范围：-180_180，单位：度
        void set_angle_offset(float offset){
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
        float get_angle(){
            float angle_data=data->get_angle();//获取原始角度
            angle_data+=angle_offset;//加上角度偏移
            angle_data=fmodf(angle_data,360.f);//确保在0-360度之间
            angle_data=angle_data>=0?angle_data:360.f+angle_data;
            return angle_data;   
        }
    protected:
        float angle_offset = 0;//转子角度偏移量
        
};
//位置闭环控制任务;
void location_contral_task(void* n){
    MOTOR* moto = (MOTOR*) n;
    moto->location_pid_contraler.reset();//重置位置闭环控制器
    float speed=0;
    while (1){
        //位置闭环控制,由位置误差决定速度,再由速度误差决定电流
        speed = moto->location_pid_contraler.control(moto->location_taget - moto->data->location);
        moto->taget_speed = speed;
        delay(1000/moto->control_frequency);
    }
};

//速度闭环控制任务
void speed_contral_task(void* n){
    MOTOR* moto = (MOTOR*) n;
    //上次更新时间
    int last_update_speed_time=micros();
    moto->speed_pid_contraler.reset();
    //在unload过后出现扰动，再次load之后不会回到扰动前的位置
    moto->speed_location_taget = moto->data->location;

    float taget_control_speed = moto->taget_speed;
    float last_taget_control_speed = moto->taget_speed;
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
        moto->speed_location_taget+=moto->data->is_online()*8192*taget_control_speed*delta_time/60;//位置误差,只有电机在线才计算累计位置

        //更新上次更新时间
        last_update_speed_time=micros();


        //由速度误差和位置误差一同计算电流
        double err = 
        /*速度环的误差=*/(taget_control_speed - moto->data->speed)
        +
        /*速度环位置误差比例系数=*/moto->speed_location_K/*这里的比例系数需要根据实际情况调整,比例系数speed_location_K可以理解为转子每相差一圈加 speed_location_K RPM速度补偿*/
        * 
        /*由速度计算得到的目标位置的误差*/(moto->speed_location_taget-moto->data->location)/8192;


        //计算控制电流
        int16_t cru = 
        /*位置映射的电流=*/moto->location_to_current_func(moto->data->location)
        +
        /*PID控制器的计算电流=*/moto->speed_pid_contraler.control(moto->data->is_online()*err);//电机在线才计算电流
        
        moto->data->set_current = cru;//设置电流
        
        delay(1000/moto->control_frequency);
    }
}




//更新电流控制任务
void update_current_task(void* p){
    
    //电流控制频率
    int frc=*(int*) p;
    
    while(1){
        //如果启用了(C620 C610)1-4号任意一个电机就更新电流
        if(motor_201.enable || motor_202.enable || motor_203.enable || motor_204.enable){
            twai_message_t tx_msg;
            tx_msg.data_length_code=8;
            tx_msg.identifier = 0x200;
            tx_msg.self=0;
            tx_msg.extd=0;
            tx_msg.data[0] = motor_201.set_current >> 8;
            tx_msg.data[1] = motor_201.set_current&0xff;
            tx_msg.data[2] = motor_202.set_current >> 8;
            tx_msg.data[3] = motor_202.set_current&0xff;
            tx_msg.data[4] = motor_203.set_current >> 8;
            tx_msg.data[5] = motor_203.set_current&0xff;
            tx_msg.data[6] = motor_204.set_current >> 8;
            tx_msg.data[7] = motor_204.set_current&0xff;
            twai_transmit(&tx_msg,portMAX_DELAY);
        }
        //如果启用了(C620 C610)5-8号任意一个电机就更新电流
        if(motor_205.enable || motor_206.enable || motor_207.enable || motor_208.enable){
            twai_message_t tx_msg;
            tx_msg.data_length_code=8;
            tx_msg.identifier = 0x1FF;
            tx_msg.self=0;
            tx_msg.extd=0;
            tx_msg.data[0] = motor_205.set_current >> 8;
            tx_msg.data[1] = motor_205.set_current&0xff;
            tx_msg.data[2] = motor_206.set_current >> 8;
            tx_msg.data[3] = motor_206.set_current&0xff;
            tx_msg.data[4] = motor_207.set_current >> 8;
            tx_msg.data[5] = motor_207.set_current&0xff;
            tx_msg.data[6] = motor_208.set_current >> 8;
            tx_msg.data[7] = motor_208.set_current&0xff;
            twai_transmit(&tx_msg,portMAX_DELAY);
        }
        //如果启用了GM6020 1-4号任意一个电机就更新电流
        if((motors[4]->enable&&motors[4]->is_GM6020)||(motors[5]->enable&&motors[5]->is_GM6020)||(motors[6]->enable&&motors[6]->is_GM6020)||(motors[7]->enable&&motors[7]->is_GM6020)){
            twai_message_t tx_msg;
            tx_msg.data_length_code=8;
            tx_msg.identifier = 0x1FE;
            tx_msg.self=0;
            tx_msg.extd=0;
            tx_msg.data[0] = motor_205.set_current >> 8;
            tx_msg.data[1] = motor_205.set_current&0xff;
            tx_msg.data[2] = motor_206.set_current >> 8;
            tx_msg.data[3] = motor_206.set_current&0xff;
            tx_msg.data[4] = motor_207.set_current >> 8;
            tx_msg.data[5] = motor_207.set_current&0xff;
            tx_msg.data[6] = motor_208.set_current >> 8;
            tx_msg.data[7] = motor_208.set_current&0xff;
            twai_transmit(&tx_msg,portMAX_DELAY);
        }
        //如果启用了GM6020 5-8号任意一个电机就更新电流
        if((motors[8]->enable&&motors[8]->is_GM6020)||(motors[9]->enable&&motors[9]->is_GM6020)||(motors[10]->enable&&motors[10]->is_GM6020)){
            twai_message_t tx_msg;
            tx_msg.data_length_code=8;
            tx_msg.identifier = 0x2FE;
            tx_msg.self=0;
            tx_msg.extd=0;
            tx_msg.data[0] = motor_209.set_current >> 8;
            tx_msg.data[1] = motor_209.set_current&0xff;
            tx_msg.data[2] = motor_20A.set_current >> 8;
            tx_msg.data[3] = motor_20A.set_current&0xff;
            tx_msg.data[4] = motor_20B.set_current >> 8;
            tx_msg.data[5] = motor_20B.set_current&0xff;
            tx_msg.data[6] = 0;
            tx_msg.data[7] = 0;
            twai_transmit(&tx_msg,portMAX_DELAY);
        }
        //延时
        delay(1000/frc);
    }
}


void moto_fb_fun(twai_message_t* can_message){//电机反馈函数
    motors[can_message->identifier-0x201]->update_data(*can_message);
}

//初始化CAN总线
void can_init(uint8_t TX_PIN, uint8_t RX_PIN,int current_update_hz){

    can_setup(TX_PIN,RX_PIN);
    xTaskCreate(update_current_task,"update_current_task",4096,&current_update_hz,5,nullptr);//电流控制任务
    for(int i=1;i<=0xB;i++){//添加电机反馈函数到can回调map
        add_user_can_func(0x200+i,moto_fb_fun);
    }
}

#endif
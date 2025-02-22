/*
 * @version: 1.0
 * @LastEditors: qingmeijiupiao
 * @Description: HXC ESP32 twai封装类 继承HXC_CAN
 * @Author: qingmeijiupiao
 * @LastEditTime: 2024-12-20 16:04:58
 * @Relay: HXC_CAN
 */
#ifndef HXC_TWAI_HPP
#define HXC_TWAI_HPP
#include "HXC_CAN.hpp"
#include "driver/twai.h" //can驱动，esp32sdk自带
#include <cstring>
// TWAI封装类
class HXC_TWAI : public HXC_CAN {
public:
    /**
     * @description: TWAI构造函数, 默认为HXC开发板A的引脚
     * @return {*}
     * @Author: qingmeijiupiao
     * @param {uint8_t} tx 连接can收发芯片TX引脚的IO号
     * @param {uint8_t} rx 连接can收发芯片RX引脚的IO号
     * @param {CAN_RATE} rate CAN速率
     */
    HXC_TWAI(uint8_t tx = 8, uint8_t rx = 18, CAN_RATE rate = CAN_RATE_1MBIT);

    /**
     * @description: TWAI初始化
     * @return {esp_err_t} 成功返回ESP_OK
     * @Author: qingmeijiupiao
     */
    esp_err_t setup();

    /**
     * @description: 发送CAN消息
     * @return {esp_err_t}
     * @Author: qingmeijiupiao
     * @param {HXC_CAN_message_t*} message CAN消息
     */
    esp_err_t send(HXC_CAN_message_t* message) override;

    /**
     * @description: 发送CAN消息
     * @return {esp_err_t}
     * @Author: qingmeijiupiao
     * @param {HXC_CAN_message_t} message CAN消息
     */
    esp_err_t send(HXC_CAN_message_t message) override;

    /**
     * @description: 停止接收CAN消息
     * @return {*}
     * @Author: qingmeijiupiao
     */
    void stop_receive();

    /**
     * @description: 停止之后使用该函数恢复接收CAN消息
     * @return {*}
     * @Author: qingmeijiupiao
     */
    void resume_receive();

    /**
     * @description: 判断此时是否在持续接收CAN消息
     * @return {bool} true:持续接收CAN消息 false:停止接收CAN消息
     * @Author: qingmeijiupiao
     */
    bool get_receive_status();

protected:
    /**
     * @description: TWAI接受数据的线程，接收到数据后转换成HXC_CAN_message，然后调用回调函数
     */
    static void twai_feedback_update_task(void* n);

    TaskHandle_t twai_fb_handle;  /**< TWAI反馈任务句柄 */
    uint8_t TX_PIN, RX_PIN;  /**< 连接CAN收发芯片的TX和RX引脚IO号 */
    HXC_CAN_message_t RX_message_buf;  /**< 接收CAN消息缓存 */
};

HXC_TWAI::HXC_TWAI(uint8_t tx, uint8_t rx, CAN_RATE rate) {
    TX_PIN = tx;
    RX_PIN = rx;
    can_rate = rate;
}

esp_err_t HXC_TWAI::setup() {
    if (is_setup == true) {// 如果已经初始化过，不再初始化
        return ESP_OK;
    }

    // 总线速率配置
    static twai_timing_config_t t_config;
    switch (this->can_rate) {
        case CAN_RATE_1MBIT:
            t_config = TWAI_TIMING_CONFIG_1MBITS();
            break;
        case CAN_RATE_800KBIT:
            t_config = TWAI_TIMING_CONFIG_800KBITS();
            break;
        case CAN_RATE_500KBIT:
            t_config = TWAI_TIMING_CONFIG_500KBITS();
            break;
        case CAN_RATE_250KBIT:
            t_config = TWAI_TIMING_CONFIG_250KBITS();
            break;
        case CAN_RATE_125KBIT:
            t_config = TWAI_TIMING_CONFIG_125KBITS();
            break;
        case CAN_RATE_100KBIT:
            t_config = TWAI_TIMING_CONFIG_100KBITS();
            break;
        default:
            t_config = TWAI_TIMING_CONFIG_1MBITS();
            break;
    }

    // 滤波器设置，接受所有地址的数据
    static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // 总线配置
    static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(gpio_num_t(TX_PIN), gpio_num_t(RX_PIN), TWAI_MODE_NO_ACK);

    // 传入驱动配置信息
    twai_driver_install(&g_config, &t_config, &f_config);

    // 启动CAN驱动
    auto status = twai_start();
    if (status != ESP_OK) {
        return status;
    }

    // 创建任务
    xTaskCreate(twai_feedback_update_task, "twai_fb", 4096, this, 5, &twai_fb_handle); // CAN反馈任务

    // 设置标志
    is_setup = true;
    
    return ESP_OK;
}

esp_err_t HXC_TWAI::send(HXC_CAN_message_t* message) {
    if(!is_setup){
        return ESP_FAIL;
    }
    twai_message_t twai_message;
    twai_message.extd = message->extd;
    twai_message.rtr = message->rtr;
    twai_message.self = message->self;
    twai_message.identifier = message->identifier;
    twai_message.data_length_code = message->data_length_code;
    //拷贝数据
    memcpy(twai_message.data, message->data, message->data_length_code);

    return twai_transmit(&twai_message, portMAX_DELAY);//发送数据并等待发送完成
}

esp_err_t HXC_TWAI::send(HXC_CAN_message_t message) {
    if(!is_setup){
        return ESP_FAIL;
    }
    twai_message_t twai_message;
    twai_message.extd = message.extd;//扩展帧标志
    twai_message.rtr = message.rtr;//远程帧标志
    twai_message.self = message.self;//自我接收请求
    twai_message.identifier = message.identifier;//CAN地址
    twai_message.data_length_code = message.data_length_code;//数据长度
    //拷贝数据
    memcpy(twai_message.data, message.data, message.data_length_code);

    return twai_transmit(&twai_message, portMAX_DELAY);//发送数据并等待发送完成
}

void HXC_TWAI::stop_receive() {
    if (twai_fb_handle == nullptr) {
        return;
    }
    vTaskDelete(twai_fb_handle);
    twai_fb_handle = nullptr;
}

void HXC_TWAI::resume_receive() {
    if (twai_fb_handle != nullptr) {
        return;
    }
    xTaskCreate(twai_feedback_update_task, "twai_fb", 4096, this, 5, &twai_fb_handle); // CAN反馈任务
}

bool HXC_TWAI::get_receive_status() {
    return twai_fb_handle != nullptr;
}

void HXC_TWAI::twai_feedback_update_task(void* n) {
    HXC_TWAI* twai = (HXC_TWAI*)n;
    twai_message_t Twai_message;

    auto To_HXC_CAN_message_t = [&](twai_message_t* twai_message) {
        twai->RX_message_buf.extd = twai_message->extd;//扩展帧标志
        twai->RX_message_buf.rtr = twai_message->rtr;//远程帧标志
        twai->RX_message_buf.self = twai_message->self;//自我接收标志
        twai->RX_message_buf.identifier = twai_message->identifier;//CAN地址
        twai->RX_message_buf.data_length_code = twai_message->data_length_code;//数据长度
        // 复制数据
        memcpy(twai->RX_message_buf.data, twai_message->data, twai_message->data_length_code);
    };

    while (1) {
        
        // 接收CAN数据
        twai_receive(&Twai_message, portMAX_DELAY);

        // 查看是否为需要的CAN消息地址，如果是就调用回调函数
        if (twai->exist_can_receive_callback_func(Twai_message.identifier)) {
            // 将twai_message转换成HXC_CAN_message
            To_HXC_CAN_message_t(&Twai_message);

            // 调用回调函数
            twai->func_map[twai->RX_message_buf.identifier](&twai->RX_message_buf);
        }
    }
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
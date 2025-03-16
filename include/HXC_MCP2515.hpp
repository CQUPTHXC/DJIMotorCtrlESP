#ifndef HXC_MCP2515_HPP
#define HXC_MCP2515_HPP
#include"HXC_CAN.hpp"
#include <SPI.h>
#include <Arduino.h>


// 使用延时模式接收CAN消息
// 延时模式:CPU占用低，但是总线数据在2000HZ以上时会丢包
// 非延时模式:CPU占用高，但是总线数据接收速度更高
#define USE_NONE_DELAY_RECEIVE 0

// MCP2515 寄存器枚举
enum MCP2515_REGISTER : uint8_t {
    MCP_RXF0SIDH = 0x00, // 接收过滤器0的标准标识符高字节
    MCP_RXF0SIDL = 0x01, // 接收过滤器0的标准标识符低字节
    MCP_RXF0EID8 = 0x02, // 接收过滤器0的扩展标识符高字节
    MCP_RXF0EID0 = 0x03, // 接收过滤器0的扩展标识符低字节
    MCP_RXF1SIDH = 0x04, // 接收过滤器1的标准标识符高字节
    MCP_RXF1SIDL = 0x05, // 接收过滤器1的标准标识符低字节
    MCP_RXF1EID8 = 0x06, // 接收过滤器1的扩展标识符高字节
    MCP_RXF1EID0 = 0x07, // 接收过滤器1的扩展标识符低字节
    MCP_RXF2SIDH = 0x08, // 接收过滤器2的标准标识符高字节
    MCP_RXF2SIDL = 0x09, // 接收过滤器2的标准标识符低字节
    MCP_RXF2EID8 = 0x0A, // 接收过滤器2的扩展标识符高字节
    MCP_RXF2EID0 = 0x0B, // 接收过滤器2的扩展标识符低字节
    MCP_CANSTAT  = 0x0E, // CAN状态寄存器
    MCP_CANCTRL  = 0x0F, // CAN控制寄存器
    MCP_RXF3SIDH = 0x10, // 接收过滤器3的标准标识符高字节
    MCP_RXF3SIDL = 0x11, // 接收过滤器3的标准标识符低字节
    MCP_RXF3EID8 = 0x12, // 接收过滤器3的扩展标识符高字节
    MCP_RXF3EID0 = 0x13, // 接收过滤器3的扩展标识符低字节
    MCP_RXF4SIDH = 0x14, // 接收过滤器4的标准标识符高字节
    MCP_RXF4SIDL = 0x15, // 接收过滤器4的标准标识符低字节
    MCP_RXF4EID8 = 0x16, // 接收过滤器4的扩展标识符高字节
    MCP_RXF4EID0 = 0x17, // 接收过滤器4的扩展标识符低字节
    MCP_RXF5SIDH = 0x18, // 接收过滤器5的标准标识符高字节
    MCP_RXF5SIDL = 0x19, // 接收过滤器5的标准标识符低字节
    MCP_RXF5EID8 = 0x1A, // 接收过滤器5的扩展标识符高字节
    MCP_RXF5EID0 = 0x1B, // 接收过滤器5的扩展标识符低字节
    MCP_TEC      = 0x1C, // 传输错误计数器
    MCP_REC      = 0x1D, // 接收错误计数器
    MCP_RXM0SIDH = 0x20, // 接收掩码0的标准标识符高字节
    MCP_RXM0SIDL = 0x21, // 接收掩码0的标准标识符低字节
    MCP_RXM0EID8 = 0x22, // 接收掩码0的扩展标识符高字节
    MCP_RXM0EID0 = 0x23, // 接收掩码0的扩展标识符低字节
    MCP_RXM1SIDH = 0x24, // 接收掩码1的标准标识符高字节
    MCP_RXM1SIDL = 0x25, // 接收掩码1的标准标识符低字节
    MCP_RXM1EID8 = 0x26, // 接收掩码1的扩展标识符高字节
    MCP_RXM1EID0 = 0x27, // 接收掩码1的扩展标识符低字节
    MCP_CNF3     = 0x28, // 配置寄存器3
    MCP_CNF2     = 0x29, // 配置寄存器2
    MCP_CNF1     = 0x2A, // 配置寄存器1
    MCP_CANINTE  = 0x2B, // CAN中断使能寄存器
    MCP_CANINTF  = 0x2C, // CAN中断标志寄存器
    MCP_EFLG     = 0x2D, // 错误标志寄存器
    MCP_TXB0CTRL = 0x30, // 传输缓冲区0控制寄存器
    MCP_TXB0SIDH = 0x31, // 传输缓冲区0的标准标识符高字节
    MCP_TXB0SIDL = 0x32, // 传输缓冲区0的标准标识符低字节
    MCP_TXB0EID8 = 0x33, // 传输缓冲区0的扩展标识符高字节
    MCP_TXB0EID0 = 0x34, // 传输缓冲区0的扩展标识符低字节
    MCP_TXB0DLC  = 0x35, // 传输缓冲区0的数据长度码
    MCP_TXB0DATA = 0x36, // 传输缓冲区0的数据，0x36后面7个地址分别为传输缓冲区1的数据
    MCP_TXB1CTRL = 0x40, // 传输缓冲区1控制寄存器
    MCP_TXB1SIDH = 0x41, // 传输缓冲区1的标准标识符高字节
    MCP_TXB1SIDL = 0x42, // 传输缓冲区1的标准标识符低字节
    MCP_TXB1EID8 = 0x43, // 传输缓冲区1的扩展标识符高字节
    MCP_TXB1EID0 = 0x44, // 传输缓冲区1的扩展标识符低字节
    MCP_TXB1DLC  = 0x45, // 传输缓冲区1的数据长度码
    MCP_TXB1DATA = 0x46, // 传输缓冲区1的数据，0x46后面7个地址分别为传输缓冲区2的数据
    MCP_TXB2CTRL = 0x50, // 传输缓冲区2控制寄存器
    MCP_TXB2SIDH = 0x51, // 传输缓冲区2的标准标识符高字节
    MCP_TXB2SIDL = 0x52, // 传输缓冲区2的标准标识符低字节
    MCP_TXB2EID8 = 0x53, // 传输缓冲区2的扩展标识符高字节
    MCP_TXB2EID0 = 0x54, // 传输缓冲区2的扩展标识符低字节
    MCP_TXB2DLC  = 0x55, // 传输缓冲区2的数据长度码
    MCP_TXB2DATA = 0x56, // 传输缓冲区2的数据，0x56后面7个地址分别为传输缓冲区3的数据
    MCP_RXB0CTRL = 0x60, // 接收缓冲区0控制寄存器
    MCP_RXB0SIDH = 0x61, // 接收缓冲区0的标准标识符高字节
    MCP_RXB0SIDL = 0x62, // 接收缓冲区0的标准标识符低字节
    MCP_RXB0EID8 = 0x63, // 接收缓冲区0的扩展标识符高字节
    MCP_RXB0EID0 = 0x64, // 接收缓冲区0的扩展标识符低字节
    MCP_RXB0DLC  = 0x65, // 接收缓冲区0的数据长度码
    MCP_RXB0DATA = 0x66, // 接收缓冲区0的数据,0x66后面7个地址分别为接收缓冲区1的数据
    MCP_RXB1CTRL = 0x70, // 接收缓冲区1控制寄存器
    MCP_RXB1SIDH = 0x71, // 接收缓冲区1的标准标识符高字节
    MCP_RXB1SIDL = 0x72, // 接收缓冲区1的标准标识符低字节
    MCP_RXB1EID8 = 0x73, // 接收缓冲区1的扩展标识符高字节
    MCP_RXB1EID0 = 0x74, // 接收缓冲区1的扩展标识符低字节
    MCP_RXB1DLC  = 0x75, // 接收缓冲区1的数据长度码
    MCP_RXB1DATA = 0x76  // 接收缓冲区1的数据，0x76后面7个地址分别为接收缓冲区2的数据
};

// MCP2515 模式枚举
enum MCP2515_MODE {
    NORMAL     = 0x00,
    SLEEP      = 0x20,
    LOOPBACK   = 0x40,
    LISTENONLY = 0x60,
    CONFIG     = 0x80
};
class HXC_MCP2515:public HXC_CAN{
public:

    /**
     * @brief 构造函数
     * @return {*}
     * @author: qingmeijiupiao
     * @param {SPIClass} *_spi SPI总线
     * @param {uint8_t} _cs_pin CS引脚
     * @param {uint8_t} _int_pin INT引脚，用于中断
     */
    HXC_MCP2515(SPIClass *_spi,uint8_t _cs_pin,uint8_t _int_pin);

    /**
     * @description:  初始化CAN总线
     * @author: qingmeijiupiao
     * @param {CAN_RATE} rate CAN总线速率枚举
     */
    esp_err_t setup(CAN_RATE rate) override;
    /**
     * @brief 发送CAN消息
     * @author qingmeijiupiao
     * @param {HXC_CAN_message_t} msg CAN消息结构体
     * @return {esp_err_t} ESP_OK:成功 ESP_FAIL:失败
     */
    esp_err_t send(HXC_CAN_message_t msg) override;
    
    /**
     * @brief  发送CAN消息
     * @author  qingmeijiupiao
     * @param {HXC_CAN_message_t*} msg CAN消息结构体指针
     * @return {esp_err_t} ESP_OK:成功 ESP_FAIL:失败
     */
    esp_err_t send(HXC_CAN_message_t *msg) override;
    

protected:


    /**
     * @brief 读取寄存器
     * @return {uint8_t} 读取到的数据
     * @author: qingmeijiupiao
     * @param {uint8_t} address 寄存器地址
     */    
    uint8_t readRegister(uint8_t address);
    /**
     * @brief : 设置寄存器
     * @return  {*}
     * @Author : qingmeijiupiao
     * @param {uint8_t} address 寄存器地址
     * @param {uint8_t} data 数据
     */
    void setRegister(uint8_t address,uint8_t data);

    /**
     * @brief :  修改寄存器
     * @return  {*}
     * @Author : qingmeijiupiao
     * @param {uint8_t} address 寄存器地址
     * @param {uint8_t} mask 掩码,需要修改的位为1，不需要修改的位为0
     * @param {uint8_t} data 数据
     * 
     */
    void modifyRegister(uint8_t address,uint8_t mask, uint8_t data);

    /**
     * @brief   从接收缓冲区读取CAN消息
     * @return  {*}
     * @Author : qingmeijiupiao
     * @param {uint8_t} buffer_num 接收缓冲区编号 0-1
     * @param {HXC_CAN_message_t} *msg CAN消息结构体指针
     */
    void read_can_message(uint8_t buffer_num, HXC_CAN_message_t *msg);

    /**
    * @brief : 获取中断标志
    * @return  {uint8_t} 中断标志 
    * @author : qingmeijiupiao
    */
    uint8_t getInterrupts();
    /**
     * @brief :  清除中断标志
     * @return  {*}
     * @Author : qingmeijiupiao
     */
    void clearInterrupts();

    /**
     * @brief :  SPI总线加锁
     * @return  {*}
     * @Author : qingmeijiupiao
     */
    void lock_spi();

    /**
     * @brief :  SPI总线解锁
     * @return  {*}
     * @Author : qingmeijiupiao
     */
    void unlock_spi();
    
    /**
     * @brief :  将CAN消息发送到TX缓冲区
     * @return  {esp_err_t} ESP_OK:成功 ESP_FAIL:失败
     * @Author : qingmeijiupiao
     * @param {uint8_t} TX_num TX缓冲区编号 0-2
     * @param {HXC_CAN_message_t*} msg CAN消息结构体指针
     */

    esp_err_t send_msg_to_tx_buffer(uint8_t TX_num, HXC_CAN_message_t* msg);

    /**
     * @brief : 接收中断处理函数
     * @return  {*}
     * @Author : qingmeijiupiao
     */
    void interrupt();
    SPIClass *spi;//SPI总线
    uint8_t cs_pin;//CS引脚
    uint8_t int_pin;//中断引脚
    HXC_CAN_message_t rx_buffer[2];//接收缓冲对象

    //中断包装函数
    static void Interrupt_repakage_func(void *arg){
        HXC_MCP2515 *can=(HXC_MCP2515*)arg;
        while (1){
            if(!digitalRead(can->int_pin)){
                can->interrupt();
            }else{
                
                #if USE_NONE_DELAY_RECEIVE == 1
                //主动让出CPU使用权方式,接收频率高时会导致CPU占用率过高
                vPortYield();
                #else
                //延时方式接收,总线上超过2000HZ会丢包
                vTaskDelay(1);
                #endif
            }
        }
        
    }

    //spi总线信号量
    std::map<SPIClass*,SemaphoreHandle_t> spi_semaphore;
};


HXC_MCP2515::HXC_MCP2515(SPIClass *_spi,uint8_t _cs_pin,uint8_t _int_pin){
    spi=_spi;
    cs_pin=_cs_pin;
    int_pin=_int_pin;
    
}

//spi互斥锁上锁
void HXC_MCP2515::lock_spi(){
    xSemaphoreTake(spi_semaphore[spi],portMAX_DELAY);
}

//解锁spi总线互斥锁
void HXC_MCP2515::unlock_spi(){
    xSemaphoreGive(spi_semaphore[spi]);
}

esp_err_t HXC_MCP2515::setup(CAN_RATE rate){
    pinMode(cs_pin,OUTPUT);
    digitalWrite(cs_pin,HIGH);
    pinMode(int_pin,INPUT);

    if(spi_semaphore.find(spi)==spi_semaphore.end()){
        spi_semaphore[spi]=xSemaphoreCreateMutex();//创建信号量
        unlock_spi();
    }
    //加锁
    lock_spi();

    // 配置模式
    MCP2515_MODE mode=CONFIG;
    modifyRegister(MCP_CANCTRL,0xE0,mode);
    // 等待配置完成
    for (int i = 0; i < 10; i++) {
        uint8_t newmode = readRegister(MCP_CANSTAT);
        newmode &= 0xE0;
        if (newmode == mode) {
            break;
        }
        vTaskDelay(10 / portTICK_RATE_MS);
    }
    // 启用接收缓冲区
    setRegister(MCP_RXB0CTRL, 0x64);  // 启用接收缓冲区 0
    
    setRegister(MCP_RXB1CTRL, 0x60);  // 启用接收缓冲区 1

    // 启用接收中断
    modifyRegister(MCP_CANINTE, 0x03, 0x03);  // 使能接收缓冲区 0,1 中断


    uint8_t cfg[3]={0x00,0x00,0x00};
    // 配置速率
    switch (this->can_rate) {
        case CAN_RATE_1MBIT:
            cfg[0]=0x00;
            cfg[1]=0x80;
            cfg[2]=0x80;
            break;
        case CAN_RATE_800KBIT:
            break;
        case CAN_RATE_500KBIT:
            cfg[0]=0x00;
            cfg[1]=0x90;
            cfg[2]=0x82;
            break;
        case CAN_RATE_250KBIT:
            cfg[0]=0x00;
            cfg[1]=0xB1;
            cfg[2]=0x85;
            break;
        case CAN_RATE_125KBIT:
            cfg[0]=0x01;
            cfg[1]=0xB1;
            cfg[2]=0x85;
            break;
        case CAN_RATE_100KBIT:
            cfg[0]=0x01;
            cfg[1]=0xB4;
            cfg[2]=0x86;
            break;
        default:
            cfg[0]=0x00;
            cfg[1]=0x80;
            cfg[2]=0x80;
            break;
    }
    setRegister(MCP_CNF1, cfg[0]);
    setRegister(MCP_CNF2, cfg[1]);
    setRegister(MCP_CNF3, cfg[2]);

    
    mode=NORMAL;
    // 切换回普通模式
    modifyRegister(MCP_CANCTRL,0xE0,mode);
    
    // 解锁
    unlock_spi();

    // 注意这里优先级必须是最低否则在非延时接收时会喂不了看门狗导致重启
    xTaskCreate(Interrupt_repakage_func,"CAN_INT",1024*2,this,0,NULL);//创建can接收任务
    is_setup=true;//设置已初始化标志位
    return ESP_OK;
}


uint8_t HXC_MCP2515::readRegister(uint8_t address){
    uint8_t data;
    digitalWrite(cs_pin,LOW);
    spi->transfer(0x3);
    spi->transfer(address);
    data=spi->transfer(0x0);
    digitalWrite(cs_pin,HIGH);
    return data;
}

void HXC_MCP2515::setRegister(uint8_t address,uint8_t data){
    digitalWrite(cs_pin,LOW);
    spi->transfer(0x2);
    spi->transfer(address);
    spi->transfer(data);
    digitalWrite(cs_pin,HIGH);
}

void HXC_MCP2515::modifyRegister(uint8_t address,uint8_t mask, uint8_t data){
    uint8_t _data[4]={0x00,0x00,0x00,0x00};
    _data[0] = 0x5;
    _data[1] = address;
    _data[2] = mask;
    _data[3] = data;
    digitalWrite(cs_pin,LOW);
    spi->transfer(_data,4);
    digitalWrite(cs_pin,HIGH);
}

esp_err_t HXC_MCP2515::send(HXC_CAN_message_t* msg){

    lock_spi();

    uint8_t tx_ctrl_flag=0;
    tx_ctrl_flag = readRegister(MCP_TXB0CTRL);
    if((tx_ctrl_flag&0x08)==0){
        send_msg_to_tx_buffer(0,msg);
        unlock_spi();
        return ESP_OK;
    }
    tx_ctrl_flag = readRegister(MCP_TXB1CTRL);
    if((tx_ctrl_flag&0x08)==0){
        send_msg_to_tx_buffer(1,msg);
        unlock_spi();
        return ESP_OK;
    }
    tx_ctrl_flag = readRegister(MCP_TXB2CTRL);
    if((tx_ctrl_flag&0x08)==0){
        send_msg_to_tx_buffer(2,msg);
        unlock_spi();
        return ESP_OK;
    }
    unlock_spi();
    return ESP_FAIL;
    
    
}
esp_err_t HXC_MCP2515::send(HXC_CAN_message_t msg){
    return send(&msg);
}

esp_err_t HXC_MCP2515::send_msg_to_tx_buffer(uint8_t TX_num, HXC_CAN_message_t* msg) {
    
    uint8_t TXBnCTRL=0x30+(TX_num<<8);
    uint8_t TXRTSCTRL=0x0D;
    uint8_t TXBnSIDH=0x31+(TX_num<<8);
    uint8_t TXBnSIDL=0x32+(TX_num<<8);
    uint8_t TXBnEID8=0x33+(TX_num<<8);
    uint8_t TXBnEID0=0x34+(TX_num<<8);
    uint8_t TXBnDLC=0x35+(TX_num<<8);
    uint8_t TXBnD0=0x36+(TX_num<<8);
    uint8_t temp=0;
    
    

    if(!msg->extd){//标准帧
        temp=(msg->identifier>>3)&0xFF;
        setRegister(TXBnSIDH,temp);
        temp=(msg->identifier<<5)&0xFF;
        setRegister(TXBnSIDL,temp);
    }else{//扩展帧
        temp=(msg->identifier)&0xFF;
        setRegister(TXBnEID0,temp);
        temp=(msg->identifier>>8)&0xFF;
        setRegister(TXBnEID8,temp);
        temp=(msg->identifier>>16)&0x3;
        temp|=(msg->identifier>>(13))&0xE0;
        temp|=0x8;//设置为扩展帧
        setRegister(TXBnSIDL,temp);
        temp=(msg->identifier>>21)&0xFF;
        setRegister(TXBnSIDH,temp);
    }
    temp=msg->data_length_code&0x0F;//数据长度
    temp|=msg->rtr?0x40:0x00;//远程帧
    setRegister(TXBnDLC,temp);
    for(uint8_t i=0;i<msg->data_length_code;i++){
        setRegister(TXBnD0+i,msg->data[i]);//数据
    }

    // 发送消息,第三位控制发送
    modifyRegister(TXBnCTRL, (1<<3), (1<<3));

    

    return ESP_OK;
}


uint8_t HXC_MCP2515::getInterrupts(){
    /* 
    * @bit7 MERRF：消息错误中断标志位
    * 1 = 中断待处理（必须由MCU清除以重置中断条件）
    * 0 = 无中断待处理
    * 
    * @bit6 WAKIF：唤醒中断标志位
    * 1 = 中断待处理（必须由MCU清除以重置中断条件）
    * 0 = 无中断待处理
    * 
    * @bit5 ERRIF：错误中断标志位（EFLG寄存器中多个源）
    * 1 = 中断待处理（必须由MCU清除以重置中断条件）
    * 0 = 无中断待处理
    * 
    * @bit4 TX2IF：发送缓冲区2空中断标志位
    * 1 = 中断待处理（必须由MCU清除以重置中断条件）
    * 0 = 无中断待处理
    * 
    * @bit3 TX1IF：发送缓冲区1空中断标志位
    * 1 = 中断待处理（必须由MCU清除以重置中断条件）
    * 0 = 无中断待处理
    * 
    * @bit2 TX0IF：发送缓冲区0空中断标志位
    * 1 = 中断待处理（必须由MCU清除以重置中断条件）
    * 0 = 无中断待处理
    * 
    * @bit1 RX1IF：接收缓冲区1满中断标志位
    * 1 = 中断待处理（必须由MCU清除以重置中断条件）
    * 0 = 无中断待处理
    * 
    * @bit0 RX0IF：接收缓冲区0满中断标志位
    * 1 = 中断待处理（必须由MCU清除以重置中断条件）
    * 0 = 无中断待处理
    */
    return readRegister(MCP_CANINTF);
}

void HXC_MCP2515::clearInterrupts(){
    setRegister(MCP_CANINTF, 0);
}

void HXC_MCP2515::read_can_message(uint8_t buffer_num, HXC_CAN_message_t *msg){
    auto clear_rxbuffer=[](HXC_CAN_message_t * buffer){
        buffer->extd=false;
        buffer->identifier=0;
        buffer->rtr=false;
        buffer->data_length_code=0;
        memset(buffer->data,0,8);
    };
    if (buffer_num == 0) {  // 如果接收缓冲区 0 中断被触发
        clear_rxbuffer(msg);
        uint8_t Register_buffer[]={0x00,0x00};
        Register_buffer[0]=readRegister(MCP_RXB0SIDH);
        Register_buffer[1]=readRegister(MCP_RXB0SIDL);
        msg->extd=Register_buffer[1]&0x8;
        if(msg->extd){
            msg->identifier=readRegister(MCP_RXB0EID0);
            msg->identifier|=readRegister(MCP_RXB0EID8)<<8;
            msg->identifier|=((Register_buffer[1])&0x3)<<16;
            msg->identifier|=((Register_buffer[1]&0xE0)>>5)<<18;
            msg->identifier|=Register_buffer[0]<<21;
        }else{
            msg->identifier=Register_buffer[0]<<3;
            msg->identifier|=(Register_buffer[1]&0xE0)>>5;
        }
        uint8_t dlc=readRegister(MCP_RXB0DLC);
        msg->rtr=dlc&0x20;
        msg->data_length_code=dlc&0x0F;
        for(int i=0;i<msg->data_length_code;i++){
            msg->data[i]=readRegister(MCP_RXB0DATA+i);
        }
        return;
    }
    if (buffer_num == 1) {
        clear_rxbuffer(&this->rx_buffer[1]);
        uint8_t Register_buffer[]={0x00,0x00};
        Register_buffer[0]=readRegister(MCP_RXB1SIDH);
        Register_buffer[1]=readRegister(MCP_RXB1SIDL);
        msg->extd=Register_buffer[1]&0x8;
        if(msg->extd){
            msg->identifier=readRegister(MCP_RXB1EID0);
            msg->identifier|=readRegister(MCP_RXB1EID8)<<8;
            msg->identifier|=((Register_buffer[1])&0x3)<<17;
            msg->identifier|=((Register_buffer[1]&0xE0)>>5)<<19;
            msg->identifier|=Register_buffer[0]<<22;
        }else{
            msg->identifier=Register_buffer[0]<<3;
            msg->identifier|=(Register_buffer[1]&0xE0)>>5;
        }
        uint8_t dlc=readRegister(MCP_RXB1DLC);
        msg->rtr=dlc&0x20;
        msg->data_length_code=dlc&0x0F;
        for(int i=0;i<msg->data_length_code;i++){
            msg->data[i]=readRegister(MCP_RXB1DATA+i);
        }
        return;
    }
}


void HXC_MCP2515::interrupt(){
    lock_spi();

    uint8_t interruptFlags = getInterrupts();  // 检查中断标志
    if (interruptFlags & 0x01) {  // 如果接收缓冲区 0 中断被触发
        // 读取接收数据
        read_can_message(0,&this->rx_buffer[0]);

        // 
        if(func_map.find(this->rx_buffer[0].identifier)!=func_map.end()){
            func_map[this->rx_buffer[0].identifier](&this->rx_buffer[0]);
        }
    }
    if (interruptFlags & 0x02) {  // 如果接收缓冲区 1 中断被触发
        // 读取接收数据
        read_can_message(1,&this->rx_buffer[1]);

        //
        if(func_map.find(this->rx_buffer[1].identifier)!=func_map.end()){
            func_map[this->rx_buffer[1].identifier](&this->rx_buffer[1]);
        }
    }
    
    // 清除中断标志s
   clearInterrupts();

   unlock_spi();
}
#endif
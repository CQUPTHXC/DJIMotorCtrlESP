#include "HXC_CAN.hpp"

/*↓↓↓↓↓函数定义↓↓↓↓↓*/

// 默认构造函数
HXC_CAN::HXC_CAN() : is_setup(false), can_rate(CAN_RATE_1MBIT) {}

// 析构函数
HXC_CAN::~HXC_CAN() {}

// 初始化函数
hxc_err_t HXC_CAN::setup(CAN_RATE can_rate) {
    return HXC_FAIL; //不应该调用基类的setup函数
}

// 发送CAN消息函数，接收消息指针作为参数
hxc_err_t HXC_CAN::send(HXC_CAN_message_t* message) {
    return HXC_FAIL; //不应该调用基类的send函数
}

// 发送CAN消息函数，接收CAN消息对象作为参数
hxc_err_t HXC_CAN::send(HXC_CAN_message_t message) {
    return HXC_FAIL; //不应该调用基类的send函数
}

// 添加CAN消息接收回调函数
void HXC_CAN::add_can_receive_callback_func(int addr, HXC_can_feedback_func func) {
    func_map[addr] = func;  // 将回调函数存入映射表
}

// 移除CAN消息接收回调函数
void HXC_CAN::remove_can_receive_callback_func(int addr) {
    if (!exist_can_receive_callback_func(addr)) {
        return;  // 如果回调函数不存在，则返回
    }
    func_map.erase(addr);  // 从映射表中删除回调函数
}

// 判断CAN消息接收回调函数是否存在
bool HXC_CAN::exist_can_receive_callback_func(int addr) {
    return func_map.find(addr) != func_map.end();  // 如果地址存在于映射表中，则返回true
}

bool HXC_CAN::get_setup_flag(){
    return is_setup;
};

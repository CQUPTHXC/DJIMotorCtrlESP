/*
 * @version: 2.0
 * @LastEditors: qingmeijiupiao
 * @Description: HXC CAN基类文件,需要用到CAN时应该使用此类作为传入类型，方便扩展不同的CAN方式
 * @Author: qingmeijiupiao
 * @LastEditTime: 2024-12-20 16:04:02
 */

#ifndef HXC_CAN_BASE_HPP
#define HXC_CAN_BASE_HPP
#include <map>

/*↓↓↓本文件的声明↓↓↓*/

//CAN基类,需要用到CAN时应该使用此类作为传入类型，方便扩展不同的CAN方式
class HXC_CAN;

//CAN消息结构体
struct HXC_CAN_message_t;

//CAN消息接收回调函数
using HXC_can_feedback_func= std::function<void(HXC_CAN_message_t* can_message)>;

//CAN速率枚举
enum CAN_RATE{
  CAN_RATE_1MBIT,
  CAN_RATE_800KBIT,
  CAN_RATE_500KBIT,
  CAN_RATE_250KBIT,
  CAN_RATE_125KBIT,
  CAN_RATE_100KBIT
};


/*↑↑↑本文件的声明↑↑↑*/

//CAN消息结构体
struct HXC_CAN_message_t{
  bool extd=0;           /**< 扩展帧格式标志（29位ID） */
  bool rtr=0;            /**< 远程帧标志 */
  bool self=0;           /**< 自我接收请求。接收时无效。 */
  uint32_t identifier;                /**< 11或29位标识符 */
  uint8_t data_length_code;           /**< 数据长度代码 */
  uint8_t data[8];    /**< 数据字节（在RTR帧中无关） */
};

//CAN基类
class HXC_CAN {
public:
    // 防止值传递CAN对象
    HXC_CAN(const HXC_CAN&) = delete;               /**< 删除拷贝构造函数 */
    HXC_CAN& operator=(const HXC_CAN&) = delete;    /**< 删除拷贝赋值函数 */
    HXC_CAN(HXC_CAN&&) = delete;                    /**< 删除移动构造函数 */
    HXC_CAN& operator=(HXC_CAN&&) = delete;         /**< 删除移动赋值函数 */

    HXC_CAN();  /**< 默认构造函数 */
    virtual ~HXC_CAN();  /**< 析构函数 */

    /**
     * @description: 初始化
     * @return {esp_err_t} 成功返回ESP_OK
     * @Author: qingmeijiupiao
     * @param {CAN_RATE} can_rate CAN速率
     */
    virtual esp_err_t setup(CAN_RATE can_rate);

    /**
     * @description: 发送CAN消息
     * @return {esp_err_t} 成功返回ESP_OK
     * @Author: qingmeijiupiao
     * @param {HXC_CAN_message_t*} message CAN消息指针
     */
    virtual esp_err_t send(HXC_CAN_message_t* message);

    /**
     * @description: 发送CAN消息
     * @return {esp_err_t} 成功返回ESP_OK
     * @Author: qingmeijiupiao
     * @param {HXC_CAN_message_t} message CAN消息
     */
    virtual esp_err_t send(HXC_CAN_message_t message);

    /**
     * @description: 添加CAN消息接收回调,收到对应地址的消息时运行回调函数
     * @return {*}
     * @Author: qingmeijiupiao
     * @param {int} addr CAN消息地址
     * @param {HXC_can_feedback_func} func 回调函数
     */
    void add_can_receive_callback_func(int addr, HXC_can_feedback_func func);

    /**
     * @description: 移除CAN消息接收回调函数
     * @return {*}
     * @Author: qingmeijiupiao
     * @param {int} addr CAN消息地址
     */
    void remove_can_receive_callback_func(int addr);

    /**
     * @description: 判断CAN消息接收回调函数是否存在
     * @return {bool} 如果回调函数存在则返回true，否则返回false
     * @Author: qingmeijiupiao
     * @param {int} addr CAN消息地址
     */
    bool exist_can_receive_callback_func(int addr);


    bool get_setup_flag();
protected:
    std::map<int, HXC_can_feedback_func> func_map;  /**< CAN回调函数映射表 */
    bool is_setup = false;  /**< CAN初始化状态标志 */
    CAN_RATE can_rate;  /**< 当前CAN速率 */
};



/*↓↓↓↓↓函数定义↓↓↓↓↓*/



// 默认构造函数
HXC_CAN::HXC_CAN() : is_setup(false), can_rate(CAN_RATE_1MBIT) {}

// 析构函数
HXC_CAN::~HXC_CAN() {}

// 初始化函数
esp_err_t HXC_CAN::setup(CAN_RATE can_rate) {
    return ESP_FAIL; //不应该调用基类的setup函数
}

// 发送CAN消息函数，接收消息指针作为参数
esp_err_t HXC_CAN::send(HXC_CAN_message_t* message) {
    return ESP_FAIL; //不应该调用基类的send函数
}

// 发送CAN消息函数，接收CAN消息对象作为参数
esp_err_t HXC_CAN::send(HXC_CAN_message_t message) {
    return ESP_FAIL; //不应该调用基类的send函数
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
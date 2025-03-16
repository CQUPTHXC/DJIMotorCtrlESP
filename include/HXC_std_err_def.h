/*
 * @version: v1.0.0
 * @LastEditors: qingmeijiupiao
 * @Description: HXC标准错误头文件
 * @author: qingmeijiupiao
 * @LastEditTime: 2025-03-16 17:18:18
 */
#ifndef HXC_STD_DEF_HPP_
#define HXC_STD_DEF_HPP_

//HXC标准错误类型
typedef int hxc_err_t;

//HXC标准错误定义

//正常
#define HXC_OK                      0

//错误
#define HXC_FAIL                    -1

//超时
#define HXC_ERR_TIMEOUT             0x107

#endif /* HXC_STD_DEF_HPP_ */

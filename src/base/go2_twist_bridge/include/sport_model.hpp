#ifndef _SPORT_MODEL_
#define _SPORT_MODEL_

#include <iostream>
#pragma pack(push, 1)

// 机器人运动API ID常量定义
const int32_t ROBOT_SPORT_API_ID_DAMP                 = 1001;  // 阻尼
const int32_t ROBOT_SPORT_API_ID_BALANCESTAND         = 1002;  // 平衡站立
const int32_t ROBOT_SPORT_API_ID_STOPMOVE             = 1003;  // 停止移动
const int32_t ROBOT_SPORT_API_ID_STANDUP              = 1004;  // 站起
const int32_t ROBOT_SPORT_API_ID_STANDDOWN            = 1005;  // 蹲下
const int32_t ROBOT_SPORT_API_ID_RECOVERYSTAND        = 1006;  // 恢复站立
const int32_t ROBOT_SPORT_API_ID_EULER                = 1007;  // 欧拉角控制
const int32_t ROBOT_SPORT_API_ID_MOVE                 = 1008;  // 移动
const int32_t ROBOT_SPORT_API_ID_SIT                  = 1009;  // 坐下
const int32_t ROBOT_SPORT_API_ID_RISESIT              = 1010;  // 站起坐下切换
const int32_t ROBOT_SPORT_API_ID_SWITCHGAIT           = 1011;  // 切换步态
const int32_t ROBOT_SPORT_API_ID_TRIGGER              = 1012;  // 触发
const int32_t ROBOT_SPORT_API_ID_BODYHEIGHT           = 1013;  // 身体高度
const int32_t ROBOT_SPORT_API_ID_FOOTRAISEHEIGHT      = 1014;  // 抬脚高度
const int32_t ROBOT_SPORT_API_ID_SPEEDLEVEL           = 1015;  // 速度等级
const int32_t ROBOT_SPORT_API_ID_HELLO                = 1016;  // 打招呼
const int32_t ROBOT_SPORT_API_ID_STRETCH              = 1017;  // 伸展
const int32_t ROBOT_SPORT_API_ID_TRAJECTORYFOLLOW     = 1018;  // 轨迹跟随
const int32_t ROBOT_SPORT_API_ID_CONTINUOUSGAIT       = 1019;  // 连续步态
const int32_t ROBOT_SPORT_API_ID_CONTENT              = 1020;  // 内容
const int32_t ROBOT_SPORT_API_ID_WALLOW               = 1021;  // 打滚
const int32_t ROBOT_SPORT_API_ID_DANCE1               = 1022;  // 舞蹈1
const int32_t ROBOT_SPORT_API_ID_DANCE2               = 1023;  // 舞蹈2
const int32_t ROBOT_SPORT_API_ID_GETBODYHEIGHT        = 1024;  // 获取身体高度
const int32_t ROBOT_SPORT_API_ID_GETFOOTRAISEHEIGHT   = 1025;  // 获取抬脚高度
const int32_t ROBOT_SPORT_API_ID_GETSPEEDLEVEL        = 1026;  // 获取速度等级
const int32_t ROBOT_SPORT_API_ID_SWITCHJOYSTICK       = 1027;  // 切换操纵杆模式
const int32_t ROBOT_SPORT_API_ID_POSE                 = 1028;  // 姿势
const int32_t ROBOT_SPORT_API_ID_SCRAPE               = 1029;  // 刮擦
const int32_t ROBOT_SPORT_API_ID_FRONTFLIP            = 1030;  // 前空翻
const int32_t ROBOT_SPORT_API_ID_FRONTJUMP            = 1031;  // 前跳
const int32_t ROBOT_SPORT_API_ID_FRONTPOUNCE          = 1032;  // 前扑

#pragma pack(pop)

#endif

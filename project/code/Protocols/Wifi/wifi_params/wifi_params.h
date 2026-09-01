/*****************************************************************************
 * 文件: wifi_params.h
 * 模块: WiFi 命令适配
 * 职责: 保留飞机端 WiFi 命令路由，车端只处理 start/stop 与 imu 相关命令
 *****************************************************************************/

#ifndef WIFI_PARAMS_H
#define WIFI_PARAMS_H

#include "zf_common_headfile.h"

/* WiFi 命令最近一次处理结果码 */
typedef enum
{
    WIFI_PARAMS_RESULT_OK = 0,             /* 执行成功 */
    WIFI_PARAMS_RESULT_ERR_FORMAT = 1,     /* 命令格式错误 */
    WIFI_PARAMS_RESULT_ERR_UNKNOWN_CMD = 2,/* 未知命令 */
    WIFI_PARAMS_RESULT_ERR_UNKNOWN_PARAM = 3,/* 未知参数 */
    WIFI_PARAMS_RESULT_ERR_RANGE = 4,      /* 参数超范围 */
    WIFI_PARAMS_RESULT_ERR_STATE = 5,      /* 当前状态不允许 */
    WIFI_PARAMS_RESULT_ERR_FLASH = 6       /* Flash 操作失败 */
} wifi_params_result_e;

/* WiFi 命令最近一次处理命令码 */
typedef enum
{
    WIFI_PARAMS_COMMAND_NONE = 0,          /* 无命令 */
    WIFI_PARAMS_COMMAND_PING = 1,          /* ping */
    WIFI_PARAMS_COMMAND_HELP = 2,          /* help */
    WIFI_PARAMS_COMMAND_GET = 3,           /* get */
    WIFI_PARAMS_COMMAND_SET = 4,           /* set */
    WIFI_PARAMS_COMMAND_SAVE = 5,          /* save */
    WIFI_PARAMS_COMMAND_LOAD = 6,          /* load */
    WIFI_PARAMS_COMMAND_LIST = 7,          /* list */
    WIFI_PARAMS_COMMAND_START = 8,         /* start */
    WIFI_PARAMS_COMMAND_STOP = 9           /* stop */
} wifi_params_command_e;

/* WiFi 命令诊断结构体：用于查看最近一次命令处理结果 */
typedef struct
{
    uint8_t last_command_code;  /* 最近一次命令码 */
    uint8_t last_result_code;   /* 最近一次结果码 */
    uint16_t last_param_index;  /* 兼容飞机端诊断字段，车端固定为 0 */
    float last_value;           /* 兼容飞机端诊断字段 */
} wifi_params_diag_t;

/*
 * 函数名: wifi_params_Init
 * 功能: 初始化 WiFi 命令适配模块内部状态
 * 输入参数: 无
 * 返回值: 无
 */
void wifi_params_Init(void);

/*
 * 函数名: wifi_params_ProcessLine
 * 功能: 处理一条完整文本命令
 * 输入参数:
 *   line - 完整文本命令，函数内部允许原地切分
 * 返回值: 无
 */
void wifi_params_ProcessLine(char *line);

/*
 * 函数名: wifi_params_GetDiag
 * 功能: 获取最近一次命令处理诊断信息
 * 输入参数:
 *   diag - 输出诊断结构体指针
 * 返回值: 无
 */
void wifi_params_GetDiag(wifi_params_diag_t *diag);

#endif /* WIFI_PARAMS_H */

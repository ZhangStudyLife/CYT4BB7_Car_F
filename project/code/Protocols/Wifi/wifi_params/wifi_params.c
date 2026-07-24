/*****************************************************************************
 * File: wifi_params.c
 * Module: WiFi command adapter
 * Purpose: keep the Air-side WiFi command route on the car without exposing
 *          car parameter editing through this module.
 *****************************************************************************/

#include "wifi_params.h"

#include <string.h>

#include "Protocols/wifi/wifi_cmd/wifi_cmd.h"
#include "Protocols/wifi/wifi_justfloat/wifi_justfloat.h"

static wifi_params_diag_t s_wifi_params_diag = {0};

static void wifi_params_set_diag(uint8_t command_code,
                                 uint8_t result_code,
                                 uint16_t param_index,
                                 float value)
{
    s_wifi_params_diag.last_command_code = command_code;
    s_wifi_params_diag.last_result_code = result_code;
    s_wifi_params_diag.last_param_index = param_index;
    s_wifi_params_diag.last_value = value;
}

static uint8_t wifi_params_is_edit_allowed(void)
{
    return 1U;
}

static void wifi_params_reply_error(uint8_t command_code,
                                    uint8_t result_code,
                                    const char *reason)
{
    wifi_params_set_diag(command_code, result_code, 0U, 0.0f);
    (void)wifi_cmd_SendLine("ERR %s", (NULL != reason) ? reason : "format");
}

static void wifi_params_process_ping(void)
{
    wifi_params_set_diag(WIFI_PARAMS_COMMAND_PING, WIFI_PARAMS_RESULT_OK, 0U, 0.0f);
    (void)wifi_cmd_SendLine("OK ping");
}

static void wifi_params_process_help(const char *topic)
{
    wifi_params_set_diag(WIFI_PARAMS_COMMAND_HELP, WIFI_PARAMS_RESULT_OK, 0U, 0.0f);

    if ((NULL != topic) && ('\0' != topic[0]) && (0U == wifi_cmd_is_help_flag(topic)))
    {
        if ((0 == wifi_cmd_ascii_stricmp(topic, "ping")) ||
            (0 == wifi_cmd_ascii_stricmp(topic, "help")) ||
            (0 == wifi_cmd_ascii_stricmp(topic, "imu")) ||
            (0 == wifi_cmd_ascii_stricmp(topic, "start")) ||
            (0 == wifi_cmd_ascii_stricmp(topic, "stop")))
        {
            (void)wifi_cmd_SendLine("OK help %s", topic);
            return;
        }

        wifi_params_reply_error(WIFI_PARAMS_COMMAND_HELP,
                                WIFI_PARAMS_RESULT_ERR_UNKNOWN_CMD,
                                "unknown command");
        return;
    }

    (void)wifi_cmd_SendLine("OK help commands=ping,help,start,stop,imu");
}

static void wifi_params_process_unsupported(uint8_t command_code)
{
    wifi_params_set_diag(command_code, WIFI_PARAMS_RESULT_ERR_UNKNOWN_PARAM, 0U, 0.0f);
    (void)wifi_cmd_SendLine("ERR unsupported");
}

static void wifi_params_process_start(void)
{
    if (0U == wifi_params_is_edit_allowed())
    {
        wifi_params_reply_error(WIFI_PARAMS_COMMAND_START, WIFI_PARAMS_RESULT_ERR_STATE, "state");
        return;
    }

    wifi_justfloat_SetStandbyUserEnable(1U);
    wifi_params_set_diag(WIFI_PARAMS_COMMAND_START, WIFI_PARAMS_RESULT_OK, 0U, 1.0f);
    (void)wifi_cmd_SendLine("OK start telemetry=on");
}

static void wifi_params_process_stop(void)
{
    if (0U == wifi_params_is_edit_allowed())
    {
        wifi_params_reply_error(WIFI_PARAMS_COMMAND_STOP, WIFI_PARAMS_RESULT_ERR_STATE, "state");
        return;
    }

    wifi_justfloat_SetStandbyUserEnable(0U);
    wifi_params_set_diag(WIFI_PARAMS_COMMAND_STOP, WIFI_PARAMS_RESULT_OK, 0U, 0.0f);
    (void)wifi_cmd_SendLine("OK stop telemetry=off");
}

void wifi_params_Init(void)
{
    memset(&s_wifi_params_diag, 0, sizeof(s_wifi_params_diag));
}

void wifi_params_ProcessLine(char *line)
{
    char *trimmed_line;
    char *cursor;
    char *token_cmd;
    char *token_arg1;
    char *token_arg2;
    char *token_arg3;

    if (NULL == line)
    {
        wifi_params_reply_error(WIFI_PARAMS_COMMAND_NONE, WIFI_PARAMS_RESULT_ERR_FORMAT, "format");
        return;
    }

    trimmed_line = wifi_cmd_trim_line(line);
    if ((NULL == trimmed_line) || ('\0' == trimmed_line[0]))
    {
        return;
    }

    cursor = trimmed_line;
    token_cmd = wifi_cmd_next_token(&cursor);
    token_arg1 = wifi_cmd_next_token(&cursor);
    token_arg2 = wifi_cmd_next_token(&cursor);
    token_arg3 = wifi_cmd_next_token(&cursor);

    if ((NULL == token_cmd) || (NULL != token_arg3))
    {
        wifi_params_reply_error(WIFI_PARAMS_COMMAND_NONE, WIFI_PARAMS_RESULT_ERR_FORMAT, "format");
        return;
    }

    wifi_cmd_ascii_strtolower(token_cmd);
    if (NULL != token_arg1)
    {
        wifi_cmd_ascii_strtolower(token_arg1);
    }

    if (0 == strcmp(token_cmd, "ping"))
    {
        if (NULL == token_arg1)
        {
            wifi_params_process_ping();
            return;
        }

        if ((0U != wifi_cmd_is_help_flag(token_arg1)) && (NULL == token_arg2))
        {
            wifi_params_process_help("ping");
            return;
        }
    }
    else if (0 == strcmp(token_cmd, "help"))
    {
        if (NULL == token_arg2)
        {
            wifi_params_process_help(token_arg1);
            return;
        }
    }
    else if (0 == strcmp(token_cmd, "start"))
    {
        if (NULL == token_arg1)
        {
            wifi_params_process_start();
            return;
        }

        if ((0U != wifi_cmd_is_help_flag(token_arg1)) && (NULL == token_arg2))
        {
            wifi_params_process_help("start");
            return;
        }
    }
    else if (0 == strcmp(token_cmd, "stop"))
    {
        if (NULL == token_arg1)
        {
            wifi_params_process_stop();
            return;
        }

        if ((0U != wifi_cmd_is_help_flag(token_arg1)) && (NULL == token_arg2))
        {
            wifi_params_process_help("stop");
            return;
        }
    }
    else if (0 == strcmp(token_cmd, "get"))
    {
        if ((NULL != token_arg1) && (NULL == token_arg2))
        {
            wifi_params_process_unsupported(WIFI_PARAMS_COMMAND_GET);
            return;
        }
    }
    else if (0 == strcmp(token_cmd, "set"))
    {
        if ((NULL != token_arg1) && (NULL != token_arg2))
        {
            wifi_params_process_unsupported(WIFI_PARAMS_COMMAND_SET);
            return;
        }
    }
    else if (0 == strcmp(token_cmd, "save"))
    {
        if (NULL == token_arg1)
        {
            wifi_params_process_unsupported(WIFI_PARAMS_COMMAND_SAVE);
            return;
        }
    }
    else if (0 == strcmp(token_cmd, "load"))
    {
        if (NULL == token_arg1)
        {
            wifi_params_process_unsupported(WIFI_PARAMS_COMMAND_LOAD);
            return;
        }
    }
    else if (0 == strcmp(token_cmd, "list"))
    {
        if (NULL == token_arg1)
        {
            wifi_params_process_unsupported(WIFI_PARAMS_COMMAND_LIST);
            return;
        }
    }

    if ((NULL != token_arg1) && (0U != wifi_cmd_is_help_flag(token_arg1)) && (NULL == token_arg2))
    {
        wifi_params_process_help(token_cmd);
        return;
    }

    if ((0 == strcmp(token_cmd, "get")) || (0 == strcmp(token_cmd, "set")) ||
        (0 == strcmp(token_cmd, "save")) || (0 == strcmp(token_cmd, "load")) ||
        (0 == strcmp(token_cmd, "start")) || (0 == strcmp(token_cmd, "stop")) ||
        (0 == strcmp(token_cmd, "list")) || (0 == strcmp(token_cmd, "ping")) ||
        (0 == strcmp(token_cmd, "help")))
    {
        wifi_params_reply_error(WIFI_PARAMS_COMMAND_NONE, WIFI_PARAMS_RESULT_ERR_FORMAT, "format");
        return;
    }

    wifi_params_reply_error(WIFI_PARAMS_COMMAND_NONE,
                            WIFI_PARAMS_RESULT_ERR_UNKNOWN_CMD,
                            "unknown command");
}

void wifi_params_GetDiag(wifi_params_diag_t *diag)
{
    if (NULL == diag)
    {
        return;
    }

    *diag = s_wifi_params_diag;
}

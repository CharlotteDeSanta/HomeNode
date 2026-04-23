#include "app_node.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "bsp_delay.h"
#include "bsp_motor.h"
#include "bsp_relay.h"
#include "tim.h"
#include "usart.h"

#define APP_BUTTON_COUNT         4U
#define APP_BUTTON_DEBOUNCE_MS   30U
#define APP_POST_RELAY_TEST_MS   100U
#define APP_POST_MOTOR_TEST_MS   120U
#define APP_POST_ESP_TIMEOUT_MS  1200U
#define APP_POST_DHT11_BOOT_MS   1800U
#define APP_POST_DHT11_RETRY_MS  1200U
#define APP_POST_DHT11_RETRY_MAX 3U
#define APP_DHT11_POLL_MS        2500U
#define APP_WIFI_RETRY_MS        10000U
#define APP_UPLOAD_INTERVAL_MS   5000U
#define APP_UPLOAD_RETRY_MS      1000U
#define APP_UPLOAD_FAIL_REJOIN_THRESHOLD 6U
#define APP_ESP_RESPONSE_SIZE    256U
#define APP_ESP_DRAIN_MS         120U
#define APP_ESP_CMD_TIMEOUT_MS   1500U
#define APP_ESP_JOIN_TIMEOUT_MS  7000U
#define APP_ESP_TCP_TIMEOUT_MS   2500U
#define APP_ESP_PROMPT_WAIT_MS   600U
#define APP_DEBUG_TX_TIMEOUT_MS  120U

/*
 * Fill these values locally before Wi-Fi debug:
 * - SSID/PASSWORD for AP join
 * - HOST/PORT for telemetry upload endpoint
 */
#define APP_WIFI_SSID     "Leon_AP"
#define APP_WIFI_PASSWORD "1887415157Cz"
#define APP_UPLOAD_HOST   "10.181.249.25"
#define APP_UPLOAD_PORT   7891U

typedef struct
{
    uint16_t pin;
    uint8_t stableState;
    uint8_t lastSample;
    uint32_t lastChangeTick;
} APP_ButtonState;

static APP_ButtonState s_buttons[APP_BUTTON_COUNT] =
{
    {GPIO_PIN_0, 0U, 0U, 0U},
    {GPIO_PIN_1, 0U, 0U, 0U},
    {GPIO_PIN_2, 0U, 0U, 0U},
    {GPIO_PIN_3, 0U, 0U, 0U}
};

static uint8_t s_relayState[3] = {0U, 0U, 0U};
static uint8_t s_fanEnabled = 0U;
static uint8_t s_fanDutyPercent = 60U;
static uint8_t s_dht11Valid = 0U;
static uint8_t s_wifiConfigured = 0U;
static uint8_t s_uploadConfigured = 0U;
static uint8_t s_wifiConnected = 0U;
static uint8_t s_uploadFailStreak = 0U;
static uint8_t s_lastUploadOk = 0U;
static uint16_t s_uploadOkCount = 0U;
static uint16_t s_uploadFailCount = 0U;
static uint32_t s_lastDhtPollTick = 0U;
static uint32_t s_lastWifiAttemptTick = 0U;
static uint32_t s_nextUploadAttemptTick = 0U;
static UART_HandleTypeDef* s_debugUart = 0;

static uint8_t app_is_wifi_configured(void)
{
    return ((APP_WIFI_SSID[0] != '\0') && (APP_WIFI_PASSWORD[0] != '\0')) ? 1U : 0U;
}

static uint8_t app_is_upload_configured(void)
{
    return (APP_UPLOAD_HOST[0] != '\0') ? 1U : 0U;
}

static void app_debug_log(const char* format, ...)
{
    char buffer[192];
    int length;
    va_list args;

    if ((s_debugUart == 0) || (format == 0))
    {
        return;
    }

    va_start(args, format);
    length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (length <= 0)
    {
        return;
    }

    if (length >= (int)sizeof(buffer))
    {
        length = (int)sizeof(buffer) - 1;
    }

    if ((length < 2) || (buffer[length - 2] != '\r') || (buffer[length - 1] != '\n'))
    {
        if ((length + 2) < (int)sizeof(buffer))
        {
            buffer[length] = '\r';
            length++;
            buffer[length] = '\n';
            length++;
            buffer[length] = '\0';
        }
    }

    (void)HAL_UART_Transmit(s_debugUart, (uint8_t*)buffer, (uint16_t)length, APP_DEBUG_TX_TIMEOUT_MS);
}

static void app_debug_log_response(const char* prefix, const char* response)
{
    char line[128];
    uint16_t srcIndex = 0U;
    uint16_t dstIndex = 0U;

    if (response == 0)
    {
        app_debug_log("%s(null)", (prefix != 0) ? prefix : "");
        return;
    }

    while ((response[srcIndex] != '\0') && (dstIndex < (sizeof(line) - 1U)))
    {
        char ch = response[srcIndex];
        srcIndex++;

        if ((ch == '\r') || (ch == '\n') || (ch == '\t'))
        {
            if ((dstIndex > 0U) && (line[dstIndex - 1U] == ' '))
            {
                continue;
            }

            ch = ' ';
        }

        line[dstIndex] = ch;
        dstIndex++;
    }

    line[dstIndex] = '\0';
    app_debug_log("%s%s", (prefix != 0) ? prefix : "", line);
}

static uint8_t app_read_button(uint16_t pin)
{
    return (HAL_GPIO_ReadPin(GPIOA, pin) == GPIO_PIN_SET) ? 1U : 0U;
}

static uint8_t app_button_pressed_event(APP_ButtonState* button, uint32_t nowTick)
{
    uint8_t sample = app_read_button(button->pin);

    if (sample != button->lastSample)
    {
        button->lastSample = sample;
        button->lastChangeTick = nowTick;
    }

    if ((nowTick - button->lastChangeTick) >= APP_BUTTON_DEBOUNCE_MS)
    {
        if (button->stableState != sample)
        {
            uint8_t previous = button->stableState;
            button->stableState = sample;

            if ((previous == 0U) && (sample == 1U))
            {
                return 1U;
            }
        }
    }

    return 0U;
}

static void app_apply_relay_state(uint8_t relayIndex)
{
    (void)BSP_Relay_Set((BSP_RelayChannel)relayIndex, s_relayState[relayIndex]);
}

static void app_apply_fan_state(void)
{
    uint16_t pulse;

    if (s_fanEnabled == 0U)
    {
        (void)BSP_Motor_Set(BSP_MOTOR_CHANNEL_A, BSP_MOTOR_STOP, 0U);
        BSP_Motor_SetStandby(0U);
        return;
    }

    BSP_Motor_SetStandby(1U);
    pulse = (uint16_t)(((__HAL_TIM_GET_AUTORELOAD(&htim3) + 1U) * s_fanDutyPercent) / 100U);
    if (pulse == 0U)
    {
        pulse = 1U;
    }

    (void)BSP_Motor_Set(BSP_MOTOR_CHANNEL_A, BSP_MOTOR_FORWARD, pulse);
}

static uint8_t app_esp_has_terminal(const char* response)
{
    if (response == 0)
    {
        return 0U;
    }

    if ((strstr(response, "\r\nOK\r\n") != 0) || (strstr(response, "OK\r\n") != 0))
    {
        return 1U;
    }

    if (strstr(response, "ERROR") != 0)
    {
        return 1U;
    }

    if (strstr(response, "FAIL") != 0)
    {
        return 1U;
    }

    if (strstr(response, "WIFI GOT IP") != 0)
    {
        return 1U;
    }

    if (strstr(response, "SEND OK") != 0)
    {
        return 1U;
    }

    if (strstr(response, "ALREADY CONNECTED") != 0)
    {
        return 1U;
    }

    if (strstr(response, ">") != 0)
    {
        return 1U;
    }

    return 0U;
}

static HAL_StatusTypeDef app_esp_read_response(BSP_ESP01S_HandleTypeDef* esp,
                                               char* response,
                                               uint16_t responseSize,
                                               uint32_t timeoutMs)
{
    uint8_t rxByte;
    uint16_t length = 0U;
    uint32_t startTick;
    HAL_StatusTypeDef status;

    if ((esp == 0) || (response == 0) || (responseSize < 2U))
    {
        return HAL_ERROR;
    }

    response[0] = '\0';
    startTick = HAL_GetTick();

    while (((HAL_GetTick() - startTick) < timeoutMs) && (length < (responseSize - 1U)))
    {
        status = BSP_ESP01S_Receive(esp, &rxByte, 1U, 20U);
        if (status != HAL_OK)
        {
            continue;
        }

        response[length] = (char)rxByte;
        length++;
        response[length] = '\0';

        if (app_esp_has_terminal(response) != 0U)
        {
            return HAL_OK;
        }
    }

    return (length > 0U) ? HAL_OK : HAL_TIMEOUT;
}

static void app_post_mark(APP_NodeContext* node, APP_PostItem item, HAL_StatusTypeDef status)
{
    if (node == 0)
    {
        return;
    }

    if (status == HAL_OK)
    {
        node->post.passMask |= (uint32_t)item;
    }
    else
    {
        node->post.failMask |= (uint32_t)item;
    }
}

static HAL_StatusTypeDef app_post_delay_test(void)
{
    uint16_t elapsedUs;

    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    BSP_Delay_Us(200U);
    elapsedUs = BSP_Delay_GetCounter();

    return (elapsedUs >= 120U) ? HAL_OK : HAL_ERROR;
}

static HAL_StatusTypeDef app_post_relay_test(void)
{
    uint32_t relayIndex;
    HAL_StatusTypeDef status;

    for (relayIndex = 0U; relayIndex < 3U; relayIndex++)
    {
        status = BSP_Relay_Set((BSP_RelayChannel)relayIndex, 1U);
        if (status != HAL_OK)
        {
            return status;
        }

        HAL_Delay(APP_POST_RELAY_TEST_MS);

        status = BSP_Relay_Set((BSP_RelayChannel)relayIndex, 0U);
        if (status != HAL_OK)
        {
            return status;
        }

        HAL_Delay(20U);
    }

    return HAL_OK;
}

static HAL_StatusTypeDef app_post_motor_test(void)
{
    uint16_t pulse;
    HAL_StatusTypeDef status;

    pulse = (uint16_t)((__HAL_TIM_GET_AUTORELOAD(&htim3) + 1U) / 4U);
    if (pulse == 0U)
    {
        pulse = 1U;
    }

    BSP_Motor_SetStandby(1U);

    status = BSP_Motor_Set(BSP_MOTOR_CHANNEL_A, BSP_MOTOR_FORWARD, pulse);
    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(APP_POST_MOTOR_TEST_MS);
    status = BSP_Motor_Set(BSP_MOTOR_CHANNEL_A, BSP_MOTOR_STOP, 0U);
    if (status != HAL_OK)
    {
        return status;
    }

    status = BSP_Motor_Set(BSP_MOTOR_CHANNEL_B, BSP_MOTOR_FORWARD, pulse);
    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(APP_POST_MOTOR_TEST_MS);
    status = BSP_Motor_Set(BSP_MOTOR_CHANNEL_B, BSP_MOTOR_STOP, 0U);
    if (status != HAL_OK)
    {
        return status;
    }

    BSP_Motor_SetStandby(0U);
    HAL_Delay(20U);
    BSP_Motor_SetStandby(1U);
    return HAL_OK;
}

static HAL_StatusTypeDef app_post_dht11_test(APP_NodeContext* node)
{
    HAL_StatusTypeDef status = HAL_ERROR;
    uint8_t attempt;

    if (node == 0)
    {
        return HAL_ERROR;
    }

    /* DHT11 requires a stabilization window after power-up. */
    HAL_Delay(APP_POST_DHT11_BOOT_MS);

    for (attempt = 0U; attempt < APP_POST_DHT11_RETRY_MAX; attempt++)
    {
        status = BSP_DHT11_Read(&node->dht11);
        if (status == HAL_OK)
        {
            return HAL_OK;
        }

        if ((attempt + 1U) < APP_POST_DHT11_RETRY_MAX)
        {
            HAL_Delay(APP_POST_DHT11_RETRY_MS);
        }
    }

    return status;
}

static void app_esp_rx_drain(BSP_ESP01S_HandleTypeDef* esp, uint32_t drainMs)
{
    uint32_t startTick = HAL_GetTick();
    uint8_t dummyByte;

    if (esp == 0)
    {
        return;
    }

    while ((HAL_GetTick() - startTick) < drainMs)
    {
        (void)BSP_ESP01S_Receive(esp, &dummyByte, 1U, 20U);
    }
}

static HAL_StatusTypeDef app_esp_send_command(APP_NodeContext* node,
                                              const char* command,
                                              const char* expect1,
                                              const char* expect2,
                                              uint32_t timeoutMs,
                                              char* response,
                                              uint16_t responseSize)
{
    char extra[96];
    uint16_t responseLength;
    uint16_t extraLength;
    uint16_t copyLength;
    uint32_t waitStart;
    HAL_StatusTypeDef extraStatus;
    HAL_StatusTypeDef status;

    if ((node == 0) || (command == 0) || (response == 0) || (responseSize < 2U))
    {
        return HAL_ERROR;
    }

    app_debug_log("[ESP CMD] %s", command);
    app_esp_rx_drain(&node->esp, APP_ESP_DRAIN_MS);
    status = BSP_ESP01S_SendString(&node->esp, command, APP_ESP_CMD_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        app_debug_log("[ESP CMD] tx failed, status=%d", (int)status);
        return status;
    }

    status = app_esp_read_response(&node->esp, response, responseSize, timeoutMs);
    if (status != HAL_OK)
    {
        app_debug_log("[ESP CMD] rx timeout, status=%d", (int)status);
        return status;
    }

    app_debug_log_response("[ESP RSP] ", response);

    if ((expect1 != 0) && (strstr(response, expect1) != 0))
    {
        return HAL_OK;
    }

    if ((expect2 != 0) && (strstr(response, expect2) != 0))
    {
        return HAL_OK;
    }

    if ((strstr(response, "ERROR") != 0) || (strstr(response, "FAIL") != 0))
    {
        app_debug_log("[ESP CMD] command failed");
        return HAL_ERROR;
    }

    if ((expect1 != 0) && (strcmp(expect1, ">") == 0))
    {
        waitStart = HAL_GetTick();
        responseLength = (uint16_t)strlen(response);

        while ((HAL_GetTick() - waitStart) < APP_ESP_PROMPT_WAIT_MS)
        {
            extraStatus = app_esp_read_response(&node->esp,
                                                extra,
                                                (uint16_t)sizeof(extra),
                                                120U);
            if (extraStatus != HAL_OK)
            {
                continue;
            }

            app_debug_log_response("[ESP RSP+] ", extra);
            extraLength = (uint16_t)strlen(extra);

            if ((responseLength < (responseSize - 1U)) && (extraLength > 0U))
            {
                copyLength = extraLength;
                if ((responseLength + copyLength) >= responseSize)
                {
                    copyLength = (uint16_t)((responseSize - 1U) - responseLength);
                }

                if (copyLength > 0U)
                {
                    memcpy(&response[responseLength], extra, copyLength);
                    responseLength = (uint16_t)(responseLength + copyLength);
                    response[responseLength] = '\0';
                }
            }

            if (strstr(response, ">") != 0)
            {
                return HAL_OK;
            }

            if ((strstr(extra, "ERROR") != 0) || (strstr(extra, "FAIL") != 0))
            {
                app_debug_log("[ESP CMD] prompt wait failed");
                return HAL_ERROR;
            }
        }
    }

    app_debug_log("[ESP CMD] unexpected response");
    return HAL_ERROR;
}

static HAL_StatusTypeDef app_esp_connect_wifi(APP_NodeContext* node)
{
    char response[APP_ESP_RESPONSE_SIZE];
    char command[160];
    HAL_StatusTypeDef status;

    if ((node == 0) || (s_wifiConfigured == 0U))
    {
        return HAL_ERROR;
    }

    app_debug_log("[NET] wifi connect begin");
    BSP_ESP01S_SetEnable(&node->esp, 1U);
    BSP_ESP01S_Reset(&node->esp);

    status = app_esp_send_command(node,
                                  "AT\r\n",
                                  "OK",
                                  0,
                                  APP_ESP_CMD_TIMEOUT_MS,
                                  response,
                                  (uint16_t)sizeof(response));
    if (status != HAL_OK)
    {
        app_debug_log("[NET] wifi connect failed: AT");
        return status;
    }

    (void)app_esp_send_command(node,
                               "ATE0\r\n",
                               "OK",
                               0,
                               APP_ESP_CMD_TIMEOUT_MS,
                               response,
                               (uint16_t)sizeof(response));

    status = app_esp_send_command(node,
                                  "AT+CWMODE=1\r\n",
                                  "OK",
                                  0,
                                  APP_ESP_CMD_TIMEOUT_MS,
                                  response,
                                  (uint16_t)sizeof(response));
    if (status != HAL_OK)
    {
        app_debug_log("[NET] wifi connect failed: CWMODE");
        return status;
    }

    status = app_esp_send_command(node,
                                  "AT+CIPMUX=0\r\n",
                                  "OK",
                                  0,
                                  APP_ESP_CMD_TIMEOUT_MS,
                                  response,
                                  (uint16_t)sizeof(response));
    if (status != HAL_OK)
    {
        app_debug_log("[NET] wifi connect failed: CIPMUX");
        return status;
    }

    (void)snprintf(command,
                   sizeof(command),
                   "AT+CWJAP=\"%s\",\"%s\"\r\n",
                   APP_WIFI_SSID,
                   APP_WIFI_PASSWORD);
    status = app_esp_send_command(node,
                                  command,
                                  "WIFI GOT IP",
                                  "OK",
                                  APP_ESP_JOIN_TIMEOUT_MS,
                                  response,
                                  (uint16_t)sizeof(response));
    if (status != HAL_OK)
    {
        if (strstr(response, "WIFI CONNECTED") != 0)
        {
            app_debug_log("[NET] wifi connected (late got ip)");
            return HAL_OK;
        }

        app_debug_log("[NET] wifi connect failed: CWJAP");
        return HAL_ERROR;
    }

    app_debug_log("[NET] wifi connected");
    return HAL_OK;
}

static HAL_StatusTypeDef app_esp_upload_telemetry(APP_NodeContext* node)
{
    char response[APP_ESP_RESPONSE_SIZE];
    char command[96];
    char tcpPayload[160];
    uint8_t sendOk = 0U;
    int payloadLength;
    HAL_StatusTypeDef status;

    if ((node == 0) || (s_uploadConfigured == 0U))
    {
        return HAL_ERROR;
    }

    payloadLength = snprintf(tcpPayload,
                             sizeof(tcpPayload),
                             "T=%u.%u,H=%u.%u,DHTOK=%u,FAN=%u,DUTY=%u,RLY1=%u,RLY2=%u,RLY3=%u\r\n",
                             (unsigned int)node->dht11.temperatureInt,
                             (unsigned int)node->dht11.temperatureDec,
                             (unsigned int)node->dht11.humidityInt,
                             (unsigned int)node->dht11.humidityDec,
                             (unsigned int)s_dht11Valid,
                             (unsigned int)s_fanEnabled,
                             (unsigned int)s_fanDutyPercent,
                             (unsigned int)s_relayState[0],
                             (unsigned int)s_relayState[1],
                             (unsigned int)s_relayState[2]);
    if ((payloadLength <= 0) || ((uint32_t)payloadLength >= sizeof(tcpPayload)))
    {
        app_debug_log("[NET] payload build failed");
        return HAL_ERROR;
    }

    app_debug_log("[NET] upload payload: %s", tcpPayload);

    (void)snprintf(command,
                   sizeof(command),
                   "AT+CIPSTART=\"TCP\",\"%s\",%lu\r\n",
                   APP_UPLOAD_HOST,
                   (unsigned long)APP_UPLOAD_PORT);
    status = app_esp_send_command(node,
                                  command,
                                  "OK",
                                  "ALREADY CONNECTED",
                                  APP_ESP_TCP_TIMEOUT_MS,
                                  response,
                                  (uint16_t)sizeof(response));
    if ((status != HAL_OK) && (strstr(response, "CONNECT") == 0))
    {
        app_debug_log("[NET] CIPSTART failed");
        return HAL_ERROR;
    }

    (void)snprintf(command, sizeof(command), "AT+CIPSEND=%lu\r\n", (unsigned long)payloadLength);
    status = app_esp_send_command(node,
                                  command,
                                  ">",
                                  0,
                                  APP_ESP_CMD_TIMEOUT_MS,
                                  response,
                                  (uint16_t)sizeof(response));
    if (status != HAL_OK)
    {
        app_debug_log("[NET] CIPSEND prompt failed");
        (void)app_esp_send_command(node,
                                   "AT+CIPCLOSE\r\n",
                                   "OK",
                                   "CLOSED",
                                   APP_ESP_CMD_TIMEOUT_MS,
                                   response,
                                   (uint16_t)sizeof(response));
        return status;
    }

    status = BSP_ESP01S_Send(&node->esp, (const uint8_t*)tcpPayload, (uint16_t)payloadLength, 3000U);
    if (status != HAL_OK)
    {
        app_debug_log("[NET] payload tx failed");
        (void)app_esp_send_command(node,
                                   "AT+CIPCLOSE\r\n",
                                   "OK",
                                   "CLOSED",
                                   APP_ESP_CMD_TIMEOUT_MS,
                                   response,
                                   (uint16_t)sizeof(response));
        return status;
    }

    status = app_esp_read_response(&node->esp,
                                   response,
                                   (uint16_t)sizeof(response),
                                   APP_ESP_TCP_TIMEOUT_MS);
    if (status == HAL_OK)
    {
        app_debug_log_response("[NET] send rsp: ", response);
        if (strstr(response, "SEND OK") != 0)
        {
            sendOk = 1U;
        }
        else if ((strstr(response, "Recv ") != 0) &&
                 (strstr(response, " bytes") != 0) &&
                 (strstr(response, "ERROR") == 0) &&
                 (strstr(response, "FAIL") == 0))
        {
            sendOk = 1U;
        }
    }

    if (sendOk == 0U)
    {
        app_debug_log("[NET] send result not ok");
        (void)app_esp_send_command(node,
                                   "AT+CIPCLOSE\r\n",
                                   "OK",
                                   "CLOSED",
                                   APP_ESP_CMD_TIMEOUT_MS,
                                   response,
                                   (uint16_t)sizeof(response));
        return HAL_ERROR;
    }

    /* Keep short-connection behavior deterministic for different server types. */
    (void)app_esp_send_command(node,
                               "AT+CIPCLOSE\r\n",
                               "OK",
                               "CLOSED",
                               APP_ESP_CMD_TIMEOUT_MS,
                               response,
                               (uint16_t)sizeof(response));

    app_debug_log("[NET] upload ok");
    return HAL_OK;
}

static void app_poll_dht11(APP_NodeContext* node, uint32_t nowTick)
{
    if (node == 0)
    {
        return;
    }

    if ((nowTick - s_lastDhtPollTick) < APP_DHT11_POLL_MS)
    {
        return;
    }

    s_lastDhtPollTick = nowTick;
    if (BSP_DHT11_Read(&node->dht11) == HAL_OK)
    {
        s_dht11Valid = 1U;
    }
    else
    {
        s_dht11Valid = 0U;
    }

    node->dht11Valid = s_dht11Valid;
}

static void app_service_esp(APP_NodeContext* node, uint32_t nowTick)
{
    if (node != 0)
    {
        node->wifiConnected = s_wifiConnected;
        node->uploadFailStreak = s_uploadFailStreak;
        node->lastUploadOk = s_lastUploadOk;
        node->uploadOkCount = s_uploadOkCount;
        node->uploadFailCount = s_uploadFailCount;
    }

    if ((node == 0) || (s_wifiConfigured == 0U))
    {
        if (node != 0)
        {
            node->wifiConnected = 0U;
        }
        return;
    }

    if (s_wifiConnected == 0U)
    {
        if ((nowTick - s_lastWifiAttemptTick) < APP_WIFI_RETRY_MS)
        {
            return;
        }

        app_debug_log("[NET] wifi disconnected, retry join");
        s_lastWifiAttemptTick = nowTick;
        if (app_esp_connect_wifi(node) == HAL_OK)
        {
            s_wifiConnected = 1U;
            s_uploadFailStreak = 0U;
            node->wifiConnected = 1U;
            s_nextUploadAttemptTick = HAL_GetTick() + APP_UPLOAD_INTERVAL_MS;
            app_debug_log("[NET] wait one upload interval after connect");
        }
        else
        {
            app_debug_log("[NET] wifi join retry failed");
        }
        return;
    }

    if (s_uploadConfigured == 0U)
    {
        return;
    }

    if ((int32_t)(nowTick - s_nextUploadAttemptTick) < 0)
    {
        return;
    }

    if (app_esp_upload_telemetry(node) == HAL_OK)
    {
        s_lastUploadOk = 1U;
        s_uploadFailStreak = 0U;
        node->lastTelemetryTick = nowTick;
        s_nextUploadAttemptTick = nowTick + APP_UPLOAD_INTERVAL_MS;
        if (s_uploadOkCount < 0xFFFFU)
        {
            s_uploadOkCount++;
        }
        node->uploadFailStreak = s_uploadFailStreak;
        node->lastUploadOk = s_lastUploadOk;
        node->uploadOkCount = s_uploadOkCount;
        node->uploadFailCount = s_uploadFailCount;
        return;
    }

    s_lastUploadOk = 0U;
    if (s_uploadFailCount < 0xFFFFU)
    {
        s_uploadFailCount++;
    }
    if (s_uploadFailStreak < 0xFFU)
    {
        s_uploadFailStreak++;
    }
    s_nextUploadAttemptTick = nowTick + APP_UPLOAD_RETRY_MS;

    if (s_uploadFailStreak >= APP_UPLOAD_FAIL_REJOIN_THRESHOLD)
    {
        app_debug_log("[NET] upload fail streak reached %u, force rejoin",
                      (unsigned int)APP_UPLOAD_FAIL_REJOIN_THRESHOLD);
        s_wifiConnected = 0U;
        node->wifiConnected = 0U;
        s_uploadFailStreak = 0U;
        s_nextUploadAttemptTick = nowTick + APP_UPLOAD_INTERVAL_MS;
    }

    node->uploadFailStreak = s_uploadFailStreak;
    node->lastUploadOk = s_lastUploadOk;
    node->uploadOkCount = s_uploadOkCount;
    node->uploadFailCount = s_uploadFailCount;
}

static HAL_StatusTypeDef app_post_esp01s_test(APP_NodeContext* node)
{
    uint8_t rxByte;
    char rxBuffer[96];
    uint16_t rxLength = 0U;
    uint32_t startTick;
    HAL_StatusTypeDef status;

    if (node == 0)
    {
        return HAL_ERROR;
    }

    memset(rxBuffer, 0, sizeof(rxBuffer));

    BSP_ESP01S_SetEnable(&node->esp, 1U);
    BSP_ESP01S_Reset(&node->esp);
    app_esp_rx_drain(&node->esp, 300U);

    status = BSP_ESP01S_SendString(&node->esp, "AT\r\n", 200U);
    if (status != HAL_OK)
    {
        return status;
    }

    startTick = HAL_GetTick();
    while (((HAL_GetTick() - startTick) < APP_POST_ESP_TIMEOUT_MS) &&
           (rxLength < (sizeof(rxBuffer) - 1U)))
    {
        status = BSP_ESP01S_Receive(&node->esp, &rxByte, 1U, 50U);
        if (status != HAL_OK)
        {
            continue;
        }

        rxBuffer[rxLength] = (char)rxByte;
        rxLength++;
        rxBuffer[rxLength] = '\0';

        if (strstr(rxBuffer, "OK") != 0)
        {
            return HAL_OK;
        }

        if (strstr(rxBuffer, "ERROR") != 0)
        {
            return HAL_ERROR;
        }
    }

    return HAL_TIMEOUT;
}

void APP_Node_RunPowerOnSelfTest(APP_NodeContext* node)
{
    if (node == 0)
    {
        return;
    }

    node->post.passMask = 0U;
    node->post.failMask = 0U;
    node->post.done = 0U;

    app_post_mark(node, APP_POST_ITEM_DELAY, app_post_delay_test());
    app_post_mark(node, APP_POST_ITEM_RELAY, app_post_relay_test());
    app_post_mark(node, APP_POST_ITEM_MOTOR, app_post_motor_test());
    app_post_mark(node, APP_POST_ITEM_DHT11, app_post_dht11_test(node));
    app_post_mark(node, APP_POST_ITEM_ESP01S, app_post_esp01s_test(node));

    BSP_Relay_SetAll(0U);
    memset(s_relayState, 0, sizeof(s_relayState));
    s_fanEnabled = 0U;
    app_apply_fan_state();
    node->post.done = 1U;
}

uint8_t APP_Node_IsPostPassed(const APP_NodeContext* node)
{
    const uint32_t requiredMask = (uint32_t)APP_POST_ITEM_DELAY |
                                  (uint32_t)APP_POST_ITEM_RELAY |
                                  (uint32_t)APP_POST_ITEM_MOTOR |
                                  (uint32_t)APP_POST_ITEM_DHT11 |
                                  (uint32_t)APP_POST_ITEM_ESP01S;

    if (node == 0)
    {
        return 0U;
    }

    if (node->post.done == 0U)
    {
        return 0U;
    }

    if ((node->post.failMask & requiredMask) != 0U)
    {
        return 0U;
    }

    return ((node->post.passMask & requiredMask) == requiredMask) ? 1U : 0U;
}

const APP_PostResult* APP_Node_GetPostResult(const APP_NodeContext* node)
{
    if (node == 0)
    {
        return 0;
    }

    return &node->post;
}

void APP_Node_SetDebugUart(UART_HandleTypeDef* huart)
{
    s_debugUart = huart;
}

HAL_StatusTypeDef APP_Node_Init(APP_NodeContext* node)
{
    HAL_StatusTypeDef status;
    uint32_t nowTick;
    uint32_t index;

    if (node == 0)
    {
        return HAL_ERROR;
    }

    memset(node, 0, sizeof(*node));

    status = BSP_Delay_Init(&htim2);
    if (status != HAL_OK)
    {
        return status;
    }

    BSP_Relay_Init();
    memset(s_relayState, 0, sizeof(s_relayState));
    s_fanEnabled = 0U;
    s_fanDutyPercent = 60U;
    s_dht11Valid = 0U;
    s_wifiConfigured = app_is_wifi_configured();
    s_uploadConfigured = app_is_upload_configured();
    s_wifiConnected = 0U;
    s_uploadFailStreak = 0U;
    s_lastUploadOk = 0U;
    s_uploadOkCount = 0U;
    s_uploadFailCount = 0U;
    node->dht11Valid = 0U;
    node->wifiConnected = 0U;
    node->uploadEnabled = s_uploadConfigured;
    node->fanDutyPercent = s_fanDutyPercent;
    node->uploadFailStreak = 0U;
    node->lastUploadOk = 0U;
    node->uploadOkCount = 0U;
    node->uploadFailCount = 0U;

    status = BSP_Motor_Init(&htim3);
    if (status != HAL_OK)
    {
        return status;
    }

    status = BSP_DHT11_Init(&node->dht11, GPIOA, GPIO_PIN_6);
    if (status != HAL_OK)
    {
        return status;
    }

    status = BSP_ESP01S_Init(&node->esp, &huart3, GPIOB, GPIO_PIN_8, GPIOB, GPIO_PIN_9);
    if (status != HAL_OK)
    {
        return status;
    }

    app_apply_fan_state();

    nowTick = HAL_GetTick();
    for (index = 0U; index < APP_BUTTON_COUNT; index++)
    {
        s_buttons[index].stableState = app_read_button(s_buttons[index].pin);
        s_buttons[index].lastSample = s_buttons[index].stableState;
        s_buttons[index].lastChangeTick = nowTick;
    }

    s_lastDhtPollTick = nowTick - APP_DHT11_POLL_MS;
    s_lastWifiAttemptTick = nowTick - APP_WIFI_RETRY_MS;
    node->lastTelemetryTick = nowTick - APP_UPLOAD_INTERVAL_MS;
    s_nextUploadAttemptTick = nowTick;
    APP_Node_RunPowerOnSelfTest(node);
    return HAL_OK;
}

void APP_Node_Process(APP_NodeContext* node)
{
    uint32_t nowTick;

    if (node == 0)
    {
        return;
    }

    nowTick = HAL_GetTick();

    if (app_button_pressed_event(&s_buttons[0], nowTick) != 0U)
    {
        s_relayState[0] ^= 1U;
        app_apply_relay_state(0U);
    }

    if (app_button_pressed_event(&s_buttons[1], nowTick) != 0U)
    {
        s_relayState[1] ^= 1U;
        app_apply_relay_state(1U);
    }

    if (app_button_pressed_event(&s_buttons[2], nowTick) != 0U)
    {
        s_relayState[2] ^= 1U;
        app_apply_relay_state(2U);
    }

    if (app_button_pressed_event(&s_buttons[3], nowTick) != 0U)
    {
        s_fanEnabled ^= 1U;
        app_apply_fan_state();
    }

    node->fanDutyPercent = s_fanDutyPercent;
    app_poll_dht11(node, nowTick);
    app_service_esp(node, nowTick);
}

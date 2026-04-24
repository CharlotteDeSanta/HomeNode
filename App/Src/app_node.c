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
#define APP_ESP_CMD_TIMEOUT_MS   1500U
#define APP_ESP_JOIN_TIMEOUT_MS  12000U
#define APP_ESP_TCP_TIMEOUT_MS   2500U
#define APP_ESP_SEND_PROMPT_TIMEOUT_MS 600U
#define APP_ESP_SEND_ACK_TIMEOUT_MS    800U
#define APP_ESP_RESET_LOW_MS     10U
#define APP_ESP_RESET_SETTLE_MS  1000U
#define APP_ESP_RX_SLICE_BYTES   64U
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

typedef enum
{
    /* Wi-Fi + TCP upload state machine (driven by app_service_esp). */
    APP_ESP_STATE_IDLE = 0,
    APP_ESP_STATE_WIFI_WAIT_RETRY,
    APP_ESP_STATE_WIFI_RESET_LOW,
    APP_ESP_STATE_WIFI_RESET_SETTLE,
    APP_ESP_STATE_WIFI_CMD_AT,
    APP_ESP_STATE_WIFI_CMD_ATE0,
    APP_ESP_STATE_WIFI_CMD_CWMODE,
    APP_ESP_STATE_WIFI_CMD_CIPMUX,
    APP_ESP_STATE_WIFI_CMD_CWJAP,
    APP_ESP_STATE_READY,
    APP_ESP_STATE_UPLOAD_CMD_CIPSTART,
    APP_ESP_STATE_UPLOAD_CMD_CIPSEND,
    APP_ESP_STATE_UPLOAD_SEND_PAYLOAD,
    APP_ESP_STATE_UPLOAD_WAIT_SEND_OK,
    APP_ESP_STATE_UPLOAD_CMD_CLOSE,
    APP_ESP_STATE_UPLOAD_RECOVER_CLOSE
} APP_EspState;

typedef enum
{
    APP_ESP_EXCHANGE_BUSY = 0,
    APP_ESP_EXCHANGE_OK,
    APP_ESP_EXCHANGE_FAIL
} APP_EspExchangeResult;

typedef struct
{
    /* One in-flight AT exchange context; reused across state-machine ticks. */
    uint8_t active;
    uint32_t tag;
    uint32_t deadlineTick;
    uint16_t responseLength;
    char response[APP_ESP_RESPONSE_SIZE];
} APP_EspExchangeContext;

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
static uint8_t s_tcpConnected = 0U;
static uint32_t s_lastDhtPollTick = 0U;
static uint32_t s_lastWifiAttemptTick = 0U;
static uint32_t s_nextUploadAttemptTick = 0U;
static APP_EspState s_espState = APP_ESP_STATE_IDLE;
static uint32_t s_espStateDeadlineTick = 0U;
static APP_EspExchangeContext s_espExchange = {0U, 0U, 0U, 0U, {0}};
static uint16_t s_uploadPayloadLength = 0U;
static char s_uploadPayload[160];
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

static uint8_t app_tick_reached(uint32_t nowTick, uint32_t targetTick)
{
    return ((int32_t)(nowTick - targetTick) >= 0) ? 1U : 0U;
}

static void app_esp_exchange_reset(void)
{
    s_espExchange.active = 0U;
    s_espExchange.tag = 0U;
    s_espExchange.deadlineTick = 0U;
    s_espExchange.responseLength = 0U;
    s_espExchange.response[0] = '\0';
}

static void app_esp_exchange_deactivate(void)
{
    s_espExchange.active = 0U;
    s_espExchange.tag = 0U;
    s_espExchange.deadlineTick = 0U;
}

static void app_esp_rx_discard_slice(BSP_ESP01S_HandleTypeDef* esp, uint16_t maxBytes)
{
    uint16_t index;
    uint8_t dummyByte;

    if (esp == 0)
    {
        return;
    }

    for (index = 0U; index < maxBytes; index++)
    {
        if (BSP_ESP01S_Receive(esp, &dummyByte, 1U, 0U) != HAL_OK)
        {
            break;
        }
    }
}

static uint8_t app_esp_collect_response_slice(BSP_ESP01S_HandleTypeDef* esp,
                                              APP_EspExchangeContext* exchange)
{
    uint8_t rxByte;
    uint16_t readCount = 0U;
    HAL_StatusTypeDef status;

    if ((esp == 0) || (exchange == 0))
    {
        return 0U;
    }

    while (readCount < APP_ESP_RX_SLICE_BYTES)
    {
        status = BSP_ESP01S_Receive(esp, &rxByte, 1U, 0U);
        if (status != HAL_OK)
        {
            break;
        }

        if (exchange->responseLength < (APP_ESP_RESPONSE_SIZE - 1U))
        {
            exchange->response[exchange->responseLength] = (char)rxByte;
            exchange->responseLength++;
            exchange->response[exchange->responseLength] = '\0';
        }

        readCount++;
    }

    return (readCount > 0U) ? 1U : 0U;
}

static uint8_t app_esp_response_has_error(const char* response)
{
    if (response == 0)
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

    return 0U;
}

static uint8_t app_esp_response_has_expect(const char* response,
                                           const char* expect1,
                                           const char* expect2)
{
    if (response == 0)
    {
        return 0U;
    }

    if ((expect1 != 0) && (strstr(response, expect1) != 0))
    {
        return 1U;
    }

    if ((expect2 != 0) && (strstr(response, expect2) != 0))
    {
        return 1U;
    }

    return 0U;
}

static APP_EspExchangeResult app_esp_step_exchange(APP_NodeContext* node,
                                                   uint32_t nowTick,
                                                   uint32_t tag,
                                                   const char* command,
                                                   const char* expect1,
                                                   const char* expect2,
                                                   uint32_t timeoutMs,
                                                   uint8_t clearRxOnStart)
{
    HAL_StatusTypeDef status;
    uint8_t rxUpdated;

    if (node == 0)
    {
        app_esp_exchange_reset();
        return APP_ESP_EXCHANGE_FAIL;
    }

    if ((s_espExchange.active != 0U) && (s_espExchange.tag != tag))
    {
        app_esp_exchange_reset();
    }

    if (s_espExchange.active == 0U)
    {
        /* First entry for this step: initialize response buffer and send command once. */
        s_espExchange.active = 1U;
        s_espExchange.tag = tag;
        s_espExchange.deadlineTick = nowTick + timeoutMs;
        s_espExchange.responseLength = 0U;
        s_espExchange.response[0] = '\0';

        if (clearRxOnStart != 0U)
        {
            app_esp_rx_discard_slice(&node->esp, (uint16_t)(APP_ESP_RX_SLICE_BYTES * 3U));
        }

        if ((command != 0) && (command[0] != '\0'))
        {
            app_debug_log("[ESP CMD] %s", command);
            status = BSP_ESP01S_SendString(&node->esp, command, APP_ESP_CMD_TIMEOUT_MS);
            if (status != HAL_OK)
            {
                app_debug_log("[ESP CMD] tx failed, status=%d", (int)status);
                app_esp_exchange_deactivate();
                return APP_ESP_EXCHANGE_FAIL;
            }
        }
    }

    /* Subsequent entries: collect a small RX slice and evaluate completion. */
    rxUpdated = app_esp_collect_response_slice(&node->esp, &s_espExchange);

    if (app_esp_response_has_expect(s_espExchange.response, expect1, expect2) != 0U)
    {
        if (rxUpdated != 0U)
        {
            app_debug_log_response("[ESP RSP] ", s_espExchange.response);
        }
        app_esp_exchange_deactivate();
        return APP_ESP_EXCHANGE_OK;
    }

    if (app_esp_response_has_error(s_espExchange.response) != 0U)
    {
        app_debug_log_response("[ESP ERR] ", s_espExchange.response);
        app_esp_exchange_deactivate();
        return APP_ESP_EXCHANGE_FAIL;
    }

    if (app_tick_reached(nowTick, s_espExchange.deadlineTick) != 0U)
    {
        app_debug_log_response("[ESP TO] ", s_espExchange.response);
        app_esp_exchange_deactivate();
        return APP_ESP_EXCHANGE_FAIL;
    }

    return APP_ESP_EXCHANGE_BUSY;
}

static uint8_t app_esp_send_recv_accepted(const char* response)
{
    if (response == 0)
    {
        return 0U;
    }

    if ((strstr(response, "Recv ") == 0) || (strstr(response, " bytes") == 0))
    {
        return 0U;
    }

    if (app_esp_response_has_error(response) != 0U)
    {
        return 0U;
    }

    return 1U;
}

static uint8_t app_esp_cipstart_likely_connected(const char* response)
{
    if ((response == 0) || (response[0] == '\0'))
    {
        return 0U;
    }

    if ((strstr(response, "ALREADY CONNECTED") != 0) ||
        (strstr(response, "CONNECT") != 0))
    {
        return 1U;
    }

    /*
     * Some traces are truncated under load (for example only "CON").
     * Treat this as likely connected and confirm by next CIPSEND stage.
     */
    if ((strstr(response, "CON") != 0) &&
        (app_esp_response_has_error(response) == 0U))
    {
        return 1U;
    }

    return 0U;
}

static uint8_t app_esp_wifi_join_likely_ok(const char* response)
{
    uint8_t hasConnected = 0U;
    uint8_t hasGotIp = 0U;
    uint8_t hasOk = 0U;
    uint8_t hasDisconnect = 0U;

    if ((response == 0) || (response[0] == '\0'))
    {
        return 0U;
    }

    /*
     * ESP-01S reply can be fragmented or slightly mangled in polling mode.
     * Accept common truncated variants and continue with normal data upload
     * verification in following stages.
     */
    if (app_esp_response_has_error(response) != 0U)
    {
        return 0U;
    }

    if ((strstr(response, "WIFI DISCONNECT") != 0) ||
        (strstr(response, "WIFI DISCONECT") != 0) ||
        (strstr(response, "WIFI DISCONN") != 0) ||
        (strstr(response, " DISCONNECT") != 0))
    {
        hasDisconnect = 1U;
    }

    if ((strstr(response, "WIFI CONNECTED") != 0) ||
        (strstr(response, "WIFI CONNECTD") != 0) ||
        (strstr(response, "WIFI CONECTE") != 0) ||
        (strstr(response, "WIFI CONECTD") != 0) ||
        (strstr(response, "WIFI CONN") != 0) ||
        (strstr(response, "WIFI CONNE") != 0) ||
        (strstr(response, "WIF CONN") != 0) ||
        (strstr(response, " CONNCT") != 0) ||
        (strstr(response, " CONECT") != 0))
    {
        hasConnected = 1U;
    }

    if ((strstr(response, "WIFI GOT IP") != 0) ||
        (strstr(response, "WIFI GOTIP") != 0) ||
        (strstr(response, " GOT IP") != 0) ||
        (strstr(response, " GOTIP") != 0) ||
        (strstr(response, " OT P") != 0))
    {
        hasGotIp = 1U;
    }

    if ((strstr(response, "\r\nOK\r\n") != 0) ||
        (strstr(response, " OK") != 0) ||
        (strstr(response, "OK\r\n") != 0))
    {
        hasOk = 1U;
    }

    if ((hasConnected != 0U) &&
        (hasDisconnect == 0U) &&
        ((hasGotIp != 0U) || (hasOk != 0U)))
    {
        return 1U;
    }

    /* Some modules only emit GOT IP fragment without full CONNECTED token. */
    if (hasGotIp != 0U)
    {
        return 1U;
    }

    /*
     * In unstable UART captures we may only receive "WIFI CONN" before timeout.
     * Accept this to avoid unnecessary reset loops; next upload stage verifies link.
     */
    if ((hasConnected != 0U) && (hasDisconnect == 0U))
    {
        return 1U;
    }

    return 0U;
}

static void app_mark_upload_success(APP_NodeContext* node, uint32_t nowTick)
{
    s_lastUploadOk = 1U;
    s_uploadFailStreak = 0U;
    s_nextUploadAttemptTick = nowTick + APP_UPLOAD_INTERVAL_MS;
    if (s_uploadOkCount < 0xFFFFU)
    {
        s_uploadOkCount++;
    }

    if (node != 0)
    {
        node->lastTelemetryTick = nowTick;
    }
}

static void app_mark_upload_failure(APP_NodeContext* node, uint32_t nowTick)
{
    s_lastUploadOk = 0U;
    s_tcpConnected = 0U;
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
        s_uploadFailStreak = 0U;
        s_nextUploadAttemptTick = nowTick + APP_UPLOAD_INTERVAL_MS;
        if (node != 0)
        {
            node->wifiConnected = 0U;
        }
    }
}

static void app_update_runtime_view(APP_NodeContext* node)
{
    if (node == 0)
    {
        return;
    }

    node->wifiConnected = s_wifiConnected;
    node->uploadFailStreak = s_uploadFailStreak;
    node->lastUploadOk = s_lastUploadOk;
    node->uploadOkCount = s_uploadOkCount;
    node->uploadFailCount = s_uploadFailCount;
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
    APP_EspExchangeResult exchangeResult;
    HAL_StatusTypeDef status;
    int payloadLength;
    char command[96];

    app_update_runtime_view(node);

    if (node == 0)
    {
        return;
    }

    if (s_wifiConfigured == 0U)
    {
        s_wifiConnected = 0U;
        s_tcpConnected = 0U;
        s_espState = APP_ESP_STATE_IDLE;
        app_esp_exchange_reset();
        app_update_runtime_view(node);
        return;
    }

    if (s_espState == APP_ESP_STATE_IDLE)
    {
        s_espState = (s_wifiConnected != 0U) ? APP_ESP_STATE_READY : APP_ESP_STATE_WIFI_WAIT_RETRY;
        app_esp_exchange_reset();
    }

    switch (s_espState)
    {
        case APP_ESP_STATE_WIFI_WAIT_RETRY:
            /* Global reconnect gate to avoid aggressive reset/AT storms. */
            if (app_tick_reached(nowTick, s_lastWifiAttemptTick + APP_WIFI_RETRY_MS) == 0U)
            {
                break;
            }

            app_debug_log("[NET] wifi disconnected, retry join");
            s_tcpConnected = 0U;
            s_lastWifiAttemptTick = nowTick;
            BSP_ESP01S_SetEnable(&node->esp, 1U);
            HAL_GPIO_WritePin(node->esp.rstPort, node->esp.rstPin, GPIO_PIN_RESET);
            s_espStateDeadlineTick = nowTick + APP_ESP_RESET_LOW_MS;
            s_espState = APP_ESP_STATE_WIFI_RESET_LOW;
            app_esp_exchange_reset();
            break;

        case APP_ESP_STATE_WIFI_RESET_LOW:
            if (app_tick_reached(nowTick, s_espStateDeadlineTick) == 0U)
            {
                break;
            }

            HAL_GPIO_WritePin(node->esp.rstPort, node->esp.rstPin, GPIO_PIN_SET);
            s_espStateDeadlineTick = nowTick + APP_ESP_RESET_SETTLE_MS;
            s_espState = APP_ESP_STATE_WIFI_RESET_SETTLE;
            break;

        case APP_ESP_STATE_WIFI_RESET_SETTLE:
            if (app_tick_reached(nowTick, s_espStateDeadlineTick) == 0U)
            {
                break;
            }

            s_espState = APP_ESP_STATE_WIFI_CMD_AT;
            app_esp_exchange_reset();
            break;

        case APP_ESP_STATE_WIFI_CMD_AT:
            exchangeResult = app_esp_step_exchange(node,
                                                   nowTick,
                                                   (uint32_t)APP_ESP_STATE_WIFI_CMD_AT,
                                                   "AT\r\n",
                                                   "OK",
                                                   0,
                                                   APP_ESP_CMD_TIMEOUT_MS,
                                                   1U);
            if (exchangeResult == APP_ESP_EXCHANGE_OK)
            {
                s_espState = APP_ESP_STATE_WIFI_CMD_ATE0;
            }
            else if (exchangeResult == APP_ESP_EXCHANGE_FAIL)
            {
                app_debug_log("[NET] wifi connect failed: AT");
                s_wifiConnected = 0U;
                s_tcpConnected = 0U;
                s_espState = APP_ESP_STATE_WIFI_WAIT_RETRY;
            }
            break;

        case APP_ESP_STATE_WIFI_CMD_ATE0:
            exchangeResult = app_esp_step_exchange(node,
                                                   nowTick,
                                                   (uint32_t)APP_ESP_STATE_WIFI_CMD_ATE0,
                                                   "ATE0\r\n",
                                                   "OK",
                                                   0,
                                                   APP_ESP_CMD_TIMEOUT_MS,
                                                   1U);
            if (exchangeResult == APP_ESP_EXCHANGE_FAIL)
            {
                app_debug_log("[NET] wifi connect warning: ATE0 skipped");
            }

            if (exchangeResult != APP_ESP_EXCHANGE_BUSY)
            {
                s_espState = APP_ESP_STATE_WIFI_CMD_CWMODE;
            }
            break;

        case APP_ESP_STATE_WIFI_CMD_CWMODE:
            exchangeResult = app_esp_step_exchange(node,
                                                   nowTick,
                                                   (uint32_t)APP_ESP_STATE_WIFI_CMD_CWMODE,
                                                   "AT+CWMODE=1\r\n",
                                                   "OK",
                                                   0,
                                                   APP_ESP_CMD_TIMEOUT_MS,
                                                   1U);
            if (exchangeResult == APP_ESP_EXCHANGE_OK)
            {
                s_espState = APP_ESP_STATE_WIFI_CMD_CIPMUX;
            }
            else if (exchangeResult == APP_ESP_EXCHANGE_FAIL)
            {
                app_debug_log("[NET] wifi connect failed: CWMODE");
                s_wifiConnected = 0U;
                s_tcpConnected = 0U;
                s_espState = APP_ESP_STATE_WIFI_WAIT_RETRY;
            }
            break;

        case APP_ESP_STATE_WIFI_CMD_CIPMUX:
            exchangeResult = app_esp_step_exchange(node,
                                                   nowTick,
                                                   (uint32_t)APP_ESP_STATE_WIFI_CMD_CIPMUX,
                                                   "AT+CIPMUX=0\r\n",
                                                   "OK",
                                                   0,
                                                   APP_ESP_CMD_TIMEOUT_MS,
                                                   1U);
            if (exchangeResult == APP_ESP_EXCHANGE_OK)
            {
                s_espState = APP_ESP_STATE_WIFI_CMD_CWJAP;
            }
            else if (exchangeResult == APP_ESP_EXCHANGE_FAIL)
            {
                app_debug_log("[NET] wifi connect failed: CIPMUX");
                s_wifiConnected = 0U;
                s_tcpConnected = 0U;
                s_espState = APP_ESP_STATE_WIFI_WAIT_RETRY;
            }
            break;

        case APP_ESP_STATE_WIFI_CMD_CWJAP:
            /* Join AP can return truncated text; handled by tolerant parser below. */
            (void)snprintf(command,
                           sizeof(command),
                           "AT+CWJAP=\"%s\",\"%s\"\r\n",
                           APP_WIFI_SSID,
                           APP_WIFI_PASSWORD);
            exchangeResult = app_esp_step_exchange(node,
                                                   nowTick,
                                                   (uint32_t)APP_ESP_STATE_WIFI_CMD_CWJAP,
                                                   command,
                                                   "WIFI GOT IP",
                                                   "OK",
                                                   APP_ESP_JOIN_TIMEOUT_MS,
                                                   1U);
            if ((exchangeResult == APP_ESP_EXCHANGE_OK) ||
                ((exchangeResult == APP_ESP_EXCHANGE_FAIL) &&
                 (app_esp_wifi_join_likely_ok(s_espExchange.response) != 0U)))
            {
                if (exchangeResult == APP_ESP_EXCHANGE_FAIL)
                {
                    app_debug_log("[NET] wifi join uncertain, accept as connected");
                }
                else
                {
                    app_debug_log("[NET] wifi connected");
                }

                s_wifiConnected = 1U;
                s_tcpConnected = 0U;
                s_uploadFailStreak = 0U;
                s_nextUploadAttemptTick = nowTick + APP_UPLOAD_INTERVAL_MS;
                app_debug_log("[NET] wait one upload interval after connect");
                s_espState = APP_ESP_STATE_READY;
            }
            else if (exchangeResult == APP_ESP_EXCHANGE_FAIL)
            {
                app_debug_log("[NET] wifi join retry failed");
                s_wifiConnected = 0U;
                s_tcpConnected = 0U;
                s_espState = APP_ESP_STATE_WIFI_WAIT_RETRY;
            }
            break;

        case APP_ESP_STATE_READY:
            if (s_wifiConnected == 0U)
            {
                s_tcpConnected = 0U;
                s_espState = APP_ESP_STATE_WIFI_WAIT_RETRY;
                break;
            }

            if (s_uploadConfigured == 0U)
            {
                break;
            }

            if (app_tick_reached(nowTick, s_nextUploadAttemptTick) == 0U)
            {
                break;
            }

            payloadLength = snprintf(s_uploadPayload,
                                     sizeof(s_uploadPayload),
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
            if ((payloadLength <= 0) || ((uint32_t)payloadLength >= sizeof(s_uploadPayload)))
            {
                app_debug_log("[NET] payload build failed");
                app_mark_upload_failure(node, nowTick);
                s_espState = (s_wifiConnected != 0U) ? APP_ESP_STATE_READY : APP_ESP_STATE_WIFI_WAIT_RETRY;
                break;
            }

            s_uploadPayloadLength = (uint16_t)payloadLength;
            app_debug_log("[NET] upload payload: %s", s_uploadPayload);
            /* Prefer long connection: skip CIPSTART when TCP link is considered alive. */
            s_espState = (s_tcpConnected != 0U) ? APP_ESP_STATE_UPLOAD_CMD_CIPSEND : APP_ESP_STATE_UPLOAD_CMD_CIPSTART;
            app_esp_exchange_reset();
            break;

        case APP_ESP_STATE_UPLOAD_CMD_CIPSTART:
            /* Open TCP only when needed; ambiguous timeout is handled conservatively. */
            (void)snprintf(command,
                           sizeof(command),
                           "AT+CIPSTART=\"TCP\",\"%s\",%lu\r\n",
                           APP_UPLOAD_HOST,
                           (unsigned long)APP_UPLOAD_PORT);
            exchangeResult = app_esp_step_exchange(node,
                                                   nowTick,
                                                   (uint32_t)APP_ESP_STATE_UPLOAD_CMD_CIPSTART,
                                                   command,
                                                   "OK",
                                                   "ALREADY CONNECTED",
                                                   APP_ESP_TCP_TIMEOUT_MS,
                                                   1U);
            if ((exchangeResult == APP_ESP_EXCHANGE_OK) ||
                ((exchangeResult == APP_ESP_EXCHANGE_FAIL) &&
                 (app_esp_cipstart_likely_connected(s_espExchange.response) != 0U)))
            {
                s_tcpConnected = 1U;
                if (exchangeResult == APP_ESP_EXCHANGE_FAIL)
                {
                    app_debug_log("[NET] CIPSTART uncertain, continue CIPSEND");
                }
                s_espState = APP_ESP_STATE_UPLOAD_CMD_CIPSEND;
            }
            else if (exchangeResult == APP_ESP_EXCHANGE_FAIL)
            {
                if (app_esp_response_has_error(s_espExchange.response) == 0U)
                {
                    s_tcpConnected = 1U;
                    app_debug_log("[NET] CIPSTART timeout w/o error, try CIPSEND");
                    s_espState = APP_ESP_STATE_UPLOAD_CMD_CIPSEND;
                }
                else
                {
                    app_debug_log("[NET] CIPSTART failed");
                    app_mark_upload_failure(node, nowTick);
                    s_espState = APP_ESP_STATE_UPLOAD_RECOVER_CLOSE;
                }
            }
            break;

        case APP_ESP_STATE_UPLOAD_CMD_CIPSEND:
            /* Ask for send prompt. If prompt text is missing but no explicit error, continue. */
            (void)snprintf(command,
                           sizeof(command),
                           "AT+CIPSEND=%u\r\n",
                           (unsigned int)s_uploadPayloadLength);
            exchangeResult = app_esp_step_exchange(node,
                                                   nowTick,
                                                   (uint32_t)APP_ESP_STATE_UPLOAD_CMD_CIPSEND,
                                                   command,
                                                   ">",
                                                   0,
                                                   APP_ESP_SEND_PROMPT_TIMEOUT_MS,
                                                   1U);
            if (exchangeResult == APP_ESP_EXCHANGE_OK)
            {
                s_espState = APP_ESP_STATE_UPLOAD_SEND_PAYLOAD;
            }
            else if (exchangeResult == APP_ESP_EXCHANGE_FAIL)
            {
                if (app_esp_response_has_error(s_espExchange.response) == 0U)
                {
                    app_debug_log("[NET] CIPSEND timeout w/o error, try payload");
                    s_espState = APP_ESP_STATE_UPLOAD_SEND_PAYLOAD;
                }
                else
                {
                    app_debug_log("[NET] CIPSEND prompt failed");
                    app_mark_upload_failure(node, nowTick);
                    s_espState = APP_ESP_STATE_UPLOAD_RECOVER_CLOSE;
                }
            }
            break;

        case APP_ESP_STATE_UPLOAD_SEND_PAYLOAD:
            if (s_uploadPayloadLength == 0U)
            {
                app_debug_log("[NET] empty payload");
                app_mark_upload_failure(node, nowTick);
                s_espState = APP_ESP_STATE_UPLOAD_RECOVER_CLOSE;
                break;
            }

            status = BSP_ESP01S_Send(&node->esp,
                                     (const uint8_t*)s_uploadPayload,
                                     s_uploadPayloadLength,
                                     APP_ESP_CMD_TIMEOUT_MS);
            if (status != HAL_OK)
            {
                app_debug_log("[NET] payload tx failed");
                app_mark_upload_failure(node, nowTick);
                s_espState = APP_ESP_STATE_UPLOAD_RECOVER_CLOSE;
                break;
            }

            s_espState = APP_ESP_STATE_UPLOAD_WAIT_SEND_OK;
            app_esp_exchange_reset();
            break;

        case APP_ESP_STATE_UPLOAD_WAIT_SEND_OK:
            /*
             * Some firmwares may not return full "SEND OK" in polling mode.
             * If no explicit ERROR/FAIL appears, treat as soft-success to keep cadence.
             */
            exchangeResult = app_esp_step_exchange(node,
                                                   nowTick,
                                                   (uint32_t)APP_ESP_STATE_UPLOAD_WAIT_SEND_OK,
                                                   0,
                                                   "SEND OK",
                                                   0,
                                                   APP_ESP_SEND_ACK_TIMEOUT_MS,
                                                   0U);
            if (exchangeResult == APP_ESP_EXCHANGE_OK)
            {
                s_tcpConnected = 1U;
                app_mark_upload_success(node, nowTick);
                app_debug_log("[NET] upload ok");
                s_espState = APP_ESP_STATE_READY;
                break;
            }

            if ((exchangeResult == APP_ESP_EXCHANGE_BUSY) &&
                (app_esp_send_recv_accepted(s_espExchange.response) != 0U))
            {
                app_esp_exchange_reset();
                s_tcpConnected = 1U;
                app_mark_upload_success(node, nowTick);
                app_debug_log("[NET] upload ok");
                s_espState = APP_ESP_STATE_READY;
                break;
            }

            if (exchangeResult == APP_ESP_EXCHANGE_FAIL)
            {
                if (app_esp_send_recv_accepted(s_espExchange.response) != 0U)
                {
                    s_tcpConnected = 1U;
                    app_mark_upload_success(node, nowTick);
                    app_debug_log("[NET] upload ok");
                    s_espState = APP_ESP_STATE_READY;
                }
                else if (app_esp_response_has_error(s_espExchange.response) == 0U)
                {
                    app_debug_log("[NET] send ack timeout, assume sent");
                    s_tcpConnected = 1U;
                    app_mark_upload_success(node, nowTick);
                    app_debug_log("[NET] upload ok");
                    s_espState = APP_ESP_STATE_READY;
                }
                else
                {
                    app_debug_log("[NET] send result not ok");
                    app_mark_upload_failure(node, nowTick);
                    s_espState = APP_ESP_STATE_UPLOAD_RECOVER_CLOSE;
                }
            }
            break;

        case APP_ESP_STATE_UPLOAD_CMD_CLOSE:
            exchangeResult = app_esp_step_exchange(node,
                                                   nowTick,
                                                   (uint32_t)APP_ESP_STATE_UPLOAD_CMD_CLOSE,
                                                   "AT+CIPCLOSE\r\n",
                                                   "OK",
                                                   "CLOSED",
                                                   APP_ESP_CMD_TIMEOUT_MS,
                                                   1U);
            if (exchangeResult == APP_ESP_EXCHANGE_BUSY)
            {
                break;
            }

            if (exchangeResult == APP_ESP_EXCHANGE_FAIL)
            {
                app_debug_log("[NET] close not confirmed");
            }

            s_tcpConnected = 0U;
            app_mark_upload_success(node, nowTick);
            app_debug_log("[NET] upload ok");
            s_espState = APP_ESP_STATE_READY;
            break;

        case APP_ESP_STATE_UPLOAD_RECOVER_CLOSE:
            /* Recovery path: try close once, then fall back to READY / reconnect flow. */
            exchangeResult = app_esp_step_exchange(node,
                                                   nowTick,
                                                   (uint32_t)APP_ESP_STATE_UPLOAD_RECOVER_CLOSE,
                                                   "AT+CIPCLOSE\r\n",
                                                   "OK",
                                                   "CLOSED",
                                                   APP_ESP_CMD_TIMEOUT_MS,
                                                   1U);
            if (exchangeResult == APP_ESP_EXCHANGE_BUSY)
            {
                break;
            }

            s_tcpConnected = 0U;
            s_espState = (s_wifiConnected != 0U) ? APP_ESP_STATE_READY : APP_ESP_STATE_WIFI_WAIT_RETRY;
            break;

        default:
            s_espState = APP_ESP_STATE_IDLE;
            app_esp_exchange_reset();
            break;
    }

    app_update_runtime_view(node);
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
    s_tcpConnected = 0U;
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
    s_espState = APP_ESP_STATE_IDLE;
    s_espStateDeadlineTick = nowTick;
    app_esp_exchange_reset();
    s_uploadPayloadLength = 0U;
    s_uploadPayload[0] = '\0';
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

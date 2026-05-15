#include "app_node.h"
#include "app_home_protocol.h"
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
#define APP_HW_SELF_TEST_ON_BOOT 0U
#define APP_SELF_TEST_RELAY_ON_MS 3000U
#define APP_SELF_TEST_RELAY_GAP_MS 500U
#define APP_DHT11_POLL_MS        2500U
#define APP_WIFI_RETRY_MS        30000U
#define APP_UPLOAD_INTERVAL_MS   20000U
#define APP_UPLOAD_RETRY_MS      20000U
#define APP_STATE_UPLOAD_DEBOUNCE_MS 1500U
#define APP_PROTOCOL_REPLY_UPLOAD_GUARD_MS 1200U
#define APP_ESP_POST_IP_UPLOAD_DELAY_MS 8000U
#define APP_CIPSTART_FAIL_RETRY_MS 6000U
#define APP_UPLOAD_FAIL_REJOIN_THRESHOLD 6U
#define APP_ESP_RESPONSE_SIZE    256U
#define APP_ESP_CMD_TIMEOUT_MS   1500U
#define APP_ESP_JOIN_TIMEOUT_MS  30000U
#define APP_ESP_CWJAP_RETRY_MS   5000U
#define APP_ESP_CWJAP_RETRY_MAX  3U
#define APP_ESP_TCP_TIMEOUT_MS   8000U
#define APP_ESP_PING_TIMEOUT_MS  5000U
#define APP_ESP_PRE_TCP_SETTLE_MS 300U
#define APP_ESP_CIPSTART_PROBE_DELAY_MS 1000U
#define APP_ESP_SEND_PROMPT_TIMEOUT_MS 2500U
#define APP_ESP_SEND_ACK_TIMEOUT_MS    12000U
#define APP_ESP_POST_SEND_CLOSE_DELAY_MS 500U
#define APP_ESP_TCP_REOPEN_SETTLE_MS   1500U
#define APP_ESP_RESET_LOW_MS     100U
#define APP_ESP_RESET_SETTLE_MS  3000U
#define APP_ESP_JOIN_SETTLE_MS   3000U
#define APP_ESP_CIFSR_RETRY_MS   2000U
#define APP_ESP_CIFSR_RETRY_MAX  3U
#define APP_ESP_RX_SLICE_BYTES   64U
#define APP_DEBUG_TX_TIMEOUT_MS  120U
#ifndef APP_NODE_ID
#define APP_NODE_ID              APP_HOME_NODE_KITCHEN
#endif

#if (APP_NODE_ID < APP_HOME_NODE_KITCHEN) || (APP_NODE_ID > APP_HOME_NODE_BEDROOM)
#error "APP_NODE_ID must be 1(kitchen), 2(living), or 3(bedroom)"
#endif
#define APP_HOME_TELEMETRY_PAYLOAD_LEN 8U
#define APP_HOME_ACK_PAYLOAD_LEN       1U
#define APP_HOME_ERR_PAYLOAD_LEN       2U
#define APP_HOME_MODE_OFF       0U
#define APP_HOME_MODE_COOL      1U
#define APP_HOME_MODE_HEAT      2U
#define APP_HOME_MODE_AUTO      3U
#define APP_HOME_FAN_OFF        0U
#define APP_HOME_FAN_LOW        1U
#define APP_HOME_FAN_MED        2U
#define APP_HOME_FAN_HIGH       3U
#define APP_HOME_FAN_AUTO       4U
#define APP_HOME_FAN_DUTY_LOW   35U
#define APP_HOME_FAN_DUTY_MED   60U
#define APP_HOME_FAN_DUTY_HIGH  100U
#define APP_HOME_FAN_DUTY_AUTO  60U

/*
 * Fill these values locally before Wi-Fi debug:
 * - SSID/PASSWORD for AP join
 * - HOST/PORT for telemetry upload endpoint
 */
#define APP_WIFI_SSID     "WXSC_Air"
#define APP_WIFI_PASSWORD ""
#define APP_UPLOAD_HOST   "10.20.209.130"
#define APP_UPLOAD_PORT   5000U

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
    APP_ESP_STATE_WIFI_CMD_SLEEP,
    APP_ESP_STATE_WIFI_CMD_CIPMODE,
    APP_ESP_STATE_WIFI_CMD_CIPMUX,
    APP_ESP_STATE_WIFI_CMD_CWJAP,
    APP_ESP_STATE_WIFI_CWJAP_RETRY_WAIT,
    APP_ESP_STATE_WIFI_JOIN_SETTLE,
    APP_ESP_STATE_WIFI_CMD_CIFSR,
    APP_ESP_STATE_WIFI_CIFSR_RETRY_WAIT,
    APP_ESP_STATE_WIFI_CMD_CIPSTATUS,
    APP_ESP_STATE_READY,
    APP_ESP_STATE_UPLOAD_CMD_PING,
    APP_ESP_STATE_UPLOAD_PING_SETTLE,
    APP_ESP_STATE_UPLOAD_CMD_CIPSTART,
    APP_ESP_STATE_UPLOAD_CIPSTART_SETTLE,
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

typedef enum
{
    APP_SEND_KIND_NONE = 0,
    APP_SEND_KIND_TELEMETRY,
    APP_SEND_KIND_ACK,
    APP_SEND_KIND_ERR
} APP_SendKind;

typedef enum
{
    APP_IPD_STATE_IDLE = 0,
    APP_IPD_STATE_MATCH,
    APP_IPD_STATE_LENGTH,
    APP_IPD_STATE_DATA
} APP_IpdState;

typedef struct
{
    APP_IpdState state;
    uint8_t matchIndex;
    uint16_t remaining;
} APP_IpdParser;

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
static uint32_t s_tcpReopenNotBeforeTick = 0U;
static APP_EspState s_espState = APP_ESP_STATE_IDLE;
static uint32_t s_espStateDeadlineTick = 0U;
static uint8_t s_cwjapRetryCount = 0U;
static uint8_t s_cifsrRetryCount = 0U;
static APP_EspExchangeContext s_espExchange = {0U, 0U, 0U, 0U, {0}};
static APP_HomeProtocolParser_t s_protocolParser;
static APP_HomeProtocolFrame_t s_protocolFrame;
static APP_IpdParser s_ipdParser = {APP_IPD_STATE_IDLE, 0U, 0U};
static uint16_t s_protocolTxSequence = 1U;
static uint16_t s_uploadPayloadLength = 0U;
static uint8_t s_uploadPayload[APP_HOME_PROTOCOL_MAX_FRAME_LEN];
static APP_SendKind s_uploadPayloadKind = APP_SEND_KIND_NONE;
static uint8_t s_stateUploadPending = 0U;
static uint32_t s_stateUploadReadyTick = 0U;
static uint32_t s_stateUploadGeneration = 0U;
static uint32_t s_uploadPayloadStateGeneration = 0U;
static uint8_t s_payloadSentAfterMissingPrompt = 0U;
static uint8_t s_pendingReplyFrame[APP_HOME_PROTOCOL_MAX_FRAME_LEN];
static uint16_t s_pendingReplyLength = 0U;
static APP_SendKind s_pendingReplyKind = APP_SEND_KIND_NONE;
static int16_t s_targetTemperatureX10 = 240;
static uint8_t s_controlMode = APP_HOME_MODE_AUTO;
static uint8_t s_controlFan = APP_HOME_FAN_MED;
static const char s_ipdToken[] = "+IPD,";
static UART_HandleTypeDef* s_debugUart = 0;

static uint8_t app_is_wifi_configured(void)
{
    /* Empty password is valid for open APs; only SSID is mandatory. */
    return (APP_WIFI_SSID[0] != '\0') ? 1U : 0U;
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

static void app_debug_log_hex(const char* prefix, const uint8_t* data, uint16_t length)
{
    char line[192];
    uint16_t limit = length;
    int offset;

    if ((prefix == 0) || (data == 0))
    {
        return;
    }

    if (limit > 32U)
    {
        limit = 32U;
    }

    offset = snprintf(line, sizeof(line), "%s len=%u:", prefix, (unsigned int)length);
    if (offset < 0)
    {
        return;
    }

    for (uint16_t index = 0U; (index < limit) && (offset < ((int)sizeof(line) - 4)); index++)
    {
        offset += snprintf(&line[offset], sizeof(line) - (uint16_t)offset, " %02X", (unsigned int)data[index]);
    }

    if ((limit < length) && (offset < ((int)sizeof(line) - 5)))
    {
        (void)snprintf(&line[offset], sizeof(line) - (uint16_t)offset, " ...");
    }

    app_debug_log("%s", line);
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

static void app_log_relay_expected_state(const char* reason)
{
    app_debug_log("[RLY] %s RLY1=%u RLY2=%u RLY3=%u",
                  (reason != 0) ? reason : "state",
                  (unsigned int)s_relayState[0],
                  (unsigned int)s_relayState[1],
                  (unsigned int)s_relayState[2]);
}

static HAL_StatusTypeDef app_apply_relay_state(uint8_t relayIndex)
{
    HAL_StatusTypeDef status;

    status = BSP_Relay_Set((BSP_RelayChannel)relayIndex, s_relayState[relayIndex]);
    app_log_relay_expected_state("expected");
    return status;
}

static void app_set_all_relay_states(uint8_t enable, const char* reason)
{
    uint8_t state = (enable != 0U) ? 1U : 0U;

    s_relayState[0] = state;
    s_relayState[1] = state;
    s_relayState[2] = state;
    BSP_Relay_SetAll(state);
    app_log_relay_expected_state(reason);
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

static void app_mark_upload_success(APP_NodeContext* node, uint32_t nowTick);
static void app_mark_upload_failure(APP_NodeContext* node, uint32_t nowTick);
static void app_esp_exchange_reset(void);
static uint8_t app_upload_payload_is_protocol_reply(void);
static void app_finish_upload_send_success_peer_closed(APP_NodeContext* node, uint32_t nowTick, const char* reason);
static void app_recover_tcp_after_peer_closed(APP_NodeContext* node, uint32_t nowTick, const char* reason);

static uint16_t app_read_le16(const uint8_t* data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static int16_t app_read_i16(const uint8_t* data)
{
    return (int16_t)app_read_le16(data);
}

static void app_write_le16(uint8_t* data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static uint16_t app_next_protocol_sequence(void)
{
    uint16_t sequence = s_protocolTxSequence;

    s_protocolTxSequence++;
    if (s_protocolTxSequence == 0U)
    {
        s_protocolTxSequence = 1U;
    }

    return sequence;
}

static uint8_t app_current_relay_flags(void)
{
    uint8_t flags = 0U;

    if (s_relayState[0] != 0U)
    {
        flags |= APP_HOME_CONTROL_FLAG_USB1;
    }
    if (s_relayState[1] != 0U)
    {
        flags |= APP_HOME_CONTROL_FLAG_USB2;
    }
    if (s_relayState[2] != 0U)
    {
        flags |= APP_HOME_CONTROL_FLAG_USB3;
    }

    return flags;
}

static void app_schedule_state_upload(uint32_t nowTick, const char* reason)
{
    s_stateUploadPending = 1U;
    s_stateUploadReadyTick = nowTick + APP_STATE_UPLOAD_DEBOUNCE_MS;
    s_stateUploadGeneration++;
    if (s_stateUploadGeneration == 0U)
    {
        s_stateUploadGeneration = 1U;
    }

    app_debug_log("[NET] state upload pending reason=%s flags=0x%02X settle=%ums",
                  (reason != 0) ? reason : "state",
                  (unsigned int)app_current_relay_flags(),
                  (unsigned int)APP_STATE_UPLOAD_DEBOUNCE_MS);
}

static uint8_t app_fan_mode_to_duty(uint8_t fan)
{
    switch (fan)
    {
        case APP_HOME_FAN_LOW:
            return APP_HOME_FAN_DUTY_LOW;
        case APP_HOME_FAN_MED:
            return APP_HOME_FAN_DUTY_MED;
        case APP_HOME_FAN_HIGH:
            return APP_HOME_FAN_DUTY_HIGH;
        case APP_HOME_FAN_AUTO:
            return APP_HOME_FAN_DUTY_AUTO;
        case APP_HOME_FAN_OFF:
        default:
            return 0U;
    }
}

static void app_queue_protocol_reply(uint8_t command,
                                     uint16_t sequence,
                                     const uint8_t* payload,
                                     uint16_t payloadLength,
                                     APP_SendKind kind)
{
    uint16_t frameLength;

    frameLength = APP_HomeProtocol_BuildFrame(APP_NODE_ID,
                                              command,
                                              sequence,
                                              payload,
                                              payloadLength,
                                              s_pendingReplyFrame,
                                              (uint16_t)sizeof(s_pendingReplyFrame));
    if (frameLength == 0U)
    {
        app_debug_log("[PROTO] reply build failed cmd=0x%02X", (unsigned int)command);
        return;
    }

    s_pendingReplyLength = frameLength;
    s_pendingReplyKind = kind;
}

static void app_queue_protocol_ack(const APP_HomeProtocolFrame_t* frame)
{
    uint8_t payload[APP_HOME_ACK_PAYLOAD_LEN];

    if (frame == 0)
    {
        return;
    }

    payload[0] = frame->command;
    app_queue_protocol_reply(APP_HOME_CMD_ACK,
                             frame->sequence,
                             payload,
                             (uint16_t)sizeof(payload),
                             APP_SEND_KIND_ACK);
    app_debug_log("[PROTO] ack queued seq=%u cmd=%s",
                  (unsigned int)frame->sequence,
                  APP_HomeProtocol_CommandToString(frame->command));
}

static void app_queue_protocol_error(const APP_HomeProtocolFrame_t* frame,
                                     APP_HomeProtocolError_t error)
{
    uint8_t payload[APP_HOME_ERR_PAYLOAD_LEN];
    uint16_t sequence = 0U;
    uint8_t command = 0U;

    if (frame != 0)
    {
        sequence = frame->sequence;
        command = frame->command;
    }

    payload[0] = command;
    payload[1] = (uint8_t)error;
    app_queue_protocol_reply(APP_HOME_CMD_ERR,
                             sequence,
                             payload,
                             (uint16_t)sizeof(payload),
                             APP_SEND_KIND_ERR);
    app_debug_log("[PROTO] err queued seq=%u cmd=0x%02X err=%u",
                  (unsigned int)sequence,
                  (unsigned int)command,
                  (unsigned int)error);
}

static void app_apply_control_frame(APP_NodeContext* node, const APP_HomeProtocolFrame_t* frame)
{
    int16_t target;
    uint8_t mode;
    uint8_t fan;
    uint8_t flags;
    uint8_t relayIndex;

    if ((node == 0) || (frame == 0) || (frame->length != APP_HOME_CONTROL_PAYLOAD_LEN))
    {
        return;
    }

    target = app_read_i16(&frame->payload[0]);
    mode = frame->payload[2];
    fan = frame->payload[3];
    flags = frame->payload[4];

    if (mode <= APP_HOME_MODE_AUTO)
    {
        s_controlMode = mode;
    }
    if (fan <= APP_HOME_FAN_AUTO)
    {
        s_controlFan = fan;
    }
    s_targetTemperatureX10 = target;

    s_relayState[0] = ((flags & APP_HOME_CONTROL_FLAG_USB1) != 0U) ? 1U : 0U;
    s_relayState[1] = ((flags & APP_HOME_CONTROL_FLAG_USB2) != 0U) ? 1U : 0U;
    s_relayState[2] = ((flags & APP_HOME_CONTROL_FLAG_USB3) != 0U) ? 1U : 0U;
    for (relayIndex = 0U; relayIndex < 3U; relayIndex++)
    {
        (void)app_apply_relay_state(relayIndex);
    }

    if ((s_controlMode == APP_HOME_MODE_OFF) || (s_controlFan == APP_HOME_FAN_OFF))
    {
        s_fanEnabled = 0U;
    }
    else
    {
        s_fanEnabled = 1U;
        s_fanDutyPercent = app_fan_mode_to_duty(s_controlFan);
        if (s_fanDutyPercent == 0U)
        {
            s_fanDutyPercent = APP_HOME_FAN_DUTY_AUTO;
        }
    }
    node->fanDutyPercent = s_fanDutyPercent;
    app_apply_fan_state();

    app_debug_log("[PROTO] control apply node=%u seq=%u target_x10=%d mode=%u fan=%u flags=0x%02X",
                  (unsigned int)frame->node,
                  (unsigned int)frame->sequence,
                  (int)s_targetTemperatureX10,
                  (unsigned int)s_controlMode,
                  (unsigned int)s_controlFan,
                  (unsigned int)flags);

    app_schedule_state_upload(HAL_GetTick(), "control");
}

static void app_handle_protocol_frame(APP_NodeContext* node, const APP_HomeProtocolFrame_t* frame)
{
    APP_HomeProtocolError_t error = APP_HOME_ERR_NONE;

    if ((node == 0) || (frame == 0))
    {
        return;
    }

    app_debug_log("[PROTO] rx node=%u cmd=%s seq=%u len=%u",
                  (unsigned int)frame->node,
                  APP_HomeProtocol_CommandToString(frame->command),
                  (unsigned int)frame->sequence,
                  (unsigned int)frame->length);

    if ((frame->command == APP_HOME_CMD_ACK) || (frame->command == APP_HOME_CMD_ERR))
    {
        return;
    }

    if (frame->node != APP_NODE_ID)
    {
        error = APP_HOME_ERR_BAD_NODE;
    }
    else
    {
        switch (frame->command)
        {
            case APP_HOME_CMD_CONTROL:
                if (frame->length != APP_HOME_CONTROL_PAYLOAD_LEN)
                {
                    error = APP_HOME_ERR_BAD_LENGTH;
                }
                else
                {
                    app_apply_control_frame(node, frame);
                }
                break;

            case APP_HOME_CMD_HEARTBEAT:
            case APP_HOME_CMD_HELLO:
                break;

            default:
                error = APP_HOME_ERR_BAD_COMMAND;
                break;
        }
    }

    if (error != APP_HOME_ERR_NONE)
    {
        app_queue_protocol_error(frame, error);
        return;
    }

    app_queue_protocol_ack(frame);
}

static void app_feed_protocol_byte(APP_NodeContext* node, uint8_t byte)
{
    APP_HomeProtocolParseResult_t result;

    result = APP_HomeProtocol_PushByte(&s_protocolParser, byte, &s_protocolFrame);
    if (result == APP_HOME_PARSE_FRAME)
    {
        app_handle_protocol_frame(node, &s_protocolFrame);
    }
    else if (result == APP_HOME_PARSE_ERROR)
    {
        app_debug_log("[PROTO] parse error=%u", (unsigned int)s_protocolParser.lastError);
    }
}

static void app_ipd_parser_reset(void)
{
    s_ipdParser.state = APP_IPD_STATE_IDLE;
    s_ipdParser.matchIndex = 0U;
    s_ipdParser.remaining = 0U;
}

static uint8_t app_esp_process_rx_byte(APP_NodeContext* node, uint8_t byte)
{
    if (node == 0)
    {
        return 1U;
    }

    switch (s_ipdParser.state)
    {
        case APP_IPD_STATE_IDLE:
            if (byte == (uint8_t)s_ipdToken[0])
            {
                s_ipdParser.state = APP_IPD_STATE_MATCH;
                s_ipdParser.matchIndex = 1U;
                return 0U;
            }
            return 1U;

        case APP_IPD_STATE_MATCH:
            if (byte == (uint8_t)s_ipdToken[s_ipdParser.matchIndex])
            {
                s_ipdParser.matchIndex++;
                if (s_ipdParser.matchIndex >= ((uint8_t)sizeof(s_ipdToken) - 1U))
                {
                    s_ipdParser.state = APP_IPD_STATE_LENGTH;
                    s_ipdParser.remaining = 0U;
                }
                return 0U;
            }

            app_ipd_parser_reset();
            if (byte == (uint8_t)s_ipdToken[0])
            {
                s_ipdParser.state = APP_IPD_STATE_MATCH;
                s_ipdParser.matchIndex = 1U;
                return 0U;
            }
            return 1U;

        case APP_IPD_STATE_LENGTH:
            if ((byte >= (uint8_t)'0') && (byte <= (uint8_t)'9'))
            {
                s_ipdParser.remaining = (uint16_t)((s_ipdParser.remaining * 10U) + (uint16_t)(byte - (uint8_t)'0'));
                return 0U;
            }

            if (byte == (uint8_t)':')
            {
                if (s_ipdParser.remaining == 0U)
                {
                    app_ipd_parser_reset();
                }
                else
                {
                    s_ipdParser.state = APP_IPD_STATE_DATA;
                    APP_HomeProtocol_InitParser(&s_protocolParser);
                }
                return 0U;
            }

            app_ipd_parser_reset();
            return 1U;

        case APP_IPD_STATE_DATA:
            app_feed_protocol_byte(node, byte);
            if (s_ipdParser.remaining > 0U)
            {
                s_ipdParser.remaining--;
            }
            if (s_ipdParser.remaining == 0U)
            {
                app_ipd_parser_reset();
            }
            return 0U;

        default:
            app_ipd_parser_reset();
            return 1U;
    }
}

static uint16_t app_build_telemetry_frame(APP_NodeContext* node, uint8_t* output, uint16_t outputSize)
{
    uint8_t payload[APP_HOME_TELEMETRY_PAYLOAD_LEN];
    int16_t temperatureX10;
    uint16_t humidityX10;
    uint16_t sequence;
    uint16_t frameLength;

    if ((node == 0) || (output == 0))
    {
        return 0U;
    }

    temperatureX10 = (int16_t)(((int16_t)node->dht11.temperatureInt * 10) + (int16_t)node->dht11.temperatureDec);
    humidityX10 = (uint16_t)(((uint16_t)node->dht11.humidityInt * 10U) + (uint16_t)node->dht11.humidityDec);

    app_write_le16(&payload[0], (uint16_t)temperatureX10);
    app_write_le16(&payload[2], humidityX10);
    payload[4] = s_controlMode;
    payload[5] = s_controlFan;
    payload[6] = 1U;
    payload[7] = app_current_relay_flags();

    sequence = app_next_protocol_sequence();
    frameLength = APP_HomeProtocol_BuildFrame(APP_NODE_ID,
                                              APP_HOME_CMD_TELEMETRY,
                                              sequence,
                                              payload,
                                              (uint16_t)sizeof(payload),
                                              output,
                                              outputSize);
    if (frameLength != 0U)
    {
        app_debug_log("[PROTO] telemetry tx seq=%u temp_x10=%d hum_x10=%u mode=%u fan=%u flags=0x%02X",
                      (unsigned int)sequence,
                      (int)temperatureX10,
                      (unsigned int)humidityX10,
                      (unsigned int)s_controlMode,
                      (unsigned int)s_controlFan,
                      (unsigned int)payload[7]);
    }

    return frameLength;
}

static void app_prepare_pending_reply_payload(void)
{
    if ((s_pendingReplyKind == APP_SEND_KIND_NONE) || (s_pendingReplyLength == 0U))
    {
        return;
    }

    memcpy(s_uploadPayload, s_pendingReplyFrame, s_pendingReplyLength);
    s_uploadPayloadLength = s_pendingReplyLength;
    s_uploadPayloadKind = s_pendingReplyKind;
}

static uint8_t app_upload_payload_is_protocol_reply(void)
{
    return ((s_uploadPayloadKind == APP_SEND_KIND_ACK) ||
            (s_uploadPayloadKind == APP_SEND_KIND_ERR)) ? 1U : 0U;
}

static void app_mark_current_send_success(APP_NodeContext* node, uint32_t nowTick)
{
    if (app_upload_payload_is_protocol_reply() != 0U)
    {
        if (s_stateUploadPending != 0U)
        {
            uint32_t guardedReadyTick = nowTick + APP_PROTOCOL_REPLY_UPLOAD_GUARD_MS;
            if ((int32_t)(s_stateUploadReadyTick - guardedReadyTick) < 0)
            {
                s_stateUploadReadyTick = guardedReadyTick;
                app_debug_log("[NET] defer state upload after reply %ums",
                              (unsigned int)APP_PROTOCOL_REPLY_UPLOAD_GUARD_MS);
            }
        }

        s_pendingReplyLength = 0U;
        s_pendingReplyKind = APP_SEND_KIND_NONE;
        app_debug_log("[NET] protocol reply ok");
    }
    else
    {
        app_debug_log("[NET] telemetry ok");
        if ((s_uploadPayloadStateGeneration != 0U) &&
            (s_uploadPayloadStateGeneration == s_stateUploadGeneration))
        {
            s_stateUploadPending = 0U;
            s_stateUploadReadyTick = 0U;
            app_debug_log("[NET] state upload synced");
        }
    }

    s_uploadPayloadKind = APP_SEND_KIND_NONE;
    s_uploadPayloadStateGeneration = 0U;
    app_mark_upload_success(node, nowTick);
}

static void app_mark_current_send_failure(APP_NodeContext* node, uint32_t nowTick)
{
    s_uploadPayloadKind = APP_SEND_KIND_NONE;
    s_uploadPayloadStateGeneration = 0U;
    s_payloadSentAfterMissingPrompt = 0U;
    app_mark_upload_failure(node, nowTick);
}

static void app_finish_upload_send_success_peer_closed(APP_NodeContext* node, uint32_t nowTick, const char* reason)
{
    if ((reason != 0) && (reason[0] != '\0'))
    {
        app_debug_log("%s", reason);
    }

    s_payloadSentAfterMissingPrompt = 0U;
    s_tcpConnected = 0U;
    s_tcpReopenNotBeforeTick = nowTick + APP_ESP_TCP_REOPEN_SETTLE_MS;
    app_esp_exchange_reset();
    app_mark_current_send_success(node, nowTick);
    s_espState = APP_ESP_STATE_READY;
}

static void app_finish_upload_send_success_with_link(APP_NodeContext* node, uint32_t nowTick, uint8_t tcpConnected)
{
    s_payloadSentAfterMissingPrompt = 0U;
    s_tcpConnected = tcpConnected;
    app_esp_exchange_reset();
    app_mark_current_send_success(node, nowTick);
    s_espState = APP_ESP_STATE_READY;
}

static void app_finish_upload_send_success(APP_NodeContext* node, uint32_t nowTick)
{
    app_finish_upload_send_success_with_link(node, nowTick, 1U);
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

static HAL_StatusTypeDef app_post_relay_test(void)
{
    uint32_t relayIndex;
    HAL_StatusTypeDef status;

    for (relayIndex = 0U; relayIndex < 3U; relayIndex++)
    {
        s_relayState[relayIndex] = 1U;
        status = app_apply_relay_state((uint8_t)relayIndex);
        if (status != HAL_OK)
        {
            return status;
        }

        HAL_Delay(APP_POST_RELAY_TEST_MS);

        s_relayState[relayIndex] = 0U;
        status = app_apply_relay_state((uint8_t)relayIndex);
        if (status != HAL_OK)
        {
            return status;
        }

        HAL_Delay(20U);
    }

    return HAL_OK;
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

static void app_esp_rx_discard_slice(APP_NodeContext* node, uint16_t maxBytes)
{
    uint16_t index;
    uint8_t dummyByte;

    if (node == 0)
    {
        return;
    }

    for (index = 0U; index < maxBytes; index++)
    {
        if (BSP_ESP01S_Receive(&node->esp, &dummyByte, 1U, 0U) != HAL_OK)
        {
            break;
        }
        (void)app_esp_process_rx_byte(node, dummyByte);
    }
}

static uint8_t app_esp_collect_response_slice(APP_NodeContext* node,
                                              APP_EspExchangeContext* exchange)
{
    uint8_t rxByte;
    uint8_t appendToResponse;
    uint16_t readCount = 0U;
    HAL_StatusTypeDef status;

    if (node == 0)
    {
        return 0U;
    }

    while (readCount < APP_ESP_RX_SLICE_BYTES)
    {
        status = BSP_ESP01S_Receive(&node->esp, &rxByte, 1U, 0U);
        if (status != HAL_OK)
        {
            break;
        }

        appendToResponse = app_esp_process_rx_byte(node, rxByte);
        if ((appendToResponse != 0U) &&
            (exchange != 0) &&
            (exchange->responseLength < (APP_ESP_RESPONSE_SIZE - 1U)))
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

    if (strstr(response, "ERRO") != 0)
    {
        return 1U;
    }

    if (strstr(response, "EROR") != 0)
    {
        return 1U;
    }

    if (strstr(response, "RROR") != 0)
    {
        return 1U;
    }

    if (strstr(response, "ERR") != 0)
    {
        return 1U;
    }

    if (strstr(response, "FAIL") != 0)
    {
        return 1U;
    }

    if (strstr(response, "Fail") != 0)
    {
        return 1U;
    }

    if (strstr(response, "fail") != 0)
    {
        return 1U;
    }

    if (strstr(response, "CLOSED") != 0)
    {
        return 1U;
    }

    if (strstr(response, "CLOSE") != 0)
    {
        return 1U;
    }

    if (strstr(response, "CLOS") != 0)
    {
        return 1U;
    }

    if (strstr(response, "LOSE") != 0)
    {
        return 1U;
    }

    if (strstr(response, "link is not valid") != 0)
    {
        return 1U;
    }

    if (strstr(response, "link is not alid") != 0)
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
            app_esp_rx_discard_slice(node, (uint16_t)(APP_ESP_RX_SLICE_BYTES * 3U));
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
    rxUpdated = app_esp_collect_response_slice(node, &s_espExchange);

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

    if ((strstr(response, "Recv ") == 0) &&
        (strstr(response, "Recv") == 0))
    {
        return 0U;
    }

    if (app_esp_response_has_error(response) != 0U)
    {
        return 0U;
    }

    return 1U;
}

static uint8_t app_esp_send_recv_seen(const char* response)
{
    if (response == 0)
    {
        return 0U;
    }

    return ((strstr(response, "Recv ") != 0) ||
            (strstr(response, "Recv") != 0)) ? 1U : 0U;
}

static uint8_t app_esp_response_has_close(const char* response)
{
    if (response == 0)
    {
        return 0U;
    }

    return ((strstr(response, "CLOSED") != 0) ||
            (strstr(response, "CLOSE") != 0) ||
            (strstr(response, "CLOS") != 0) ||
            (strstr(response, "LOSE") != 0)) ? 1U : 0U;
}

static uint8_t app_esp_send_ok_likely(const char* response)
{
    if ((response == 0) || (response[0] == '\0'))
    {
        return 0U;
    }

    if ((strstr(response, "SEND OK") != 0) ||
        (strstr(response, "SEN OK") != 0) ||
        (strstr(response, "SND OK") != 0) ||
        (strstr(response, "SEND O") != 0))
    {
        return 1U;
    }

    return 0U;
}

static uint8_t app_esp_cipstart_likely_connected(const char* response)
{
    if ((response == 0) || (response[0] == '\0'))
    {
        return 0U;
    }

    if (app_esp_response_has_error(response) != 0U)
    {
        return 0U;
    }

    if ((strstr(response, "ALREADY CONNECTED") != 0) ||
        (strstr(response, "ALREADY") != 0) ||
        (strstr(response, "AREADY") != 0) ||
        (strstr(response, "ALRADY") != 0) ||
        (strstr(response, "ALEADY") != 0) ||
        (strstr(response, "CONNECT") != 0) ||
        (strstr(response, "CNNECT") != 0) ||
        (strstr(response, "CONEC") != 0) ||
        (strstr(response, "ONNCT") != 0) ||
        (strstr(response, "CON") != 0))
    {
        return 1U;
    }

    return 0U;
}

static uint8_t app_esp_cifsr_likely_ok(const char* response)
{
    if ((response == 0) || (response[0] == '\0'))
    {
        return 0U;
    }

    if (app_esp_response_has_error(response) != 0U)
    {
        return 0U;
    }

    if (strstr(response, "STAIP,\"0.0.0.0\"") != 0)
    {
        return 0U;
    }

    if ((strstr(response, "CIFSR:STAIP") != 0) ||
        (strstr(response, "STAIP") != 0))
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

    if ((strstr(response, "WIFI CONNECTED") != 0) ||
        (strstr(response, "WIFI CONNECTD") != 0) ||
        (strstr(response, "WIFI CONNET") != 0) ||
        (strstr(response, "WIFI CONECTE") != 0) ||
        (strstr(response, "WIFI CONECTD") != 0) ||
        (strstr(response, "WIFI CONN") != 0) ||
        (strstr(response, "WIFI CONNE") != 0) ||
        (strstr(response, "WIFI CO") != 0) ||
        (strstr(response, "WIFI C") != 0) ||
        (strstr(response, "WIFI ONNET") != 0) ||
        (strstr(response, "WIFI ONN") != 0) ||
        (strstr(response, "ONCTD") != 0) ||
        (strstr(response, "ONNCTD") != 0) ||
        (strstr(response, "ONNECTD") != 0) ||
        (strstr(response, "WIF CO") != 0) ||
        (strstr(response, "WIF CONN") != 0) ||
        (strstr(response, " CONNCT") != 0) ||
        (strstr(response, " CONECT") != 0))
    {
        hasConnected = 1U;
    }

    if ((strstr(response, "WIFI GOT IP") != 0) ||
        (strstr(response, "WIFI GOTIP") != 0) ||
        (strstr(response, "WIFIGOT") != 0) ||
        (strstr(response, " GOT IP") != 0) ||
        (strstr(response, " GOTIP") != 0) ||
        (strstr(response, "GOT P") != 0) ||
        (strstr(response, "WII GT I") != 0) ||
        (strstr(response, "WI GT I") != 0) ||
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
    if (hasConnected != 0U)
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
    (void)node;

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
        app_debug_log("[NET] upload fail streak reached %u, keep WiFi and retry TCP later",
                      (unsigned int)APP_UPLOAD_FAIL_REJOIN_THRESHOLD);
        s_uploadFailStreak = 0U;
        s_tcpConnected = 0U;
        s_nextUploadAttemptTick = nowTick + APP_UPLOAD_RETRY_MS;
    }
}

static void app_recover_tcp_without_wifi_rejoin(APP_NodeContext* node, uint32_t nowTick, const char* reason)
{
    if ((reason != 0) && (reason[0] != '\0'))
    {
        app_debug_log("%s", reason);
    }

    s_tcpConnected = 0U;
    s_payloadSentAfterMissingPrompt = 0U;
    s_tcpReopenNotBeforeTick = nowTick + APP_ESP_TCP_REOPEN_SETTLE_MS;
    app_mark_current_send_failure(node, nowTick);
    app_esp_exchange_reset();
    s_espState = APP_ESP_STATE_UPLOAD_RECOVER_CLOSE;
}

static void app_recover_tcp_after_peer_closed(APP_NodeContext* node, uint32_t nowTick, const char* reason)
{
    if ((reason != 0) && (reason[0] != '\0'))
    {
        app_debug_log("%s", reason);
    }

    s_tcpConnected = 0U;
    s_payloadSentAfterMissingPrompt = 0U;
    s_tcpReopenNotBeforeTick = nowTick + APP_ESP_TCP_REOPEN_SETTLE_MS;
    app_mark_current_send_failure(node, nowTick);
    app_esp_exchange_reset();
    s_espState = APP_ESP_STATE_READY;
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

            app_debug_log("[NET] wifi not connected, join AP");
            s_tcpConnected = 0U;
            s_cwjapRetryCount = 0U;
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
                s_espState = APP_ESP_STATE_WIFI_CMD_SLEEP;
            }
            else if (exchangeResult == APP_ESP_EXCHANGE_FAIL)
            {
                app_debug_log("[NET] wifi connect failed: CWMODE");
                s_wifiConnected = 0U;
                s_tcpConnected = 0U;
                s_espState = APP_ESP_STATE_WIFI_WAIT_RETRY;
            }
            break;

        case APP_ESP_STATE_WIFI_CMD_SLEEP:
            exchangeResult = app_esp_step_exchange(node,
                                                   nowTick,
                                                   (uint32_t)APP_ESP_STATE_WIFI_CMD_SLEEP,
                                                   "AT+SLEEP=0\r\n",
                                                   "OK",
                                                   0,
                                                   APP_ESP_CMD_TIMEOUT_MS,
                                                   1U);
            if (exchangeResult == APP_ESP_EXCHANGE_FAIL)
            {
                app_debug_log("[NET] wifi connect warning: SLEEP skipped");
            }

            if (exchangeResult != APP_ESP_EXCHANGE_BUSY)
            {
                s_espState = APP_ESP_STATE_WIFI_CMD_CIPMODE;
            }
            break;

        case APP_ESP_STATE_WIFI_CMD_CIPMODE:
            exchangeResult = app_esp_step_exchange(node,
                                                   nowTick,
                                                   (uint32_t)APP_ESP_STATE_WIFI_CMD_CIPMODE,
                                                   "AT+CIPMODE=0\r\n",
                                                   "OK",
                                                   0,
                                                   APP_ESP_CMD_TIMEOUT_MS,
                                                   1U);
            if (exchangeResult == APP_ESP_EXCHANGE_FAIL)
            {
                app_debug_log("[NET] wifi connect warning: CIPMODE skipped");
            }

            if (exchangeResult != APP_ESP_EXCHANGE_BUSY)
            {
                s_espState = APP_ESP_STATE_WIFI_CMD_CIPMUX;
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

                s_wifiConnected = 0U;
                s_tcpConnected = 0U;
                s_uploadFailStreak = 0U;
                s_cwjapRetryCount = 0U;
                s_cifsrRetryCount = 0U;
                s_espStateDeadlineTick = nowTick + APP_ESP_JOIN_SETTLE_MS;
                app_debug_log("[NET] wait before IP verify");
                s_espState = APP_ESP_STATE_WIFI_JOIN_SETTLE;
            }
            else if (exchangeResult == APP_ESP_EXCHANGE_FAIL)
            {
                if (s_cwjapRetryCount < APP_ESP_CWJAP_RETRY_MAX)
                {
                    s_cwjapRetryCount++;
                    app_debug_log("[NET] wifi join failed, retry CWJAP %u/%u",
                                  (unsigned int)s_cwjapRetryCount,
                                  (unsigned int)APP_ESP_CWJAP_RETRY_MAX);
                    s_espStateDeadlineTick = nowTick + APP_ESP_CWJAP_RETRY_MS;
                    s_espState = APP_ESP_STATE_WIFI_CWJAP_RETRY_WAIT;
                    app_esp_exchange_reset();
                    break;
                }

                app_debug_log("[NET] wifi join retry failed, reset ESP later");
                s_cwjapRetryCount = 0U;
                s_wifiConnected = 0U;
                s_tcpConnected = 0U;
                s_espState = APP_ESP_STATE_WIFI_WAIT_RETRY;
            }
            break;

        case APP_ESP_STATE_WIFI_CWJAP_RETRY_WAIT:
            (void)app_esp_collect_response_slice(node, 0);
            if (app_tick_reached(nowTick, s_espStateDeadlineTick) == 0U)
            {
                break;
            }

            app_esp_exchange_reset();
            s_espState = APP_ESP_STATE_WIFI_CMD_CWJAP;
            break;

        case APP_ESP_STATE_WIFI_JOIN_SETTLE:
            (void)app_esp_collect_response_slice(node, 0);
            if (app_tick_reached(nowTick, s_espStateDeadlineTick) == 0U)
            {
                break;
            }

            app_esp_exchange_reset();
            s_espState = APP_ESP_STATE_WIFI_CMD_CIFSR;
            break;

        case APP_ESP_STATE_WIFI_CMD_CIFSR:
            /* Confirm the ESP8266 station IP and AT responsiveness after AP join. */
            exchangeResult = app_esp_step_exchange(node,
                                                   nowTick,
                                                   (uint32_t)APP_ESP_STATE_WIFI_CMD_CIFSR,
                                                   "AT+CIFSR\r\n",
                                                   "OK",
                                                   0,
                                                   APP_ESP_TCP_TIMEOUT_MS,
                                                   1U);
            if ((exchangeResult == APP_ESP_EXCHANGE_OK) ||
                ((exchangeResult == APP_ESP_EXCHANGE_FAIL) &&
                 (app_esp_cifsr_likely_ok(s_espExchange.response) != 0U)))
            {
                if (exchangeResult == APP_ESP_EXCHANGE_FAIL)
                {
                    app_debug_log("[NET] CIFSR truncated, accept IP");
                }
                s_wifiConnected = 1U;
                s_tcpConnected = 0U;
                s_cifsrRetryCount = 0U;
                s_nextUploadAttemptTick = nowTick + APP_ESP_POST_IP_UPLOAD_DELAY_MS;
                app_debug_log("[NET] wifi ip confirmed, wait TCP stack settle %ums",
                              (unsigned int)APP_ESP_POST_IP_UPLOAD_DELAY_MS);
                s_espState = APP_ESP_STATE_WIFI_CMD_CIPSTATUS;
            }
            else if (exchangeResult == APP_ESP_EXCHANGE_FAIL)
            {
                if (s_cifsrRetryCount < APP_ESP_CIFSR_RETRY_MAX)
                {
                    s_cifsrRetryCount++;
                    app_debug_log("[NET] CIFSR verify pending, retry %u/%u",
                                  (unsigned int)s_cifsrRetryCount,
                                  (unsigned int)APP_ESP_CIFSR_RETRY_MAX);
                    s_espStateDeadlineTick = nowTick + APP_ESP_CIFSR_RETRY_MS;
                    s_espState = APP_ESP_STATE_WIFI_CIFSR_RETRY_WAIT;
                    break;
                }

                app_debug_log("[NET] CIFSR verify failed, wait before WiFi retry");
                s_cifsrRetryCount = 0U;
                s_wifiConnected = 0U;
                s_tcpConnected = 0U;
                s_lastWifiAttemptTick = nowTick;
                s_espState = APP_ESP_STATE_WIFI_WAIT_RETRY;
            }
            break;

        case APP_ESP_STATE_WIFI_CIFSR_RETRY_WAIT:
            (void)app_esp_collect_response_slice(node, 0);
            if (app_tick_reached(nowTick, s_espStateDeadlineTick) == 0U)
            {
                break;
            }

            app_esp_exchange_reset();
            s_espState = APP_ESP_STATE_WIFI_CMD_CIFSR;
            break;

        case APP_ESP_STATE_WIFI_CMD_CIPSTATUS:
            /* Diagnostic only: report ESP TCP/IP state before the first CIPSTART. */
            exchangeResult = app_esp_step_exchange(node,
                                                   nowTick,
                                                   (uint32_t)APP_ESP_STATE_WIFI_CMD_CIPSTATUS,
                                                   "AT+CIPSTATUS\r\n",
                                                   "OK",
                                                   0,
                                                   APP_ESP_CMD_TIMEOUT_MS,
                                                   1U);
            if (exchangeResult == APP_ESP_EXCHANGE_OK)
            {
                s_espState = APP_ESP_STATE_READY;
            }
            else if (exchangeResult == APP_ESP_EXCHANGE_FAIL)
            {
                app_debug_log("[NET] CIPSTATUS diagnostic failed, continue");
                s_espState = APP_ESP_STATE_READY;
            }
            break;

        case APP_ESP_STATE_READY:
            if (s_wifiConnected == 0U)
            {
                s_tcpConnected = 0U;
                s_espState = APP_ESP_STATE_WIFI_WAIT_RETRY;
                break;
            }

            (void)app_esp_collect_response_slice(node, 0);

            if (s_uploadConfigured == 0U)
            {
                break;
            }

            if ((s_tcpConnected == 0U) &&
                (app_tick_reached(nowTick, s_tcpReopenNotBeforeTick) == 0U))
            {
                break;
            }

            if (s_pendingReplyKind != APP_SEND_KIND_NONE)
            {
                app_prepare_pending_reply_payload();
            }
            else
            {
                if (s_stateUploadPending != 0U)
                {
                    if (app_tick_reached(nowTick, s_stateUploadReadyTick) == 0U)
                    {
                        break;
                    }

                    if ((s_lastUploadOk == 0U) &&
                        (app_tick_reached(nowTick, s_nextUploadAttemptTick) == 0U))
                    {
                        break;
                    }

                    s_uploadPayloadStateGeneration = s_stateUploadGeneration;
                }
                else
                {
                    if (app_tick_reached(nowTick, s_nextUploadAttemptTick) == 0U)
                    {
                        break;
                    }

                    s_uploadPayloadStateGeneration = 0U;
                }

                payloadLength = (int)app_build_telemetry_frame(node,
                                                               s_uploadPayload,
                                                               (uint16_t)sizeof(s_uploadPayload));
                if (payloadLength <= 0)
                {
                    s_uploadPayloadStateGeneration = 0U;
                    app_debug_log("[NET] telemetry build failed");
                    app_mark_upload_failure(node, nowTick);
                    s_espState = (s_wifiConnected != 0U) ? APP_ESP_STATE_READY : APP_ESP_STATE_WIFI_WAIT_RETRY;
                    break;
                }

                s_uploadPayloadLength = (uint16_t)payloadLength;
                s_uploadPayloadKind = APP_SEND_KIND_TELEMETRY;
            }

            /*
             * Keep the socket alive; repeated CIPCLOSE can leave ESP8266 stuck in CLOSE/busy state.
             * AT+PING is unreliable here and can leave a stale ERR before TCP, so open TCP directly.
             */
            s_payloadSentAfterMissingPrompt = 0U;
            s_espState = (s_tcpConnected != 0U) ?
                         APP_ESP_STATE_UPLOAD_CMD_CIPSEND :
                         APP_ESP_STATE_UPLOAD_CMD_CIPSTART;
            app_esp_exchange_reset();
            break;

        case APP_ESP_STATE_UPLOAD_CMD_PING:
            /* Pre-warm ESP8266 ARP/routing cache so CIPSTART can spend its short window on TCP. */
            (void)snprintf(command,
                           sizeof(command),
                           "AT+PING=\"%s\"\r\n",
                           APP_UPLOAD_HOST);
            exchangeResult = app_esp_step_exchange(node,
                                                   nowTick,
                                                   (uint32_t)APP_ESP_STATE_UPLOAD_CMD_PING,
                                                   command,
                                                   "OK",
                                                   0,
                                                   APP_ESP_PING_TIMEOUT_MS,
                                                   1U);
            if (exchangeResult == APP_ESP_EXCHANGE_OK)
            {
                app_debug_log("[NET] host ping ok, start TCP");
                s_espStateDeadlineTick = nowTick + APP_ESP_PRE_TCP_SETTLE_MS;
                s_espState = APP_ESP_STATE_UPLOAD_PING_SETTLE;
                app_esp_exchange_reset();
            }
            else if (exchangeResult == APP_ESP_EXCHANGE_FAIL)
            {
                app_debug_log("[NET] host ping skipped, start TCP");
                s_espStateDeadlineTick = nowTick + APP_ESP_PRE_TCP_SETTLE_MS;
                s_espState = APP_ESP_STATE_UPLOAD_PING_SETTLE;
                app_esp_exchange_reset();
            }
            break;

        case APP_ESP_STATE_UPLOAD_PING_SETTLE:
            (void)app_esp_collect_response_slice(node, 0);
            if (app_tick_reached(nowTick, s_espStateDeadlineTick) == 0U)
            {
                break;
            }

            s_espState = APP_ESP_STATE_UPLOAD_CMD_CIPSTART;
            app_esp_exchange_reset();
            break;

        case APP_ESP_STATE_UPLOAD_CMD_CIPSTART:
            /* Open TCP only when needed; probe CIPSEND if the AT response is ambiguous. */
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
                    app_debug_log("[NET] CIPSTART no final OK, wait then probe CIPSEND");
                    s_tcpConnected = 1U;
                    s_espStateDeadlineTick = nowTick + APP_ESP_CIPSTART_PROBE_DELAY_MS;
                    s_espState = APP_ESP_STATE_UPLOAD_CIPSTART_SETTLE;
                    app_esp_exchange_reset();
                }
                else
                {
                    app_debug_log("[NET] CIPSTART failed, retry TCP in %ums",
                                  (unsigned int)APP_CIPSTART_FAIL_RETRY_MS);
                    s_tcpConnected = 0U;
                    app_mark_current_send_failure(node, nowTick);
                    s_nextUploadAttemptTick = nowTick + APP_CIPSTART_FAIL_RETRY_MS;
                    s_espState = APP_ESP_STATE_READY;
                    app_esp_exchange_reset();
                }
            }
            break;

        case APP_ESP_STATE_UPLOAD_CIPSTART_SETTLE:
            (void)app_esp_collect_response_slice(node, 0);
            if (app_tick_reached(nowTick, s_espStateDeadlineTick) == 0U)
            {
                break;
            }

            s_espState = APP_ESP_STATE_UPLOAD_CMD_CIPSEND;
            app_esp_exchange_reset();
            break;

        case APP_ESP_STATE_UPLOAD_CMD_CIPSEND:
            /* Require the send prompt before binary payload, or the frame may be parsed as AT text. */
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
                s_payloadSentAfterMissingPrompt = 0U;
                s_espState = APP_ESP_STATE_UPLOAD_SEND_PAYLOAD;
            }
            else if (exchangeResult == APP_ESP_EXCHANGE_FAIL)
            {
                if (app_esp_response_has_error(s_espExchange.response) == 0U)
                {
                    app_recover_tcp_without_wifi_rejoin(node,
                                                        nowTick,
                                                        "[NET] CIPSEND prompt timeout, recover TCP");
                }
                else
                {
                    app_debug_log("[NET] CIPSEND prompt failed");
                    app_mark_current_send_failure(node, nowTick);
                    s_espState = APP_ESP_STATE_UPLOAD_RECOVER_CLOSE;
                }
            }
            break;

        case APP_ESP_STATE_UPLOAD_SEND_PAYLOAD:
            if (s_uploadPayloadLength == 0U)
            {
                app_debug_log("[NET] empty payload");
                app_mark_current_send_failure(node, nowTick);
                s_espState = APP_ESP_STATE_UPLOAD_RECOVER_CLOSE;
                break;
            }

            status = BSP_ESP01S_Send(&node->esp,
                                     (const uint8_t*)s_uploadPayload,
                                     s_uploadPayloadLength,
                                     APP_ESP_CMD_TIMEOUT_MS);
            app_debug_log_hex("[NET] tx hex", s_uploadPayload, s_uploadPayloadLength);
            if (status != HAL_OK)
            {
                app_debug_log("[NET] payload tx failed");
                app_mark_current_send_failure(node, nowTick);
                s_espState = APP_ESP_STATE_UPLOAD_RECOVER_CLOSE;
                break;
            }

            s_espState = APP_ESP_STATE_UPLOAD_WAIT_SEND_OK;
            app_esp_exchange_reset();
            break;

        case APP_ESP_STATE_UPLOAD_WAIT_SEND_OK:
            /*
             * "Recv xx bytes" only confirms that ESP accepted UART payload bytes.
             * Wait for SEND OK before keeping the TCP socket for the next CIPSEND.
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
                if (app_upload_payload_is_protocol_reply() != 0U)
                {
                    if (app_esp_response_has_close(s_espExchange.response) != 0U)
                    {
                        app_finish_upload_send_success_peer_closed(node,
                                                                   nowTick,
                                                                   "[NET] protocol reply peer-closed");
                    }
                    else
                    {
                        app_finish_upload_send_success(node, nowTick);
                    }
                    break;
                }

                s_tcpConnected = 1U;
                app_finish_upload_send_success(node, nowTick);
                break;
            }

            if ((exchangeResult == APP_ESP_EXCHANGE_BUSY) &&
                (app_esp_send_ok_likely(s_espExchange.response) != 0U))
            {
                if (app_upload_payload_is_protocol_reply() != 0U)
                {
                    if (app_esp_response_has_close(s_espExchange.response) != 0U)
                    {
                        app_finish_upload_send_success_peer_closed(node,
                                                                   nowTick,
                                                                   "[NET] protocol reply peer-closed");
                    }
                    else
                    {
                        app_finish_upload_send_success(node, nowTick);
                    }
                    break;
                }

                app_esp_exchange_reset();
                s_tcpConnected = 1U;
                app_finish_upload_send_success(node, nowTick);
                break;
            }

            if (exchangeResult == APP_ESP_EXCHANGE_FAIL)
            {
                if (app_esp_send_ok_likely(s_espExchange.response) != 0U)
                {
                    if (app_upload_payload_is_protocol_reply() != 0U)
                    {
                        if (app_esp_response_has_close(s_espExchange.response) != 0U)
                        {
                            app_finish_upload_send_success_peer_closed(node,
                                                                       nowTick,
                                                                       "[NET] protocol reply peer-closed");
                        }
                        else
                        {
                            app_finish_upload_send_success(node, nowTick);
                        }
                        break;
                    }

                    s_tcpConnected = 1U;
                    app_finish_upload_send_success(node, nowTick);
                }
                else if (((s_uploadPayloadKind == APP_SEND_KIND_ACK) ||
                          (s_uploadPayloadKind == APP_SEND_KIND_ERR)) &&
                         (app_esp_send_recv_seen(s_espExchange.response) != 0U))
                {
                    app_debug_log("[NET] protocol reply accepted without SEND OK");
                    if (app_esp_response_has_close(s_espExchange.response) != 0U)
                    {
                        app_finish_upload_send_success_peer_closed(node,
                                                                   nowTick,
                                                                   "[NET] protocol reply peer-closed");
                    }
                    else
                    {
                        /*
                         * ESP acknowledged UART payload ("Recv xx bytes") but did not report SEND OK.
                         * Treat reply as delivered, then force a clean TCP reopen before next payload
                         * to avoid "link is not valid" on immediate follow-up telemetry.
                         */
                        s_payloadSentAfterMissingPrompt = 0U;
                        s_tcpConnected = 0U;
                        s_tcpReopenNotBeforeTick = nowTick + APP_ESP_TCP_REOPEN_SETTLE_MS;
                        app_esp_exchange_reset();
                        app_mark_current_send_success(node, nowTick);
                        s_espState = APP_ESP_STATE_UPLOAD_RECOVER_CLOSE;
                    }
                    break;
                }
                else if (app_esp_send_recv_accepted(s_espExchange.response) != 0U)
                {
                    app_recover_tcp_without_wifi_rejoin(node,
                                                        nowTick,
                                                        "[NET] send accepted without SEND OK, recover TCP");
                }
                else if (app_esp_response_has_error(s_espExchange.response) == 0U)
                {
                    if (s_payloadSentAfterMissingPrompt != 0U)
                    {
                        app_recover_tcp_without_wifi_rejoin(node,
                                                            nowTick,
                                                            "[NET] send ack timeout after missing prompt, recover TCP");
                        break;
                    }

                    app_recover_tcp_without_wifi_rejoin(node,
                                                        nowTick,
                                                        "[NET] send ack timeout, recover TCP");
                }
                else
                {
                    if (app_esp_response_has_close(s_espExchange.response) != 0U)
                    {
                        app_recover_tcp_after_peer_closed(node,
                                                          nowTick,
                                                          "[NET] send result peer-closed, recover TCP");
                    }
                    else
                    {
                        app_debug_log("[NET] send result not ok");
                        app_mark_current_send_failure(node, nowTick);
                        s_espState = APP_ESP_STATE_UPLOAD_RECOVER_CLOSE;
                    }
                }
            }
            break;

        case APP_ESP_STATE_UPLOAD_CMD_CLOSE:
            (void)app_esp_collect_response_slice(node, 0);
            if (app_tick_reached(nowTick, s_espStateDeadlineTick) == 0U)
            {
                break;
            }

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
            s_tcpReopenNotBeforeTick = nowTick + APP_ESP_TCP_REOPEN_SETTLE_MS;
            app_mark_current_send_success(node, nowTick);
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
            s_tcpReopenNotBeforeTick = nowTick + APP_ESP_TCP_REOPEN_SETTLE_MS;
            s_espState = (s_wifiConnected != 0U) ? APP_ESP_STATE_READY : APP_ESP_STATE_WIFI_WAIT_RETRY;
            break;

        default:
            s_espState = APP_ESP_STATE_IDLE;
            app_esp_exchange_reset();
            break;
    }

    app_update_runtime_view(node);
}

static void app_self_test_mark(APP_NodeContext* node, APP_SelfTestItem item, HAL_StatusTypeDef status)
{
    if (node == 0)
    {
        return;
    }

    if (status == HAL_OK)
    {
        node->selfTest.passMask |= (uint32_t)item;
    }
    else
    {
        node->selfTest.failMask |= (uint32_t)item;
    }
}

static uint32_t app_post_required_mask(void)
{
    return (uint32_t)APP_POST_ITEM_RELAY;
}

static uint32_t app_self_test_required_mask(void)
{
    return (uint32_t)APP_SELF_TEST_ITEM_RELAY1 |
           (uint32_t)APP_SELF_TEST_ITEM_RELAY2 |
           (uint32_t)APP_SELF_TEST_ITEM_RELAY3;
}

static uint8_t app_test_result_passed(const APP_TestResult* result, uint32_t requiredMask)
{
    if (result == 0)
    {
        return 0U;
    }

    if (result->done == 0U)
    {
        return 0U;
    }

    if ((result->failMask & requiredMask) != 0U)
    {
        return 0U;
    }

    return ((result->passMask & requiredMask) == requiredMask) ? 1U : 0U;
}

static void app_log_test_summary(const char* name, const APP_TestResult* result, uint32_t requiredMask)
{
    if ((name == 0) || (result == 0))
    {
        return;
    }

    app_debug_log("[%s] done pass=%lu fail=%lu all=%u",
                  name,
                  (unsigned long)result->passMask,
                  (unsigned long)result->failMask,
                  (unsigned int)app_test_result_passed(result, requiredMask));
}

static HAL_StatusTypeDef app_self_test_relay(uint8_t relayIndex)
{
    HAL_StatusTypeDef status;

    app_debug_log("[SELF] relay%u on", (unsigned int)(relayIndex + 1U));
    s_relayState[relayIndex] = 1U;
    status = app_apply_relay_state(relayIndex);
    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(APP_SELF_TEST_RELAY_ON_MS);

    app_debug_log("[SELF] relay%u off", (unsigned int)(relayIndex + 1U));
    s_relayState[relayIndex] = 0U;
    status = app_apply_relay_state(relayIndex);
    HAL_Delay(APP_SELF_TEST_RELAY_GAP_MS);
    return status;
}

void APP_Node_RunHardwareSelfTest(APP_NodeContext* node)
{
    static const APP_SelfTestItem relayItems[3] =
    {
        APP_SELF_TEST_ITEM_RELAY1,
        APP_SELF_TEST_ITEM_RELAY2,
        APP_SELF_TEST_ITEM_RELAY3
    };
    uint32_t relayIndex;
    HAL_StatusTypeDef status;

    if (node == 0)
    {
        return;
    }

    node->selfTest.passMask = 0U;
    node->selfTest.failMask = 0U;
    node->selfTest.done = 0U;

    app_set_all_relay_states(0U, "self begin off");
    app_debug_log("[SELF] hardware self-test begin");
    for (relayIndex = 0U; relayIndex < 3U; relayIndex++)
    {
        status = app_self_test_relay((uint8_t)relayIndex);
        app_self_test_mark(node, relayItems[relayIndex], status);
    }

    app_set_all_relay_states(0U, "self final off");
    node->selfTest.done = 1U;
    app_log_test_summary("SELF", &node->selfTest, app_self_test_required_mask());
}

static void app_capture_button_baseline(uint32_t nowTick)
{
    uint32_t index;

    for (index = 0U; index < APP_BUTTON_COUNT; index++)
    {
        s_buttons[index].stableState = app_read_button(s_buttons[index].pin);
        s_buttons[index].lastSample = s_buttons[index].stableState;
        s_buttons[index].lastChangeTick = nowTick;
    }
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

    app_post_mark(node, APP_POST_ITEM_RELAY, app_post_relay_test());

    app_set_all_relay_states(0U, "post final off");
    node->post.done = 1U;
    app_log_test_summary("POST", &node->post, app_post_required_mask());
}

uint8_t APP_Node_IsPostPassed(const APP_NodeContext* node)
{
    if (node == 0)
    {
        return 0U;
    }

    return app_test_result_passed(&node->post, app_post_required_mask());
}

uint8_t APP_Node_IsSelfTestPassed(const APP_NodeContext* node)
{
    if (node == 0)
    {
        return 0U;
    }

    return app_test_result_passed(&node->selfTest, app_self_test_required_mask());
}

const APP_PostResult* APP_Node_GetPostResult(const APP_NodeContext* node)
{
    if (node == 0)
    {
        return 0;
    }

    return &node->post;
}

const APP_SelfTestResult* APP_Node_GetSelfTestResult(const APP_NodeContext* node)
{
    if (node == 0)
    {
        return 0;
    }

    return &node->selfTest;
}

void APP_Node_SetDebugUart(UART_HandleTypeDef* huart)
{
    s_debugUart = huart;
}

HAL_StatusTypeDef APP_Node_Init(APP_NodeContext* node)
{
    HAL_StatusTypeDef status;
    uint32_t nowTick;

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
    app_set_all_relay_states(0U, "init off");
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
    s_tcpReopenNotBeforeTick = 0U;
    s_cwjapRetryCount = 0U;
    s_cifsrRetryCount = 0U;
    s_protocolTxSequence = 1U;
    s_uploadPayloadKind = APP_SEND_KIND_NONE;
    s_stateUploadPending = 0U;
    s_stateUploadReadyTick = 0U;
    s_stateUploadGeneration = 0U;
    s_uploadPayloadStateGeneration = 0U;
    s_payloadSentAfterMissingPrompt = 0U;
    s_pendingReplyLength = 0U;
    s_pendingReplyKind = APP_SEND_KIND_NONE;
    s_targetTemperatureX10 = 240;
    s_controlMode = APP_HOME_MODE_AUTO;
    s_controlFan = APP_HOME_FAN_MED;
    app_ipd_parser_reset();
    APP_HomeProtocol_InitParser(&s_protocolParser);
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
    app_capture_button_baseline(nowTick);
    s_espState = APP_ESP_STATE_IDLE;
    s_espStateDeadlineTick = nowTick;
    app_esp_exchange_reset();
    s_uploadPayloadLength = 0U;
    s_uploadPayload[0] = '\0';

#if APP_HW_SELF_TEST_ON_BOOT
    APP_Node_RunHardwareSelfTest(node);
#endif

    nowTick = HAL_GetTick();
    app_capture_button_baseline(nowTick);
    /* Start Wi-Fi immediately after self-test; defer the first DHT read. */
    s_lastDhtPollTick = nowTick;
    s_lastWifiAttemptTick = nowTick - APP_WIFI_RETRY_MS;
    node->lastTelemetryTick = nowTick - APP_UPLOAD_INTERVAL_MS;
    s_nextUploadAttemptTick = nowTick;
    app_debug_log("[NET] cfg ssid=\"%s\" passLen=%u host=\"%s\" port=%lu",
                  APP_WIFI_SSID,
                  (unsigned int)strlen(APP_WIFI_PASSWORD),
                  APP_UPLOAD_HOST,
                  (unsigned long)APP_UPLOAD_PORT);
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
        app_debug_log("[KEY] PA0 toggle relay1 -> %u", (unsigned int)s_relayState[0]);
        (void)app_apply_relay_state(0U);
        app_schedule_state_upload(nowTick, "relay1");
    }

    if (app_button_pressed_event(&s_buttons[1], nowTick) != 0U)
    {
        s_relayState[1] ^= 1U;
        app_debug_log("[KEY] PA1 toggle relay2 -> %u", (unsigned int)s_relayState[1]);
        (void)app_apply_relay_state(1U);
        app_schedule_state_upload(nowTick, "relay2");
    }

    if (app_button_pressed_event(&s_buttons[2], nowTick) != 0U)
    {
        s_relayState[2] ^= 1U;
        app_debug_log("[KEY] PA2 toggle relay3 -> %u", (unsigned int)s_relayState[2]);
        (void)app_apply_relay_state(2U);
        app_schedule_state_upload(nowTick, "relay3");
    }

    if (app_button_pressed_event(&s_buttons[3], nowTick) != 0U)
    {
        s_fanEnabled ^= 1U;
        if (s_fanEnabled == 0U)
        {
            s_controlFan = APP_HOME_FAN_OFF;
        }
        else if (s_controlFan == APP_HOME_FAN_OFF)
        {
            s_controlFan = APP_HOME_FAN_AUTO;
            s_fanDutyPercent = APP_HOME_FAN_DUTY_AUTO;
        }
        app_debug_log("[KEY] PA3 toggle fan -> %u", (unsigned int)s_fanEnabled);
        app_apply_fan_state();
        app_schedule_state_upload(nowTick, "fan");
    }

    node->fanDutyPercent = s_fanDutyPercent;
    app_service_esp(node, nowTick);
    app_poll_dht11(node, nowTick);
}

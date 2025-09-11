#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "driver/uart.h"
#include "cJSON.h"
#include <inttypes.h>
#include "config.h"

#define TAG "BRIDGE"

// ===== Globals =====
static QueueHandle_t q_to_pi = NULL;  // to Raspberry Pi
static QueueHandle_t q_to_esp = NULL; // back to readers
static QueueHandle_t q_acks = NULL;   // req_id acks from Pi
static SemaphoreHandle_t peer_mutex = NULL;
static peer_state_t peers[peer_arr_length];
static uint32_t g_req_seq = 1;

// ===== Utilities =====
static void deob(char *s)
{ // function that deobfusicate the UID
    uint8_t key = (uint8_t)(SECRET_KEY & 0xFF);
    for (size_t i = 0; i < strlen(s); i++)
        s[i] ^= key;
}
static bool mac_equal(const uint8_t *a, const uint8_t *b)
{
    for (int i = 0; i < 6; i++)
    {
        if (a[i] != b[i])
        {
            return false;
        }
    }
    return true;
}

static int peer_find(const uint8_t *mac)
{
    for (int i = 0; i < peer_arr_length; i++)
        if (peers[i].in_use && mac_equal(peers[i].mac, mac))
            return i;
    return -1;
}
static int peer_get_or_add(const uint8_t *mac)
{
    int i = peer_find(mac);
    if (i >= 0)
        return i;
    for (int k = 0; k < peer_arr_length; k++)
    {
        if (!peers[k].in_use)
        {
            memcpy(peers[k].mac, mac, 6);
            peers[k].first_char = 0;
            peers[k].in_use = true;
            return k;
        }
    }
    return -1;
}

// Convert MAC to string and back (for JSON readability)
static void mac_to_str(const uint8_t mac[6], char out[18])
{
    snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}
static bool str_to_mac(const char *s, uint8_t mac[6])
{
    unsigned v[6];
    if (sscanf(s, "%2x:%2x:%2x:%2x:%2x:%2x",
               &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6)
        return false;
    for (int i = 0; i < 6; i++)
        mac[i] = (uint8_t)v[i];
    return true;
}

// ===== UART helpers: JSON Lines =====
static void uart_send_json(cJSON *obj)
{
    char *text = cJSON_PrintUnformatted(obj); // heap
    if (!text)
        return;
    uart_write_bytes(UART_PORT, text, strlen(text));
    uart_write_bytes(UART_PORT, "\n", 1);
    cJSON_free(text);
}
static bool uart_read_line(char *buf, size_t maxlen)
{
    size_t idx = 0;
    while (idx < maxlen - 1)
    {
        uint8_t c;
        int n = uart_read_bytes(UART_PORT, &c, 1, pdMS_TO_TICKS(50));
        if (n == 1)
        {
            if (c == '\n')
            {
                buf[idx] = 0;
                return true;
            }
            buf[idx++] = (char)c;
        }
        else
        {
            // timeout slice; let other tasks run
            taskYIELD();
        }
    }
    buf[maxlen - 1] = 0;
    return true; // line truncated, still try
}

// ===== ESP-NOW callbacks =====
static esp_err_t ensure_peer_added(const uint8_t *mac)
{
    esp_now_peer_info_t p = {0};
    memcpy(p.peer_addr, mac, 6);
    p.channel = WIFI_CHANNEL;
    p.encrypt = false;
    esp_err_t e = esp_now_add_peer(&p);
    if (e == ESP_ERR_ESPNOW_EXIST)
        return ESP_OK;
    return e;
}

static void espnow_recv_cb(const esp_now_recv_info_t *info,
                           const uint8_t *data, int len)
{
    if (!info || len < (int)sizeof(rfid_msg_t))
        return;

    rfid_msg_t m;
    memcpy(&m, data, sizeof(m));

    // deobfuscate into a temp buffer
    char plain[64];
    memset(plain, 0, sizeof(plain));
    strncpy(plain, m.uid, sizeof(plain) - 1);
    deob(plain);

    // remember first char per MAC
    xSemaphoreTake(peer_mutex, portMAX_DELAY);
    int idx = peer_get_or_add(info->src_addr);
    if (idx >= 0)
        peers[idx].first_char = (uint8_t)plain[0];
    xSemaphoreGive(peer_mutex);

    // ESP_LOGI(TAG,
    //      "RX from %02X:%02X:%02X:%02X:%02X:%02X len=%d req_seq(next)=%" PRIu32,
    //      info->src_addr[0], info->src_addr[1], info->src_addr[2],
    //      info->src_addr[3], info->src_addr[4], info->src_addr[5],
    //      len, g_req_seq + 1);
    // ESP_LOGI(TAG, "UID(deob): '%s'", plain);

    // enqueue to Pi
    to_pi_t item = {0};
    item.req_id = __atomic_add_fetch(&g_req_seq, 1, __ATOMIC_RELAXED);
    memcpy(item.mac, info->src_addr, 6);
    strncpy(item.uid, plain, sizeof(item.uid) - 1);
    item.attempts = 0;
    xQueueSend(q_to_pi, &item, 0);
}

static void espnow_sent_cb(const uint8_t *mac, esp_now_send_status_t st)
{
    (void)mac;
    (void)st; // optional logging
}

// ===== Tasks =====
static void uart_tx_task(void *arg)
{
    (void)arg;
    to_pi_t front;
    for (;;)
    {
        // block until at least one message exists
        xQueuePeek(q_to_pi, &front, portMAX_DELAY);

        // send JSON: {"t":"req","id":123,"mac":"AA:..","uid":"..."}
        cJSON *root = cJSON_CreateObject();
        char macs[18];
        mac_to_str(front.mac, macs);
        cJSON_AddStringToObject(root, "t", "req");
        cJSON_AddNumberToObject(root, "id", (double)front.req_id);
        cJSON_AddStringToObject(root, "mac", macs);
        cJSON_AddStringToObject(root, "uid", front.uid);
        uart_send_json(root);
        cJSON_Delete(root);

        // wait for ack or timeout, retry up to MAX_RETRIES
        uint32_t ack_id = 0;
        if (xQueueReceive(q_acks, &ack_id, pdMS_TO_TICKS(ACK_WAIT_MS)))
        {
            if (ack_id == front.req_id)
            {
                // pop it for real
                to_pi_t tmp;
                xQueueReceive(q_to_pi, &tmp, 0);
                continue;
            }
        }
        // timed out or wrong ack; retry:
        to_pi_t tmp;
        xQueuePeek(q_to_pi, &tmp, 0);
        tmp.attempts++;
        if (tmp.attempts > MAX_RETRIES)
        {
            ESP_LOGW(TAG, "REQ %" PRIu32 " dropped after retries", (uint32_t)tmp.req_id);
            xQueueReceive(q_to_pi, &tmp, 0); // drop
        }
        else
        {
            xQueueReceive(q_to_pi, &tmp, 0); // remove then requeue at tail
            xQueueSend(q_to_pi, &tmp, 0);
        }
    }
}

static void uart_rx_task(void *arg)
{
    (void)arg;
    char line[JSON_LINE_MAX];
    for (;;)
    {
        if (!uart_read_line(line, sizeof(line)))
            continue;
        if (line[0] == 0)
            continue;

        cJSON *root = cJSON_Parse(line);
        if (!root)
            continue;
        const cJSON *t = cJSON_GetObjectItem(root, "t");
        if (!cJSON_IsString(t))
        {
            cJSON_Delete(root);
            continue;
        }

        if (strcmp(t->valuestring, "resp") == 0)
        {
            // {"t":"resp","id":123,"mac":"..","status":1,"ts":...,"event":..,"ticket":"..","name":".."}
            to_esp_t out = {0};
            const cJSON *id = cJSON_GetObjectItem(root, "id");
            const cJSON *macs = cJSON_GetObjectItem(root, "mac");
            const cJSON *st = cJSON_GetObjectItem(root, "status");
            const cJSON *ts = cJSON_GetObjectItem(root, "ts");
            const cJSON *ev = cJSON_GetObjectItem(root, "event");
            const cJSON *ticket = cJSON_GetObjectItem(root, "ticket");
            const cJSON *name = cJSON_GetObjectItem(root, "name");
            if (cJSON_IsNumber(id) && cJSON_IsString(macs) && cJSON_IsNumber(st))
            {
                out.req_id = (uint32_t)id->valuedouble;
                str_to_mac(macs->valuestring, out.mac);
                out.status.status = (uint8_t)st->valuedouble;
                out.status.timestamp = (uint32_t)(cJSON_IsNumber(ts) ? ts->valuedouble : 0);
                out.status.event_type = (uint8_t)(cJSON_IsNumber(ev) ? ev->valuedouble : 0);
                if (cJSON_IsString(ticket))
                    strncpy(out.status.ticket_id, ticket->valuestring, sizeof(out.status.ticket_id) - 1);
                if (cJSON_IsString(name))
                    strncpy(out.status.name, name->valuestring, sizeof(out.status.name) - 1);

                // fill auth_key from peer table
                xSemaphoreTake(peer_mutex, portMAX_DELAY);
                int idx = peer_find(out.mac);
                uint32_t auth = (idx >= 0) ? (uint32_t)peers[idx].first_char : 0;
                xSemaphoreGive(peer_mutex);
                out.status.auth_key = auth;

                // enqueue to ESP-NOW sender
                xQueueSend(q_to_esp, &out, 0);

                // notify ack
                uint32_t aid = out.req_id;
                xQueueSend(q_acks, &aid, 0);
            }
        }
        else if (strcmp(t->valuestring, "ack") == 0)
        {
            // optional plain ack path if your Pi wants to separate ack and resp
            const cJSON *id = cJSON_GetObjectItem(root, "id");
            if (cJSON_IsNumber(id))
            {
                uint32_t aid = (uint32_t)id->valuedouble;
                xQueueSend(q_acks, &aid, 0);
            }
        }
        cJSON_Delete(root);
    }
}

static void esp_tx_task(void *arg)
{
    (void)arg;
    to_esp_t m;
    for (;;)
    {
        if (xQueueReceive(q_to_esp, &m, portMAX_DELAY))
        {
            ensure_peer_added(m.mac);
            esp_now_send(m.mac, (uint8_t *)&m.status, sizeof(m.status));
        }
    }
}

// ===== Setup =====
static void init_uart(void)
{
    uart_config_t cfg = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT};
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_RX_BUF, UART_TX_BUF, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static void init_wifi_espnow(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_now_init());
    esp_now_register_recv_cb(espnow_recv_cb);
    esp_now_register_send_cb(espnow_sent_cb);
}

void app_main(void)
{
    peer_mutex = xSemaphoreCreateMutex();
    q_to_pi = xQueueCreate(Q_DEPTH, sizeof(to_pi_t));
    q_to_esp = xQueueCreate(Q_DEPTH, sizeof(to_esp_t));
    q_acks = xQueueCreate(Q_DEPTH, sizeof(uint32_t));

    init_uart();
    init_wifi_espnow();

    xTaskCreatePinnedToCore(uart_tx_task, "uart_tx", 4096, NULL, 8, NULL, 0);
    xTaskCreatePinnedToCore(uart_rx_task, "uart_rx", 4096, NULL, 9, NULL, 0);
    xTaskCreatePinnedToCore(esp_tx_task, "esp_tx", 4096, NULL, 7, NULL, 1);

    ESP_LOGI(TAG, "Bridge started (FreeRTOS)");
}

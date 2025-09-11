#define WIFI_CHANNEL        0           // stay on current
#define UART_PORT           UART_NUM_0
#define UART_TX_PIN         1
#define UART_RX_PIN         3

#define UART_BAUD           115200
#define UART_RX_BUF         2048
#define UART_TX_BUF         1024
#define JSON_LINE_MAX       512

#define SECRET_KEY          0xA5A5F00D
#define Q_DEPTH             32
#define ACK_WAIT_MS         800         // retransmit timeout
#define MAX_RETRIES         3

typedef struct {
    char uid[64]; // ESP-NOW payload from readers (XOR obfuscated on-wire)
} rfid_msg_t;

typedef struct {
    uint8_t  status;      // 0 denied, 1 granted
    uint32_t timestamp;   // epoch or millis
    uint8_t  event_type;  // ENTRY/EXIT/ERROR etc.
    char     ticket_id[16];
    char     name[32];
    uint32_t auth_key;    // filled on ESP
} status_msg_t;

typedef struct {
    uint32_t req_id;
    uint8_t  mac[6];
    char     uid[64];     // plain, de-obfuscated
    uint8_t  attempts;
} to_pi_t;

typedef struct {
    uint32_t    req_id;
    uint8_t     mac[6];
    status_msg_t status;  // auth_key filled before send
} to_esp_t;

typedef struct {
    uint8_t mac[6];
    uint8_t first_char; // from last plain UID[0]
    bool    in_use;
} peer_state_t;

#define peer_arr_length 20
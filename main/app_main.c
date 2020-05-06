#define _GNU_SOURCE
#include <sys/cdefs.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "driver/i2s.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "driver/uart.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

#include "dig_i2s_adc.h"
#include "pages.h"

#define EXAMPLE_ESP_WIFI_MODE_AP   CONFIG_ESP_WIFI_MODE_AP //TRUE:AP FALSE:STA
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_MAX_STA_CONN       CONFIG_MAX_STA_CONN

#define I2S_ADC_UNIT            (ADC_UNIT_1)
#define BUF_LEN                 CONFIG_CAPTURE_BUF_LEN
#define DEFAULT_CLK_DIV         10
#define DEFAULT_CHANNELS_MASK   0x01
#define MAX_CLIENTS             1
#define RX_BUF_SIZE             128

uint16_t buf[BUF_LEN];

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int WIFI_CONNECTED_BIT = BIT0;

static const char *TAG = "Horrorscope ESP32";

SemaphoreHandle_t accept_sem;
StaticSemaphore_t accept_sem_buf;
int srv_sock;

struct data_pkt_hdr {
    uint8_t msgtype;
    uint8_t buf_id;
    uint16_t reserved;
    uint32_t start;
    uint32_t end;
};

struct web_client {
    int sock;
    int task_idx;
    char task_name[16];
    uint8_t rx_buf[RX_BUF_SIZE];
    size_t rx_buf_idx;
    struct sockaddr_in addr;
    int request_terminated;

    char request_line[RX_BUF_SIZE];
    char *request_method, *http_version, *query_string, *request_uri;
};

struct web_client clients[MAX_CLIENTS];


static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "got ip:%s",
                 ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_AP_STACONNECTED:
        ESP_LOGI(TAG, "station:"MACSTR" join, AID=%d",
                 MAC2STR(event->event_info.sta_connected.mac),
                 event->event_info.sta_connected.aid);
        break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
        ESP_LOGI(TAG, "station:"MACSTR"leave, AID=%d",
                 MAC2STR(event->event_info.sta_disconnected.mac),
                 event->event_info.sta_disconnected.aid);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

void wifi_init_softap()
{
    wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished.SSID:%s password:%s",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
}


void wifi_init_sta()
{
    wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL) );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    ESP_LOGI(TAG, "connect to ap SSID:%s password:%s",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
}


esp_err_t i2s_adc_setup(uint8_t channels, int clock_div)
{
    esp_err_t ret;
    adc_channel_t channel[8];
    const adc_channel_t all_channels[] = {
        ADC1_CHANNEL_0, ADC1_CHANNEL_1, ADC1_CHANNEL_2, ADC1_CHANNEL_3,
        ADC1_CHANNEL_4, ADC1_CHANNEL_5, ADC1_CHANNEL_6, ADC1_CHANNEL_7,
    };

    size_t num_channels = 0;
    for (int i = 0; i < 8; i++) {
        int enabled = !!(channels & (1 << i));
        printf("Channel CH%d %s enabled\n", i, enabled ? "is" : "is not");
        if (enabled) {
            adc_channel_t c = all_channels[i];
            channel[num_channels++] = c;
        }
    }

    ret = i2s_adc_init(I2S_NUM_0);
    if (ret != ESP_OK) {
        printf("i2s adc init fail\n");
        return ESP_FAIL;
    }

    if (num_channels == 1) {
        ret = adc_i2s_mode_init(I2S_ADC_UNIT, channel[0]);
        if (ret != ESP_OK) return ret;
    } else {
        //Configuring scan channels
        ret = adc_i2s_scale_mode_init(I2S_ADC_UNIT, channel, num_channels);
        if (ret != ESP_OK) return ret;
    }


    //ADC sampling rate = 20Msps / clkm_num.
    ret = i2s_adc_set_clk(I2S_NUM_0, clock_div);
    if (ret != ESP_OK) return ret;

    ret = i2s_adc_driver_install(I2S_NUM_0, NULL, NULL);
    if (ret != ESP_OK){
        printf("driver install fail\n");
        return ret;
    }

#ifdef DEBUG
    //Out put WS signal from gpio18(only for debug mode)
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[18], PIN_FUNC_GPIO);
    gpio_set_direction(18, GPIO_MODE_DEF_OUTPUT);
    gpio_matrix_out(18, I2S0I_WS_OUT_IDX, 0, 0);
#endif

    return ESP_OK;
}

esp_err_t pwm_setup(void)
{
    int ret;
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .freq_hz = 5000,                      // frequency of PWM signal
        .speed_mode = LEDC_HIGH_SPEED_MODE,           // timer mode
        .timer_num = LEDC_TIMER_0            // timer index
    };
    // Set configuration of timer0 for high speed channels
    ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK)
        return ret;
    
    ledc_channel_config_t ledc_channel = {
        .channel    = LEDC_CHANNEL_0,
        .duty       = 4096,
        .gpio_num   = 19,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0,
    };
    return ledc_channel_config(&ledc_channel);
}


static esp_err_t scope_acquire() {
    esp_err_t ret;

    ret = i2s_adc_prepare(I2S_NUM_0, buf, BUF_LEN*2);
    if (ret != ESP_OK) {
        printf("i2s_adc_prepare fail\n");
        return ret;
    }

    vTaskDelay(1);

    ret = i2s_adc_start(I2S_NUM_0);
    if (ret != ESP_OK) {
        printf("i2s_adc_start fail\n");
        return ret;
    }

    ret = i2s_wait_adc_done(I2S_NUM_0, 1000/portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        printf("i2s_wait_adc_done fail\n");
        return ret;
    }

    return ret;
}


static esp_err_t http_recv_hdr_line(struct web_client *c, char *output, ssize_t *line_len)
{
    size_t max_output = *line_len;

    while (1) {
        // First: check if rx_buf already contains a finished line
        uint8_t *end = memmem(c->rx_buf, c->rx_buf_idx, "\r\n", 2);
        if (end != NULL) {
            size_t output_len = end - c->rx_buf;
            *line_len = output_len;
            ESP_LOGI(TAG, "Line from socket %d is %d bytes long", c->sock, output_len);
            if (output_len == 0) {
                c->request_terminated = 1;
            }

            if (output != NULL) {
                if (output_len + 1 > max_output) {
                    // Not enough space in output buffer
                    ESP_LOGI(TAG, "Line from socket %d is too long", c->sock);
                    return ESP_FAIL;
                } else {
                    // Save the line in the output
                    memcpy(output, c->rx_buf, output_len);
                    output[output_len] = 0;
                    ESP_LOGI(TAG, "Line from socket %d: %s", c->sock, output);
                }
            }

            // Move back the line and advance index
            memcpy(c->rx_buf, c->rx_buf + output_len + 2, c->rx_buf_idx - output_len - 2);
            c->rx_buf_idx -= output_len;
            return ESP_OK;
        }

        // Second: check if rx_buf is full
        size_t space = RX_BUF_SIZE - c->rx_buf_idx;
        if (space == 0) {
            // Move back the last two bytes, this avoids accidentally losing a line
            // Moving only one byte would potentially lead to premature termination of headers
            memcpy(c->rx_buf, c->rx_buf + (RX_BUF_SIZE - 2), 2);
            c->rx_buf_idx = 2;
            if (output != NULL) {
                ESP_LOGE(TAG, "overflow when receiving line from socket %d", c->sock);
                *line_len = -2;
                return ESP_FAIL;
            }
            // If the output is being ignored, we can safely discard the whole line and read more
        }

        // Finally: read some more data over TCP
        ssize_t r = read(c->sock, c->rx_buf + c->rx_buf_idx, space);
        if (r == 0) {
            ESP_LOGI(TAG, "socket %d closed by remote host", c->sock);
            c->request_terminated = 1;
            *line_len = 0;
            return ESP_FAIL;
        } else if (r < 0) {
            ESP_LOGE(TAG, "read failed: errno %d on socket %d", errno, c->sock);
            c->request_terminated = 1;
            *line_len = -1;
            return ESP_FAIL;
        }
        c->rx_buf_idx += r;
    }
}


static esp_err_t http_write(struct web_client *c, const void *buf, size_t len)
{
    const uint8_t *bytebuf = buf;
    ESP_LOGD(TAG, "writing %d bytes to socket %d", len, c->sock);
    while (len > 0) {
        size_t txlen = len > 1400 ? 1400 : len;
        int r = write(c->sock, bytebuf, txlen);
        if (r < 0) {
            ESP_LOGE(TAG, "failed write on socket %d: errno %d", c->sock, errno);
            return ESP_FAIL;
        } else if (r > 0 && r <= txlen) {
            ESP_LOGD(TAG, "wrote %d bytes to socket %d", r, c->sock);
            len -= r;
            bytebuf += r;
        } else {
            ESP_LOGE(TAG, "failed write on socket %d: returned %d", c->sock, r);
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}


static esp_err_t http_get_query_parameter(struct web_client *c, char *param_name, char *outbuf, size_t outlen)
{
    char *haystack = c->query_string;
    size_t param_name_len = strlen(param_name);
    if (param_name_len == 0) {
        return ESP_FAIL;
    }

    while (1) {
        if (0 == memcmp(haystack, param_name, param_name_len)) {
            char term = haystack[param_name_len];
            if (term == '&' || term == '=' || term == 0) {
                if (outbuf == NULL || outlen == 0) {
                    return ESP_OK;
                } else if (term == '=') {
                    char *value = haystack + param_name_len + 1;
                    char *value_end = strchrnul(value, '&');
                    size_t value_len = value_end - value;
                    if (value_len >= outlen) {
                        value_len = outlen - 1;
                    }
                    memcpy(outbuf, value, value_len);
                    outbuf[value_len] = 0;
                    return ESP_OK;
                } else {
                    outbuf[0] = 0;
                    return ESP_OK;
                }
            }
        }

        haystack = strchr(haystack, '&');
        if (haystack == NULL) {
            return ESP_FAIL;
        } else {
            haystack++;
        }
    }
}


static esp_err_t http_handle_setup(struct web_client *c)
{
    esp_err_t ret;
    char parameter_buf[16];
    ESP_LOGI(TAG, "Handling a setup http request");

    uint8_t channels = DEFAULT_CHANNELS_MASK;
    uint32_t clk_div = DEFAULT_CLK_DIV;

    ret = http_get_query_parameter(c, "channels", parameter_buf, sizeof(parameter_buf));
    if (ret == ESP_OK) {
        channels = (uint8_t) strtol(parameter_buf, NULL, 16);
    }
    ret = http_get_query_parameter(c, "div", parameter_buf, sizeof(parameter_buf));
    if (ret == ESP_OK) {
        clk_div = (uint8_t) strtol(parameter_buf, NULL, 10);
    }
    ret = i2s_adc_driver_uninstall(I2S_NUM_0);
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "Failed unistalling the driver");
        goto setup_failed;
    }

    ret = i2s_adc_setup(channels, clk_div);
    if (ret != ESP_OK) {
        goto setup_failed;
    }

    ret = http_write(c, resp_html_hdr, resp_html_hdr_len);
    if (ret != ESP_OK) return ret;
    return http_write(c, "OK", 2);

setup_failed:
    ret = http_write(c, resp_error_hdr, resp_error_hdr_len);
    if (ret != ESP_OK) return ret;
    return http_write(c, webpage_error_start, webpage_error_length);
}


static esp_err_t http_route_request(struct web_client *c)
{
    esp_err_t ret;
    ESP_LOGI(TAG, "Requested path is %s", c->request_uri);

    if (c->request_uri[0] != '/') {
        goto notfound;
    } else if (0 == strcmp(c->request_uri, "/")) {
        ret = http_write(c, resp_html_hdr, resp_html_hdr_len);
        if (ret != ESP_OK) return ret;
        return http_write(c, webpage_index_start, webpage_index_length);

    } else if (0 == strcmp(c->request_uri, "/capture")) {
        ret = http_write(c, resp_binary_hdr, resp_binary_hdr_len);
        if (ret != ESP_OK) return ret;
        ret = scope_acquire();
        if (ret != ESP_OK) return ret;
        return http_write(c, buf, BUF_LEN * sizeof(buf[0]));

    } else if (0 == strcmp(c->request_uri, "/buffer")) {
        ret = http_write(c, resp_binary_hdr, resp_binary_hdr_len);
        if (ret != ESP_OK) return ret;
        return http_write(c, buf, BUF_LEN * sizeof(buf[0]));

    } else if (0 == strcmp(c->request_uri, "/setup")) {
        return http_handle_setup(c);

    } else if (0 == strcmp(c->request_uri, "/dygraph.css")) {
        ret = http_write(c, resp_binary_hdr, resp_binary_hdr_len);
        if (ret != ESP_OK) return ret;
        return http_write(c, webpage_dygraph_css_start, webpage_dygraph_css_length);

    } else if (0 == strcmp(c->request_uri, "/dygraph.js")) {
        ret = http_write(c, resp_binary_hdr, resp_binary_hdr_len);
        if (ret != ESP_OK) return ret;
        return http_write(c, webpage_dygraph_js_start, webpage_dygraph_js_length);

    } else if (0 == strcmp(c->request_uri, "/favicon.ico")) {
        ret = http_write(c, resp_icon_hdr, resp_icon_hdr_len);
        if (ret != ESP_OK) return ret;
        return http_write(c, webpage_favicon_ico_start, webpage_favicon_ico_length);
    }

notfound:
    ret = http_write(c, resp_notfound_hdr, resp_notfound_hdr_len);
    if (ret != ESP_OK) return ret;
    return http_write(c, webpage_notfound_start, webpage_notfound_length);

}


static esp_err_t http_handle_client(struct web_client *c)
{
    esp_err_t ret;
    ssize_t request_line_len = RX_BUF_SIZE;
    char *strtok_r_saveptr;

    xSemaphoreTake(accept_sem, portMAX_DELAY); 
    {
        socklen_t addrlen = sizeof(c->addr);
        ESP_LOGI(TAG, "task %d accepting a client...", c->task_idx);
        c->sock = accept(srv_sock, (struct sockaddr *) &c->addr, &addrlen);
    }
    xSemaphoreGive(accept_sem);

    if (c->sock < 0) {
        ESP_LOGE(TAG, "accept failed: errno %d", errno);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "accepted socket %d", c->sock);

    c->rx_buf_idx = 0;
    c->request_terminated = 0;

    ret = http_recv_hdr_line(c, c->request_line, &request_line_len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ignored request on socket %d: missing %s", c->sock, "Request-Line");
        goto close_and_return;
    }

    c->request_method = strtok_r(c->request_line, " ", &strtok_r_saveptr);
    if (c->request_method == NULL) {
        ESP_LOGE(TAG, "ignored request on socket %d: missing %s", c->sock, "Method");
        goto close_and_return;
    }
    ESP_LOGD(TAG, "request_method is %s", c->request_method);

    c->request_uri = strtok_r(NULL, " ", &strtok_r_saveptr);
    if (c->request_uri == NULL) {
        ESP_LOGE(TAG, "ignored request on socket %d: missing %s", c->sock, "Request-URI");
        goto close_and_return;
    }
    ESP_LOGD(TAG, "request_uri is %s", c->request_uri);

    c->http_version = strtok_r(NULL, " ", &strtok_r_saveptr);
    if (c->http_version == NULL) {
        ESP_LOGE(TAG, "ignored request on socket %d: missing %s", c->sock, "HTTP-Version");
        goto close_and_return;
    }
    ESP_LOGD(TAG, "http_version is %s", c->http_version);

    c->query_string = strchr(c->request_uri, '?');
    if (c->query_string == NULL) {
        c->query_string = "";
    } else {
        *(c->query_string++) = 0;
    }

    ret = http_route_request(c);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Responded successfully to query on socket %d", c->sock);
    } else {
        ESP_LOGW(TAG, "Error when routing query on socket %d", c->sock);
    }

    while (!c->request_terminated) {
        ssize_t slen;
        http_recv_hdr_line(c, NULL, &slen);
    }

close_and_return:
    close(c->sock);
    return ESP_OK;
}


static void http_client_task(void *pvParameters)
{
    int client_task_idx = (int) pvParameters;
    esp_err_t ret = ESP_OK;
    while (ret == ESP_OK) {
        struct web_client *c = &clients[client_task_idx];
        c->task_idx = client_task_idx;
        ret = http_handle_client(c);
    }
}


void app_main()
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
#if EXAMPLE_ESP_WIFI_MODE_AP
    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
    wifi_init_softap();
#else
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
#endif /*EXAMPLE_ESP_WIFI_MODE_AP*/

    ret = pwm_setup();
    ESP_ERROR_CHECK(ret);

    ret = i2s_adc_setup(DEFAULT_CHANNELS_MASK, DEFAULT_CLK_DIV);
    ESP_ERROR_CHECK(ret);

    struct sockaddr_in srv_addr;
    srv_addr.sin_addr.s_addr = inet_addr("0.0.0.0");
    srv_addr.sin_family = AF_INET;;
    srv_addr.sin_port = htons(80);
    srv_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (srv_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        goto error;
    }

    int r = bind(srv_sock, (struct sockaddr *) &srv_addr, sizeof(srv_addr));
    if (r < 0) {
        ESP_LOGE(TAG, "Unable to bind socket: errno %d", errno);
        goto error;
    }

    r = listen(srv_sock, MAX_CLIENTS);
    if (r < 0) {
        ESP_LOGE(TAG, "Unable to listen on socket: errno %d", errno);
        goto error;
    }
    
    accept_sem = xSemaphoreCreateMutexStatic(&accept_sem_buf);

    for (int i = 0; i < MAX_CLIENTS; i++) {
        sprintf(clients[i].task_name, "client_%d", (uint8_t) i);
        xTaskCreate(http_client_task, clients[i].task_name, 3000, (void *)i, 5, NULL);
    }

error:
    while(1) vTaskDelay(1000/portTICK_PERIOD_MS);
}


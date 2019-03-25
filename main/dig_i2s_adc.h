#ifndef __I2S_ADC_H__
#define __I2S_ADC_H__

#include "esp_err.h"
#include <esp_types.h>
#include "soc/soc.h"
#include "esp_attr.h"
#include "driver/periph_ctrl.h"
#include "driver/adc.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*adc_done_cb)(void *param);

/**
 * @brief Set ADC sample clock
 *    Configure WS frequency
 *
 * @param i2s_num   I2S_NUM_0, I2S_NUM_1
 * @param clkm      clock div number
 *
 * @note
 *     Currrently ADC sampling rate = 4000,000 / clkm and 4,000,000 should be divisible by clkm.
 *
 * @return
 *     - ESP_OK               Success
 *     - ESP_ERR_INVALID_ARG  Parameter error
 */
esp_err_t i2s_adc_set_clk(i2s_port_t i2s_num, uint8_t clkm);

/**
 * @brief Configure I2S work in adc mode
 *
 * Digital ADC sampling can be triggered by the WS signal, the ADC sampling rate depending on the frequency of WS signal.
 *
 * @param i2s_num   I2S_NUM_0, I2S_NUM_1
 *
 * @return
 *     - ESP_OK               Success
 *     - ESP_ERR_INVALID_ARG  Parameter error
 */
esp_err_t i2s_adc_init(i2s_port_t i2s_num);

/**
 * @brief Uninstall I2S ADC driver
 *
 * @param i2s_num  I2S_NUM_0, I2S_NUM_1
 *
 * @return
 *     - ESP_OK               Success
 *     - ESP_ERR_INVALID_ARG  Parameter error
 */
esp_err_t i2s_adc_driver_uninstall(i2s_port_t i2s_num);

/**
 * @brief Install I2S ADC driver
 *
 * @param i2s_num  I2S_NUM_0, I2S_NUM_1
 * @param cb  call back ISR callback function, can be NULL.
 * @param param  call back parameter.
 *
 * @note
 * Since the callback function is called in interrupt, it should be short and efficient and cannot be blocked.
 *
 * @return
 *     - ESP_OK               Success
 *     - ESP_ERR_INVALID_ARG  Parameter error
 */
esp_err_t i2s_adc_driver_install(i2s_port_t i2s_num, adc_done_cb cb, void *param);

/**
 * @brief Start AD conversion
 *    Set up DMA to receive the result of the A/D conversion. This function does not wait for the AD conversion to complete,
 *    You should call `i2s_wait_adc_done` to wait for the AD conversion to complete.
 *
 * @param i2s_num  I2S_NUM_0, I2S_NUM_1
 * @param buf  Buffer that stores the A/D results. the buffer cannot be in SPIRAM.
 * @param len The length of data that want to sample.
 *
 * @note
 *     Each sample point takes two bytes, channel num:[15~12]  A/D results: [11~0],
 *     the lenght = sample_point * 2 * total_channel_num, and must be an integer multiple of 4.
 *
 * @return
 *     - ESP_OK               Success
 *     - ESP_ERR_INVALID_ARG  Parameter error
 */
esp_err_t i2s_adc_prepare(i2s_port_t i2s_num, void *buf, size_t len);
esp_err_t i2s_adc_start(i2s_port_t i2s_num);

/**
 * @brief Waite AD conversion done
 *
 * @param i2s_num  I2S_NUM_0, I2S_NUM_1
 * @param ticks_to_wait the wait time for AD conversion done.
 *
 * @return
 *     - ESP_OK               AD conversion is done
 *     - ESP_ERR_INVALID_ARG  AD conversion not finished
 */
esp_err_t i2s_wait_adc_done(i2s_port_t i2s_num, TickType_t ticks_to_wait);

/**
 * @brief Set ADC scale table
 *
 * @param ADC unit index
 * @param channel List of channels that need to be scanned
 * @param channel_num The total number of channels that need to be scanned.
 * e.g:
 *    If you want to scan channel 1, 2, 3 and 4.
 *    the  channel[] = { ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3 };
 *    channel_num = 4
 *
 * @return
 *     - ESP_OK success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t adc_i2s_scale_mode_init(adc_unit_t adc_unit, adc_channel_t *channel, uint8_t channel_num);


/*  Since IDF has not added this function yet, it needs to be manually added by the user (driver/rtc_module.c)

esp_err_t adc_i2s_scale_mode_init(adc_unit_t adc_unit, adc_channel_t *channel, uint8_t channel_num)
{
    ADC_CHECK_UNIT(adc_unit);
    esp_err_t ret = ESP_FAIL;
    uint8_t table_len = channel_num;
    adc_power_always_on();
    ret = adc_set_i2s_data_len(adc_unit, table_len);
    if(ret != ESP_OK) return ret;
    for(int i = 0; i < channel_num; i++) {
        ret = adc_gpio_init(adc_unit, channel[i]);
        if(ret != ESP_OK) return ret;
        ret =  adc_set_i2s_data_pattern(adc_unit, i, channel[i], ADC_WIDTH_BIT_12, ADC_ATTEN_DB_11);
        if(ret != ESP_OK) return ret;
    }
    portENTER_CRITICAL(&rtc_spinlock);
    if (adc_unit & ADC_UNIT_1) {
        adc_set_controller( ADC_UNIT_1, ADC_CTRL_DIG );
    }
    if (adc_unit & ADC_UNIT_2) {
        adc_set_controller( ADC_UNIT_2, ADC_CTRL_DIG );
    }
    portEXIT_CRITICAL(&rtc_spinlock);
    adc_set_i2s_data_source(ADC_I2S_DATA_SRC_ADC);
    //adc_set_i2s_data_source(ADC_I2S_DATA_SRC_IO_SIG);
    adc_set_clk_div(SAR_ADC_CLK_DIV_DEFUALT);
    // Set internal FSM wait time.
    adc_set_fsm_time(ADC_FSM_RSTB_WAIT_DEFAULT, ADC_FSM_START_WAIT_DEFAULT, ADC_FSM_STANDBY_WAIT_DEFAULT,
            ADC_FSM_TIME_KEEP);
    adc_set_work_mode(adc_unit);
    adc_set_data_format(ADC_ENCODE_12BIT);
    adc_set_measure_limit(ADC_MAX_MEAS_NUM_DEFAULT, ADC_MEAS_NUM_LIM_DEFAULT);
    //Invert The Level, Invert SAR ADC1 data
    adc_set_data_inv(adc_unit, false);
    return ESP_OK;
}
*/

#ifdef __cplusplus
}
#endif


#endif
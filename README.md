# esp32scope

This is **Work in progress**, I'm not sure this is ever going to work.

This is an attempt to port the HorrorScope (https://github.com/albert-spruyt/HorrorScope) to the ESP32, but using WiFi instead of USB.

## Features

- Samplerate: 20 Msps
- Memory depth: 50 Kpts
- Resolution: 12 bits

## I2S ADC access
I don't really understand the I2S ADC code myself, I mostly copied it from the "i2s_adc_scale.tar.gz" example found on the forum.

https://www.esp32.com/viewtopic.php?t=2346#p37378

```C
/*  Since IDF has not added this function yet, it needs to be manually added by the user (driver/rtc_module.c) */

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
```

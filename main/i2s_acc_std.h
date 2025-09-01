/*
 * i2s_acc_std.h
 *
 *  Created on: 2020/09/10
 *      Author: nishi
 *  https://qiita.com/ain1084/items/1c00669461d079d3b242
 */

#ifndef MAIN_I2S_ACC_STD_H_
#define MAIN_I2S_ACC_STD_H_

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <stdlib.h>  // ★追記(malloc, free用)
#include <memory.h>

#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_err.h"

#include "esp_system.h"
#include "freertos/FreeRTOS.h"

#include "freertos/task.h"


#include <driver/i2s_std.h>

//#include "driver/gpio.h"
//#include "driver/spi_common.h"
//#include "sdmmc_cmd.h"
//#include "format_wav.h"
#include "esp_log.h"


#define USE_THREAD_TASK


enum I2s_acc_sts {
    ACC_START,
    ACC_IDLE
};

enum I2s_chan_sts {
    CHAN_START,
    CHAN_IDLE,
    CHAN_ENABLE,
};


void i2s_std_init(void);
void i2s_std_start(void);
void i2s_std_stop(void);

int32_t i2s_std_getSample(uint8_t *dt,int32_t dl,int conv_type);
void i2s_std_close(void);

#endif /* MAIN_I2S_ACC_H_ */

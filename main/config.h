#pragma once

//#include <driver/i2s.h>
//#include <driver/i2s_std.h>

// if you want to use ADC input then you need to use I2S_NUM_0
#define I2S_MIC_DEVICE I2S_NUM_0
//#define I2S_MIC_DEVICE I2S_NUM_AUTO
//#define I2S_MIC_DEVICE I2S_NUM_1

// I2S Microphone Settings
// Which channel is the I2S microphone on? I2S_CHANNEL_FMT_ONLY_LEFT or I2S_CHANNEL_FMT_ONLY_RIGHT
// Generally they will default to LEFT - but you may need to attach the L/R pin to GND
#define USE_ESP_EYE
#define USE_I2S_ACC_STD

#if !defined(USE_ESP_EYE)
    #define I2S_MIC_CHANNEL I2S_CHANNEL_FMT_ONLY_LEFT
    // #define I2S_MIC_CHANNEL I2S_CHANNEL_FMT_ONLY_RIGHT
    //     .bck_io_num = I2S_MIC_SERIAL_CLOCK,
    #define I2S_MIC_SERIAL_CLOCK GPIO_NUM_26
    //     .ws_io_num = I2S_MIC_LEFT_RIGHT_CLOCK,
    #define I2S_MIC_LEFT_RIGHT_CLOCK GPIO_NUM_22
    //     .data_in_num = I2S_MIC_SERIAL_DATA
    #define I2S_MIC_SERIAL_DATA GPIO_NUM_21
#else
    //#define SAMPLE_RATE (16000)
    //#define PIN_I2S_BCLK 26   --> CLK  OK
    //#define PIN_I2S_LRC 32    --> WS
    //#define PIN_I2S_DIN 33    --> IN OK
    //#define PIN_I2S_DOUT -1   --> OUT

    //  .bck_io_num = 26,    // CLK
    //  .ws_io_num = 32,     // WS
    //  .data_out_num = -1,  // OUT
    //  .data_in_num = 33,   // IN


    #define I2S_MIC_CHANNEL I2S_CHANNEL_FMT_ONLY_LEFT
    // #define I2S_MIC_CHANNEL I2S_CHANNEL_FMT_ONLY_RIGHT
    //     .bck_io_num = I2S_MIC_SERIAL_CLOCK,
    #define I2S_MIC_SERIAL_CLOCK GPIO_NUM_26
    //     .ws_io_num = I2S_MIC_LEFT_RIGHT_CLOCK,
    #define I2S_MIC_LEFT_RIGHT_CLOCK GPIO_NUM_32
    //     .data_in_num = I2S_MIC_SERIAL_DATA
    #define I2S_MIC_SERIAL_DATA GPIO_NUM_33
#endif


// how often should we check for new samples from the microphone
#define DRIVER_POLL_INTERVAL_MS 5
// DMA settings
#define DMA_BUFFER_COUNT 10
// keep this fairly low to for low latency
#define DMA_BUFFER_SAMPLES 100

// speaker settings
#define I2S_SPEAKER_DEVICE I2S_NUM_1

#define I2S_SPEAKER_SERIAL_CLOCK GPIO_NUM_19
#define I2S_SPEAKER_LEFT_RIGHT_CLOCK GPIO_NUM_27
#define I2S_SPEAKER_SERIAL_DATA GPIO_NUM_18

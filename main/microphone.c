#define BTSTACK_FILE__ "btstack_audio_esp32.c"

#include "btstack_config.h"
#include "btstack_debug.h"
#include "btstack_audio.h"
#include "btstack_run_loop.h"
//#include "hal_audio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
//#include "driver/i2s.h"
//#include <driver/i2s_std.h>
#include "i2s_acc_std.h"

#include "config.h"

// add by nishi
#include "esp_log.h"

static const char *TAG = "microphone";

//i2s_chan_handle_t rx_channel_handle = NULL;
static i2s_chan_handle_t     rx_chan;        // I2S rx channel handler

// client
static void (*recording_callback)(const int16_t *buffer, uint16_t num_samples);

// timer to fill output ring buffer
static btstack_timer_source_t driver_timer_source;

// current gain
static int source_gain = 1;
// are we currently streaming from the microphone
static bool is_source_streaming;
// samples that are read from the microphone
//static int32_t buffer_in[DMA_BUFFER_SAMPLES];
static uint8_t buffer_in[DMA_BUFFER_SAMPLES*2];
// samples that have been converted to PCM 16 bit
//static int16_t samples_in[DMA_BUFFER_SAMPLES];
static uint8_t samples_in[DMA_BUFFER_SAMPLES*2];

static int call_count=0;

// read samples from the microphone and send them off to bluetooth
static void copy_samples(void)
{
    call_count++;
    if(call_count > 200){
        //printf("bytes_read: %d\n", bytes_read);
        ESP_LOGI(TAG, "%s #1",__func__);
        call_count=0;
    }
    // read from I2S - we pass portMAX_DELAY so we don't delay
    size_t bytes_read = 0;
    #if !defined(USE_ESP_EYE)
        //i2s_read(I2S_MIC_DEVICE, buffer_in, DMA_BUFFER_SAMPLES * sizeof(int32_t), &bytes_read, portMAX_DELAY);
        i2s_read(I2S_MIC_DEVICE, buffer_in, DMA_BUFFER_SAMPLES * sizeof(int16_t), &bytes_read, portMAX_DELAY);

        // how many samples have we read
        //int samples_read = bytes_read / sizeof(int32_t);
        int samples_read = bytes_read / sizeof(int16_t);
        if (samples_read > 0)
        {
            printf("copy_samples() #2 buffer_in:%x ,%x, %x, %x, %x, %x ,%x\n",buffer_in[0],buffer_in[1],buffer_in[2],buffer_in[3],buffer_in[4],buffer_in[5],buffer_in[6]);
            // convert the samples to 16 bit
            for (int i = 0; i < samples_read; i++)
            {
                // in theory we should shift to the right by 16 bits, but MEMS microphones have a very
                // high dynamic range, so if we shift all the way we lose a lot of signal
                //samples_in[i] = source_gain * (buffer_in[i] >> 11);
                samples_in[i*2] = buffer_in[i*2+1];
                samples_in[i*2+1] = buffer_in[i*2];

            }
            // send the samples off to be processed
            (*recording_callback)((int16_t *)samples_in, samples_read);
        }
    #elif defined(USE_I2S_ACC_STD)
        int limit_cnt = 5;
        //dt_l = getSample(data,len);
        int conv_type=1;
        int c_cnt=1;
        bytes_read = i2s_std_getSample(samples_in,DMA_BUFFER_SAMPLES*2,conv_type);
        while(bytes_read==0){
            if((c_cnt % 4)==0)
                vTaskDelay(1);
            if(bytes_read==0){
                //ESP_LOGI(BT_AV_TAG, "%s #2 dt_l=%i", __func__,dt_l);
            }
            else if(DMA_BUFFER_SAMPLES*2 != bytes_read){
                ESP_LOGI(TAG, "%s #3 bytes_read=%i", __func__,bytes_read);
            }
            bytes_read = i2s_std_getSample(samples_in,DMA_BUFFER_SAMPLES*2,conv_type);
            c_cnt++;
            if(c_cnt >= limit_cnt)
                break;
        }

        int samples_read = bytes_read / sizeof(int16_t);
        if (samples_read > 0)
        {
            // send the samples off to be processed
            (*recording_callback)((int16_t *)samples_in, samples_read);
        }

        //if(dt_l > 8)
        //    printf("bt_app_a2d_data_cb() #4 data:%x , %x , %x , %x ,%x ,%x, %x,%x\n\n",data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7]);
    #else
        //if (i2s_channel_read(rx_handle, (char *)i2s_readraw_buff, SAMPLE_SIZE, &bytes_read, 1000) == ESP_OK) {
        /* Read i2s data */
        if (i2s_channel_read(rx_chan, buffer_in, DMA_BUFFER_SAMPLES * sizeof(int16_t), &bytes_read, 1000) == ESP_OK) {
            int samples_read = bytes_read / sizeof(int16_t);
            if (samples_read > 0)
            {
                //printf("copy_samples() #2 buffer_in:%x ,%x, %x, %x, %x, %x ,%x\n",buffer_in[0],buffer_in[1],buffer_in[2],buffer_in[3],buffer_in[4],buffer_in[5],buffer_in[6]);
                // convert the samples to 16 bit
                for (int i = 0; i < samples_read; i++)
                {
                    // in theory we should shift to the right by 16 bits, but MEMS microphones have a very
                    // high dynamic range, so if we shift all the way we lose a lot of signal
                    //samples_in[i] = source_gain * (buffer_in[i] >> 11);
                    samples_in[i*2] = buffer_in[i*2+1];
                    samples_in[i*2+1] = buffer_in[i*2];

                }
                // send the samples off to be processed
                (*recording_callback)((int16_t *)samples_in, samples_read);
            }
        }
        else {
            printf("Read Failed!\n");
        }
    #endif
}

// callback from the timer
static void driver_timer_handler_source(btstack_timer_source_t *ts)
{
    // if we're streaming from the microphone then copy the samples from the I2S device
    if (recording_callback)
    {
        copy_samples();
    }
    // re-set timer
    btstack_run_loop_set_timer(ts, DRIVER_POLL_INTERVAL_MS);
    btstack_run_loop_add_timer(ts);
}

// setup the I2S driver
static int btstack_audio_esp32_source_init(
    uint8_t channels,
    uint32_t samplerate,
    void (*recording)(const int16_t *buffer, uint16_t num_samples))
{

    ESP_LOGI(TAG, "%s #1",__func__);

    recording_callback = recording;
    #if !defined(USE_ESP_EYE)
        i2s_config_t config = {
            .mode = I2S_MODE_MASTER | I2S_MODE_RX,
            .sample_rate = samplerate,
            .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
            .channel_format = channels == 2 ? I2S_CHANNEL_FMT_RIGHT_LEFT : I2S_MIC_CHANNEL,
            .communication_format = I2S_COMM_FORMAT_STAND_I2S,
            .dma_buf_count = DMA_BUFFER_COUNT, // Number of DMA buffers. Max 128.
            .dma_buf_len = DMA_BUFFER_SAMPLES, // Size of each DMA buffer in samples. Max 1024.
            .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1};

        i2s_pin_config_t pins = {
            .bck_io_num = I2S_MIC_SERIAL_CLOCK,
            .ws_io_num = I2S_MIC_LEFT_RIGHT_CLOCK,
            .data_out_num = -1,
            .data_in_num = I2S_MIC_SERIAL_DATA};


        esp_err_t ret = 0;

        ret=i2s_driver_install(I2S_MIC_DEVICE, &config, 0, NULL);
        //ret = i2s_driver_install((i2s_port_t)1, &config, 0, NULL);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "%s Error in i2s_driver_install",__func__);
        }
        ret=i2s_set_pin(I2S_MIC_DEVICE, &pins);
        //ret = i2s_set_pin((i2s_port_t)1, &pins);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "%s Error in i2s_set_pin",__func__);
        }

        ret=i2s_zero_dma_buffer(I2S_MIC_DEVICE);
        //ret = i2s_zero_dma_buffer((i2s_port_t)1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "%s Error in initializing dma buffer with 0",__func__);
        }

    #elif defined(USE_I2S_ACC_STD)
        i2s_std_init();
    #else

        //#define SAMPLE_RATE (16000)
        #define PIN_I2S_BCLK 26
        #define PIN_I2S_WS 32
        #define PIN_I2S_DIN 33
        #define PIN_I2S_DOUT -1


        // Start listening for audio: MONO @ 16KHz
        static i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
        ESP_ERROR_CHECK(i2s_new_channel(&rx_chan_cfg, NULL, &rx_chan));

        static i2s_std_config_t rx_std_cfg = {
            //.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(44100),
            //.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(44000),
            .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000),
            // 下記の指定で、MSB LSB の順番が変えられるみたい。
            // 今までの指定。
            .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
            // 下に変えてみる。
            //.slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,I2S_SLOT_MODE_STEREO),
            //.slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,I2S_SLOT_MODE_MONO),

            .gpio_cfg = {
                //.mclk = GPIO_NUM_0,
                .mclk = I2S_GPIO_UNUSED,    // some codecs may require mclk signal, this example doesn't need it
                //.bclk = 26,
                .bclk = PIN_I2S_BCLK,
                //.ws = 32,
                .ws = PIN_I2S_WS,
                //.dout = -1,
                .dout = PIN_I2S_DOUT,
                //.din = 33,
                .din = PIN_I2S_DIN,
                .invert_flags = {
                    .mclk_inv = false,
                    .bclk_inv = false,
                    .ws_inv = false
                }
            }
        };

        /* Default is only receiving left slot in mono mode,
            * update to right here to show how to change the default configuration */
        //rx_std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_RIGHT;
        rx_std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;    // これが、必要みたい。by nishi 2025.8.26
        ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &rx_std_cfg));

        /* Enable the RX channel */
        //ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
    #endif

    return 0;
}

// add by nishi dummy now 2025.8.29
//uint32_t (*get_samplerate)(void)
static uint32_t btstack_audio_esp32_get_samplerate(void){
    ESP_LOGI(TAG, "%s #1",__func__);
    // dummy by nishi
    return 16000;
}


// update the gain
static void btstack_audio_esp32_source_gain(uint8_t gain)
{
    ESP_LOGI(TAG, "%s #1",__func__);
    source_gain = gain;
}

// start streaming from the microphone
static void btstack_audio_esp32_source_start_stream()
{
    ESP_LOGI(TAG, "%s #1",__func__);
    // start i2s
    #if !defined(USE_ESP_EYE)
        i2s_start(I2S_MIC_DEVICE);
    #elif defined(USE_I2S_ACC_STD)
        i2s_std_start();
    #else
        ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
    #endif

    // start timer
    btstack_run_loop_set_timer_handler(&driver_timer_source, &driver_timer_handler_source);
    btstack_run_loop_set_timer(&driver_timer_source, DRIVER_POLL_INTERVAL_MS);
    btstack_run_loop_add_timer(&driver_timer_source);

    // state
    is_source_streaming = true;
}

// stop streaming from the microphone
static void btstack_audio_esp32_source_stop_stream(void)
{
    ESP_LOGI(TAG, "%s #1",__func__);

    if (!is_source_streaming)
        return;

    // stop timer
    btstack_run_loop_remove_timer(&driver_timer_source);

    // stop i2s
    #if !defined(USE_ESP_EYE)
        i2s_stop(I2S_MIC_DEVICE);
    #elif defined(USE_I2S_ACC_STD)
        i2s_std_stop();
    #else
        ESP_ERROR_CHECK(i2s_channel_disable(rx_chan));
    #endif

    // state
    is_source_streaming = false;
}

// shutdown the driver
static void btstack_audio_esp32_source_close(void)
{
    ESP_LOGI(TAG, "%s #1",__func__);
    if (is_source_streaming)
    {
        btstack_audio_esp32_source_stop_stream();
    }
    // uninstall driver
    #if !defined(USE_ESP_EYE)
        i2s_driver_uninstall(I2S_MIC_DEVICE);
    #elif defined(USE_I2S_ACC_STD)
        i2s_std_close();
    #else
        ESP_ERROR_CHECK(i2s_del_channel(rx_chan));
    #endif
}
// /home/nishi/esp/v5.5/esp-idf/components/btstack/src/btstack_audio.h
// line 141
static const btstack_audio_source_t btstack_audio_source_esp32 = {
    /* 1 int (*init)(..);*/ &btstack_audio_esp32_source_init,
    /* 2 uint32_t (*get_samplerate)(void); */
    // add by nishi for compile error
                            &btstack_audio_esp32_get_samplerate,
    /* 3 void (*set_gain)(uint8_t gain); */
    /* void (*set_gain)(uint8_t gain); */ &btstack_audio_esp32_source_gain,
    /* 4. void (*start_stream)(void); */
    /* void (*start_stream(void));*/ &btstack_audio_esp32_source_start_stream,
    /* 5. void (*stop_stream)(void); */
    /* void (*stop_stream)(void)  */ &btstack_audio_esp32_source_stop_stream,
    /* 6. void (*close)(void); */
    /* void (*close)(void); */ &btstack_audio_esp32_source_close};

const btstack_audio_source_t *btstack_audio_esp32_source_get_instance(void)
{
    return &btstack_audio_source_esp32;
}

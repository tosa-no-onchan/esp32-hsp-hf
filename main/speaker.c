/*
 * Copyright (C) 2018 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL MATTHIAS
 * RINGWALD OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at 
 * contact@bluekitchen-gmbh.com
 *
 */

/*
 *  btstack_audio_esp32.c
 *
 *  Implementation of btstack_audio.h using polling ESP32 I2S driver
 *
 */

#include "btstack_config.h"
#include "btstack_debug.h"
#include "btstack_audio.h"
#include "btstack_run_loop.h"
//#include "hal_audio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

//#include "driver/i2s.h"
#include <driver/i2s_std.h>

#include "config.h"


// add by nishi
#include "esp_log.h"
#include "i2s_example_pins.h"

static const char *TAG = "speaker";

static i2s_chan_handle_t                tx_chan;        // I2S tx channel handler

#define EXAMPLE_STD_BCLK_IO1        EXAMPLE_I2S_BCLK_IO1      // I2S bit clock io number
#define EXAMPLE_STD_WS_IO1          EXAMPLE_I2S_WS_IO1      // I2S word select io number
#define EXAMPLE_STD_DOUT_IO1        EXAMPLE_I2S_DOUT_IO1     // I2S data out io number
#define EXAMPLE_STD_DIN_IO1         EXAMPLE_I2S_DIN_IO1     // I2S data in io number
#if !defined(EXAMPLE_I2S_DUPLEX_MODE)
  #define EXAMPLE_STD_BCLK_IO2        EXAMPLE_I2S_BCLK_IO2     // I2S bit clock io number
  #define EXAMPLE_STD_WS_IO2          EXAMPLE_I2S_WS_IO2     // I2S word select io number
  #define EXAMPLE_STD_DOUT_IO2        EXAMPLE_I2S_DOUT_IO2     // I2S data out io number
  #define EXAMPLE_STD_DIN_IO2         EXAMPLE_I2S_DIN_IO2     // I2S data in io number
#endif


// client
static void (*playback_callback)(int16_t *buffer, uint16_t num_samples) = NULL;

// timer to fill output ring buffer
static btstack_timer_source_t driver_timer;

static bool is_sink_streaming;

static int sink_gain = 1;
static int sink_channels = 2;

// provide enough space for 2 channels - the way channels is done in this example is a bit odd
static int16_t buffer[DMA_BUFFER_SAMPLES * 2];
static void fill_buffer(void)
{
  size_t bytes_written;
  if (playback_callback)
  {
    #if !defined(USE_ESP_EYE)
      (*playback_callback)(buffer, DMA_BUFFER_SAMPLES);
      for (int i = 0; i < DMA_BUFFER_SAMPLES * sink_channels; i++)
      {
        // gain doesn't seem to be wired up
        // buffer[i] = buffer[i] * sink_gain;
        // seems to be exceptionally loud so we scale it down
        buffer[i] = buffer[i] * 0.5;
      }
      i2s_write(I2S_SPEAKER_DEVICE, buffer, DMA_BUFFER_SAMPLES * sink_channels * sizeof(int16_t), &bytes_written, portMAX_DELAY);
    #else
      // 下記を参照します。
      // ~/esp/v5.5/esp-idf/examples/peripherals/i2s/i2s_basic/i2s_std/main/i2s_std_example_main.c
      (*playback_callback)(buffer, DMA_BUFFER_SAMPLES);
      for (int i = 0; i < DMA_BUFFER_SAMPLES * sink_channels; i++)
      {
        // gain doesn't seem to be wired up
        // buffer[i] = buffer[i] * sink_gain;
        // seems to be exceptionally loud so we scale it down
        buffer[i] = buffer[i] * 0.5;
      }
      // ここは、まだ組み込んでいません!!
      /* Enable the TX channel */
      //ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));

    #endif
  }
}

static void driver_timer_handler(btstack_timer_source_t *ts)
{

  // playback buffer ready to fill
  if (playback_callback)
  {
    fill_buffer();
  }

  // re-set timer
  btstack_run_loop_set_timer(ts, DRIVER_POLL_INTERVAL_MS);
  btstack_run_loop_add_timer(ts);
}

static int btstack_audio_esp32_sink_init(
    uint8_t channels,
    uint32_t samplerate,
    void (*playback)(int16_t *buffer, uint16_t num_samples))
{
  playback_callback = playback;
  sink_channels = channels;

  ESP_LOGI(TAG, "%s #1",__func__);

  #if !defined(USE_ESP_EYE)

      i2s_config_t config =
          {
              .mode = I2S_MODE_MASTER | I2S_MODE_TX, // Playback only
              .sample_rate = samplerate,
              .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
              .channel_format = channels == 2 ? I2S_CHANNEL_FMT_RIGHT_LEFT : I2S_CHANNEL_FMT_ONLY_LEFT,
              .communication_format = I2S_COMM_FORMAT_STAND_I2S,
              .dma_buf_count = DMA_BUFFER_COUNT, // Number of DMA buffers. Max 128.
              .dma_buf_len = DMA_BUFFER_SAMPLES, // Size of each DMA buffer in samples. Max 1024.
              .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1};

      i2s_pin_config_t pins = {
          .bck_io_num = I2S_SPEAKER_SERIAL_CLOCK,
          .ws_io_num = I2S_SPEAKER_LEFT_RIGHT_CLOCK,
          .data_out_num = I2S_SPEAKER_SERIAL_DATA,
          .data_in_num = I2S_PIN_NO_CHANGE};

      i2s_driver_install(I2S_SPEAKER_DEVICE, &config, 0, NULL);
      i2s_set_pin(I2S_SPEAKER_DEVICE, &pins);
      i2s_zero_dma_buffer(I2S_SPEAKER_DEVICE);

    #else
      /* Setp 1: Determine the I2S channel configuration and allocate two channels one by one
      * The default configuration can be generated by the helper macro,
      * it only requires the I2S controller id and I2S role
      * The tx and rx channels here are registered on different I2S controller,
      * Except ESP32 and ESP32-S2, others allow to register two separate tx & rx channels on a same controller */
      i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
      ESP_ERROR_CHECK(i2s_new_channel(&tx_chan_cfg, &tx_chan, NULL));

      /* Step 2: Setting the configurations of standard mode and initialize each channels one by one
      * The slot configuration and clock configuration can be generated by the macros
      * These two helper macros is defined in 'i2s_std.h' which can only be used in STD mode.
      * They can help to specify the slot and clock configurations for initialization or re-configuring */
      i2s_std_config_t tx_std_cfg = {
          .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(16000),
          .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
          .gpio_cfg = {
              .mclk = I2S_GPIO_UNUSED,    // some codecs may require mclk signal, this example doesn't need it
              .bclk = EXAMPLE_STD_BCLK_IO1,
              .ws   = EXAMPLE_STD_WS_IO1,
              .dout = EXAMPLE_STD_DOUT_IO1,
              .din  = EXAMPLE_STD_DIN_IO1,
              .invert_flags = {
                  .mclk_inv = false,
                  .bclk_inv = false,
                  .ws_inv   = false,
              },
          },
      };
      ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &tx_std_cfg));

    #endif

  return 0;
}

/* uint32_t (*get_samplerate)(void);*/
/* add dummy by nishi 2025.8.29 */
static uint32_t btstack_audio_esp32_sink_get_samplerate(void){
  ESP_LOGI(TAG, "%s #1",__func__);
  // dummy by nishi 2025.8.29
  return 16000;
}


static void btstack_audio_esp32_sink_gain(uint8_t gain)
{
  ESP_LOGI(TAG, "%s #1",__func__);
  sink_gain = gain;
}

static void btstack_audio_esp32_sink_start_stream(void)
{
  ESP_LOGI(TAG, "%s #1",__func__);
  // start i2s
  #if !defined(USE_ESP_EYE)
    i2s_start(I2S_SPEAKER_DEVICE);
  #else
    //ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));
  #endif

  // start timer
  btstack_run_loop_set_timer_handler(&driver_timer, &driver_timer_handler);
  btstack_run_loop_set_timer(&driver_timer, DRIVER_POLL_INTERVAL_MS);
  btstack_run_loop_add_timer(&driver_timer);

  // state
  is_sink_streaming = true;
}

static void btstack_audio_esp32_sink_stop_stream(void)
{
  ESP_LOGI(TAG, "%s #1",__func__);
  if (!is_sink_streaming)
    return;

  // stop timer
  btstack_run_loop_remove_timer(&driver_timer);

  // stop i2s
  #if !defined(USE_ESP_EYE)
    i2s_stop(I2S_SPEAKER_DEVICE);
  #else
    //ESP_ERROR_CHECK(i2s_channel_disable(tx_chan));
  #endif

  // state
  is_sink_streaming = false;
}

static void btstack_audio_esp32_sink_close(void)
{
  ESP_LOGI(TAG, "%s #1",__func__);
  if (is_sink_streaming)
  {
    btstack_audio_esp32_sink_stop_stream();
  }

  // uninstall driver
  #if !defined(USE_ESP_EYE)
    i2s_driver_uninstall(I2S_SPEAKER_DEVICE);
  #else
    ESP_ERROR_CHECK(i2s_del_channel(tx_chan));
  #endif
}
// components/btstack/src/btstack_audio.h:98: btstack_audio_sink_t;
static const btstack_audio_sink_t btstack_audio_sink_esp32 = {
    /* 1. int (*init)(..);*/ &btstack_audio_esp32_sink_init,
    /* 2. uint32_t (*get_samplerate)(void); */
                                      // add dummy by nishi 2025.8.29
                                      &btstack_audio_esp32_sink_get_samplerate,
    /* 3. void (*set_volume)(uint8_t volume); */
    /* void (*set_gain)(uint8_t gain); */ &btstack_audio_esp32_sink_gain,
    /* 4. void (*start_stream)(void); */
    /* void (*start_stream(void));*/ &btstack_audio_esp32_sink_start_stream,
    /* 5. void (*stop_stream)(void); */
    /* void (*stop_stream)(void)  */ &btstack_audio_esp32_sink_stop_stream,
    /* 6. void (*close)(void); */
    /* void (*close)(void); */ &btstack_audio_esp32_sink_close};

const btstack_audio_sink_t *btstack_audio_esp32_sink_get_instance(void)
{
  return &btstack_audio_sink_esp32;
}

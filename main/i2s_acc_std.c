/*
 * i2s_acc.c
 *
 *  Created on: 2020/09/10
 *      Author: nishi
 *
 *  https://docs.espressif.com/projects/esp-idf/en/v3.3/api-reference/peripherals/i2s.html#_CPPv417i2s_comm_format_t
 * 
 * /home/nishi/esp/v5.5/esp-idf/examples/peripherals/i2s/i2s_basic/i2s_std/main/i2s_std_example_main.c
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//#include <driver/i2s_std.h>

#include "esp_log.h"
#include "i2s_acc_std.h"


//#
//# I2S MEMS MIC Configuration
//#
//CONFIG_EXAMPLE_SAMPLE_RATE=44100
//CONFIG_EXAMPLE_BIT_SAMPLE=16
//CONFIG_EXAMPLE_I2S_DATA_GPIO=5
//CONFIG_EXAMPLE_I2S_CLK_GPIO=4
//# end of I2S MEMS MIC Configuration

//#define CONFIG_EXAMPLE_SAMPLE_RATE 44100
#define CONFIG_EXAMPLE_SAMPLE_RATE 16000
#define CONFIG_EXAMPLE_BIT_SAMPLE 16
#if defined(CONFIG_EXAMPLE_I2S_DATA_GPIO)
    #undef CONFIG_EXAMPLE_I2S_DATA_GPIO
#endif
#if defined(CONFIG_EXAMPLE_I2S_CLK_GPIO)
    #undef CONFIG_EXAMPLE_I2S_CLK_GPIO
#endif
//#define CONFIG_EXAMPLE_I2S_DATA_GPIO 5
#define CONFIG_EXAMPLE_I2S_DATA_GPIO 33
//#define CONFIG_EXAMPLE_I2S_CLK_GPIO 4
#define CONFIG_EXAMPLE_I2S_CLK_GPIO 26


//------------
#define I2S_SAMPLE_RATE     16000
#define I2S_BUFFER_SIZE     1024
#define I2S_PIN_PDMCLK 26
#define I2S_PIN_PDMDIN 33


//#define SAMPLE_RATE (16000)
//#define PIN_I2S_BCLK 26   --> CLK  OK
//#define PIN_I2S_LRC 32    --> WS
//#define PIN_I2S_DIN 33    --> IN OK
//#define PIN_I2S_DOUT -1   --> OUT

//      .bck_io_num = 26,    // IIS_SCLK 
//      .ws_io_num = 32,     // IIS_WS
//      .data_out_num = -1,  // IIS_DOUT
//      .data_in_num = 33,   // IIS_DIN



// https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/i2s.html

// https://esp32.com/viewtopic.php?t=15143
// scl:G32, clock:G26, dat:G33
// https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf
// https://www.reddit.com/r/esp32/comments/tz6t47/which_are_the_i%C2%B2s_pins_of_the_esp32/?tl=ja
// #define PIN_I2S_SD_IN 35 // ピン35は入力専用！

// https://lang-ship.com/blog/work/mic-m5stickc-m5stackfire-espeye-2/
//#define I2S_PIN_CLK         26
//#define I2S_PIN_WS          32
//#define I2S_PIN_DOUT        I2S_PIN_NO_CHANGE
//#define I2S_PIN_DIN         33
//  i2s_pin_config_t pin_config = {
//    .bck_io_num           = I2S_PIN_CLK, -> 26
//    .ws_io_num            = I2S_PIN_WS,  -> 32
//    .data_out_num         = I2S_PIN_DOUT, -> -1
//    .data_in_num          = I2S_PIN_DIN,  -> 33
//  };


#define SPI_DMA_CHAN        SPI_DMA_CH_AUTO
#define NUM_CHANNELS        (1) // For mono recording only!
//#define SD_MOUNT_POINT      "/sdcard"
#define SAMPLE_SIZE         (CONFIG_EXAMPLE_BIT_SAMPLE * 1024)
#define BYTE_RATE           (CONFIG_EXAMPLE_SAMPLE_RATE * (CONFIG_EXAMPLE_BIT_SAMPLE / 8)) * NUM_CHANNELS


//#define SAMPLE_RATE (16000)
#define PIN_I2S_BCLK 26
#define PIN_I2S_WS 32
#define PIN_I2S_DIN 33
#define PIN_I2S_DOUT -1

// from i2s_acc.c
//      .bck_io_num = 26,    // IIS_BCLK  -> CLK
//      .ws_io_num = 32,     // IIS_LCLK  -> WS
//      .data_out_num = -1,  // IIS_DOUT
//      .data_in_num = 33,   // IIS_DSIN


static const char *TAG = "I2S_ACC_STD";
static int init_f=0;
static bool i2s_read_on=false;

enum I2s_acc_sts i2s_acc_sts = ACC_START;
enum I2s_chan_sts i2s_chan_sts = CHAN_START;

//#define SAMPLE_SIZE         (CONFIG_EXAMPLE_BIT_SAMPLE * 1024)
//#define BYTE_RATE           (CONFIG_EXAMPLE_SAMPLE_RATE * (CONFIG_EXAMPLE_BIT_SAMPLE / 8)) * NUM_CHANNELS

//i2s_chan_handle_t rx_channel_handle = NULL;
static i2s_chan_handle_t     rx_chan;        // I2S rx channel handler

static int16_t i2s_readraw_buff[SAMPLE_SIZE];


//#define USE_TEST_PRINT

// キューの大きさ
#define QUEUE_LENGTH 4
//#define QUEUE_LENGTH 6
//#define QUEUE_LENGTH 2
// 通常のキュー
QueueHandle_t xQueue;   // 送信待ちキュー
QueueHandle_t xQueue_free;  // Freeキュー


//#define I2S_STD_BUFF_SIZE 2048
//#define I2S_STD_BUFF_SIZE 512
//#define I2S_STD_BUFF_SIZE 640
//#define I2S_STD_BUFF_SIZE 512/2/2
#define I2S_STD_BUFF_SIZE 100*2

inline int i2s_std_mono2mono_cp_16bit(uint8_t *dst,uint8_t *src,size_t buf_len){
	int lp=buf_len / 2;	// samples per i2s frame
	// I2S  32bit stereo data and 1 channel active ->  16bit mono data
    for (int i = 0; i < lp; ++i) {
      dst[0] = src[1];
      dst[1] = src[0];
      dst+=2;
      src+=2;
    }
    return buf_len;
}

//static int i2s_std_mono2mono_cp_16bit(uint8_t *dst,uint8_t *src,size_t buf_len){
//	int lp=buf_len / 2;	// samples per i2s frame
//	// I2S  32bit stereo data and 1 channel active ->  16bit mono data
//    for (int i = 0; i < lp; ++i) {
//      dst[0] = src[1];
//      dst[1] = src[0];
//      dst+=2;
//      src+=2;
//    }
//    return buf_len;
//}

/*
  16bit mono 16000[Hz] -> 16bit 2channel 32000[Hz] に拡張する
 src[I2S_STD_BUFF_SIZE] -> dts[I2S_STD_BUFF_SIZE*2*2]
*/
static int i2s_std_mono2stero_32000_cp_16bit(uint8_t *dst,uint8_t *src,size_t buf_len){
	int lp=buf_len / 2 /2 /2 ;	// samples per i2s frame  512 /2/2/2=64
  //uint16_t *dst_16 = (uint8_t *)dst;
  //uint16_t *src_16 = (uint8_t *)src;

	// I2S  32bit stereo data and 1 channel active ->  16bit mono data
    for (int i = 0; i < lp; ++i) {
      dst[0] = src[1];
      dst[1] = src[0];
      dst[2] = dst[0];
      dst[3] = dst[1];
      dst[4] = dst[0];
      dst[5] = dst[1];
      dst[6] = dst[0];
      dst[7] = dst[1];
      dst+=8;
      src+=2;
    }
    return buf_len;
}


static void i2s_std_read_task(void *args)
{
  //uint8_t *r_buf = (uint8_t *)calloc(1, I2S_STD_BUFF_SIZE);
  uint8_t *r_buf;
  //assert(r_buf); // Check if r_buf allocation success
  size_t r_bytes = 0;

  int task_cnt=0;
  /* Enable the RX channel */
  //ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));

  /* ATTENTION: The print and delay in the read task only for monitoring the data by human,
    * Normally there shouldn't be any delays to ensure a short polling time,
    * Otherwise the dma buffer will overflow and lead to the data lost */
  while (1) {
    switch(i2s_chan_sts){
      case CHAN_ENABLE:
      {
        if(i2s_read_on){
          // Free キューのチェック
          int n = uxQueueMessagesWaiting(xQueue_free);
          #if defined(USE_TEST_PRINT)
            if(n==0)
              ESP_LOGI(TAG, "%s #1 n: %d", __func__, n);
          #endif
          if(n>0){
            // Freeキュー を貰います。
            int ret = xQueueReceive( xQueue_free, &r_buf, 0 );

            //ESP_LOGI(TAG, "%s #2 r_buf:%p", __func__, (void *)r_buf);

            /* Read i2s data */
            if (i2s_channel_read(rx_chan, r_buf, I2S_STD_BUFF_SIZE, &r_bytes, 1000) == ESP_OK) {
              //printf("Read Task: i2s read %d bytes\n-----------------------------------\n", r_bytes);
              // check OK by nishi 2025.8.27
              //printf("rbuf: %x , %x , %x , %x ,%x , %x , %x , %x \n\n",r_buf[0], r_buf[1], r_buf[2], r_buf[3], r_buf[4], r_buf[5], r_buf[6], r_buf[7]);

              // 送信キューの 最後に追加
              xQueueSend(xQueue, &r_buf, 0);
            } 
            else {
                printf("i2s_std_read_task() #10: i2s read failed\n");
            }
          }
        }
        task_cnt++;
        //if(task_cnt>20){
        if(task_cnt>50){
        //if(task_cnt>250){
        //if(task_cnt>500){
          vTaskDelay(1);
          task_cnt=0;
        }
        break;
      }
      //esp_task_wdt_reset();
      default:
        vTaskDelay(pdMS_TO_TICKS(200));
        break;
    }
  }
  vTaskDelete(NULL);
}

void i2s_std_mk_queue(){
  // キュー作成
  xQueue = xQueueCreate( QUEUE_LENGTH, sizeof(uint8_t *) );
  xQueue_free = xQueueCreate( QUEUE_LENGTH, sizeof(uint8_t *) );

  for(int i=0; i < QUEUE_LENGTH;i++){
    uint8_t * ptr = malloc(I2S_STD_BUFF_SIZE);
    assert(ptr); // Check if r_buf allocation success
    // Freeキュー へ、追加します。
    //xQueueSend(xQueue_free, ptr, 0);
    xQueueSend(xQueue_free, &ptr, 0);
    //printf("The address of x is: %p\n", (void *)ptr); 
    printf("i2s_std_mk_queue() #2 ptr:%p\n",(void *)ptr);
//i2s_std_mk_queue() #2 ptr:0x3ffc828c
//i2s_std_mk_queue() #2 ptr:0x3ffc8490
//i2s_std_mk_queue() #2 ptr:0x3ffc8694
//i2s_std_mk_queue() #2 ptr:0x3ffc8898

  }
}

void i2s_std_init() {
  //if(init_f==1)
  //  return;

  if(i2s_chan_sts == CHAN_START){
    static i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);

    // add by nishi 2025.9.1
    rx_chan_cfg.auto_clear = true; // Auto clear the legacy data in the DMA buffer
    //rx_chan_cfg.dma_desc_num = EXAMPLE_I2S_DMA_DESC_NUM;
    //rx_chan_cfg.dma_desc_num = 8;
    rx_chan_cfg.dma_desc_num = 4;
    //rx_chan_cfg.dma_frame_num = EXAMPLE_I2S_DMA_FRAME_NUM;
    rx_chan_cfg.dma_frame_num = I2S_STD_BUFF_SIZE;

    ESP_ERROR_CHECK(i2s_new_channel(&rx_chan_cfg, NULL, &rx_chan));

    //i2s_chan_config_t channel_config = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    //i2s_chan_handle_t tx_channel_handle;
    //ESP_ERROR_CHECK(i2s_new_channel(&channel_config,NULL, &rx_channel_handle /* rx_handle */));

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

    i2s_chan_sts = CHAN_IDLE;

    /* Enable the RX channel */
    //ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
    //i2s_chan_sts = CHAN_ENABLE;

  }

  if(i2s_acc_sts == ACC_START){
    i2s_std_mk_queue();

    //xTaskCreate(i2s_std_read_task, "i2s_std_read_task", 4096, NULL, 5, NULL);
    //xTaskCreate(i2s_std_read_task, "i2s_std_read_task", 4096, NULL, 24, NULL);
    //xTaskCreate(i2s_std_read_task, "i2s_std_read_task", 1024, NULL, 24, NULL);
    //xTaskCreatePinnedToCore(i2s_std_read_task, "i2s_std_read_task", 2048, NULL, 24, NULL,1);
    //xTaskCreatePinnedToCore(i2s_std_read_task, "i2s_std_read_task", 2048, NULL, 5, NULL,1);
    //xTaskCreatePinnedToCore(i2s_std_read_task, "i2s_std_read_task", 2048, NULL, 10, NULL,1);
    //xTaskCreatePinnedToCore(i2s_std_read_task, "i2s_std_read_task", 4096, NULL, 15, NULL,1);
    //xTaskCreatePinnedToCore(i2s_std_read_task, "i2s_std_read_task", 4096, NULL, 20, NULL,1);
    xTaskCreatePinnedToCore(i2s_std_read_task, "i2s_std_read_task", 4096, NULL, 20, NULL,0);
    i2s_acc_sts=ACC_IDLE;
  }
  //init_f=1;
}

void i2s_std_start(void){
  /* Enable the RX channel */
  if(i2s_chan_sts == CHAN_IDLE){
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
    i2s_chan_sts = CHAN_ENABLE;
  }
  i2s_read_on=true;
}


void i2s_std_stop(void){
  if(i2s_chan_sts == CHAN_ENABLE){
    i2s_chan_sts = CHAN_IDLE;
    i2s_read_on=false;
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_ERROR_CHECK(i2s_channel_disable(rx_chan));
  }

}

void i2s_std_close(void){

}


#if defined(USE_THREAD_TASK)
int32_t i2s_std_getSample(uint8_t *dt,int32_t dl,int conv_type){
  i2s_read_on=true;
  int32_t dl_act=0;
  uint8_t *ptr;

  //if(!init_f){
	//  i2s_std_init();
	//  //init_f=1;
  //}

  int n = uxQueueMessagesWaiting(xQueue);
  #if defined(USE_TEST_PRINT)
    if(n==0)
      ESP_LOGI(TAG, "%s #1 n: %d", __func__, n);
  #endif
  if(n > 0){
    if(dl >= I2S_STD_BUFF_SIZE){
      // 送信キューを貰います。
      int ret = xQueueReceive( xQueue, &ptr, 0 );
      //printf("i2s_std_getSample() #1 ret:%d\n",ret);
      //ESP_LOGI(TAG, "%s #2 ptr:%p", __func__, (void *)ptr);

      //printf("i2s_std_getSample() #3 ptr:%x , %x , %x , %x\n",ptr[0],ptr[1],ptr[2],ptr[3]);
      switch(conv_type){
        case 1:{
          dl_act=I2S_STD_BUFF_SIZE;
          i2s_std_mono2mono_cp_16bit(dt,ptr,I2S_STD_BUFF_SIZE);
          }
          break;
        case 2:{
          dl_act=i2s_std_mono2stero_32000_cp_16bit(dt,ptr,dl);
          }
          break;
        case 0:
          dl_act=I2S_STD_BUFF_SIZE;
          memcpy(dt,ptr,I2S_STD_BUFF_SIZE);
          break;
      }

      // Noise generate dummy
      //#define USE_TEST_DUMMY
      #if defined(USE_TEST_DUMMY)
        int16_t *p_buf = (int16_t *)dt;
        for (int i = 0; i < (dl >> 1); i++) {
            p_buf[i] = rand() % (1 << 16);
        }
      #endif

      // チェック OK by nishi 2025.8.27
      //printf("i2s_std_getSample() #4 dt:%x , %x , %x , %x ,%x ,%x, %x,%x\n\n",dt[0],dt[1],dt[2],dt[3],dt[4],dt[5],dt[6],dt[7]);
      // Freeキュー へ、戻します。
      xQueueSend(xQueue_free, &ptr, 0);
    }
    else{
      ESP_LOGW(TAG, "%s #5 data size unmatch: %d", __func__, dl);
      //printf("i2s_std_getSample() #5 data size unmatch\n");
    }
  }
  return dl_act;
}
#else
int32_t i2s_std_getSample(uint8_t *dt,int32_t dl,int conv_type){
  size_t bytes_read,tatals;
  if(!init_f){
	  //i2s_std_init();
	  //init_f=1;
  }
  tatals=0;
  int32_t dl_act=0;
  while(1){
    //if (i2s_channel_read(rx_handle, (char *)i2s_readraw_buff, SAMPLE_SIZE, &bytes_read, 1000) == ESP_OK) {
    if (i2s_channel_read(rx_chan, dt, dl, &bytes_read, 1000) == ESP_OK) {
        //printf("bytes_read: %d\n", bytes_read);
        //printf("[0] %d [1] %d [2] %d [3]%d ...\n", i2s_readraw_buff[0], i2s_readraw_buff[1], i2s_readraw_buff[2], i2s_readraw_buff[3]);
        //printf("[0] %d [1] %d [2] %d [3]%d ...\n", dt[0], dt[1], dt[2], dt[3]);
        #if defined(USE_SD_CARD)
            // Write the samples to the WAV file
            fwrite(i2s_readraw_buff, bytes_read, 1, f);
        #endif
        tatals += bytes_read;

        switch(conv_type){
          case 1:{
            //dl_act=I2S_STD_BUFF_SIZE;
            i2s_std_mono2mono_cp_16bit(dt,dt,dl);
            break;
          }
          case 2:{
            dl_act=i2s_std_mono2stero_32000_cp_16bit(dt,dt,dl);
            break;
          }
          case 0:
            //dl_act=I2S_STD_BUFF_SIZE;
            //memcpy(dt,ptr,I2S_STD_BUFF_SIZE);
            break;
        }
        break;
    } 
    else {
        printf("Read Failed!\n");
        break;
    }
  }

  //i2s_read((i2s_port_t)1, (void *)dt, dl,&bytes_read, 10);
  //if (bytes_read <= 0 || dl != bytes_read) {
  //  ESP_LOGE(TAG, "Error in I2S read : %d", bytes_read);
  //}
  return bytes_read;
}
#endif

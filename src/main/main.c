#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <math.h>
#include <dirent.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/uart.h"

#include "tinyusb.h"

#include "esp_timer.h"


#define PIN_MOSI (35)
#define PIN_CLK  (36)
#define PIN_DACS (38)


#define PIN_OP (1)
#define PIN_OM (37)
#define PIN_CH (48)
#define PIN_BO (21)

#define MIDI_NOTE_OFF 0x80
#define MIDI_NOTE_ON  0x90

#define N_KEY 24

typedef struct key_st{
  uint8_t pitch;
  uint8_t flag;
  uint16_t onoff;
  uint16_t plyhd;
} KEY_t;

KEY_t keys[N_KEY]={0};

#define SAMP_RATE 8000

#define SAMP_PER_KEY 16000

#define TAPER 1000

spi_device_handle_t spi;

#define N_TBTN 24
static const int tbtn[N_TBTN] = {
  10,11,12,13,2,42,41,40,39,45,47,14,
  4,5,6,7,15,16,17,18,8,3,46,9
};



uint8_t tbtn_prev[N_TBTN] = {0};

int op_prv = 1;
int om_prv = 1;

int ch_prv = 1;
int bo_prv = 1;

#define GETSAMP(p) {\
  int idx = (((p)-21)*SAMP_PER_KEY+b)*2;\
  int A = ((uint8_t*)salaman)[idx];\
  int B = ((uint8_t*)salaman)[idx+1];\
  int16_t C = (A|(B<<8));\
  y = (float)C/32768.0;\
}


enum interface_count {
  ITF_NUM_MIDI = 0,
  ITF_NUM_MIDI_STREAMING,
  ITF_COUNT
};
enum usb_endpoints {
  EP_EMPTY = 0,
  EPNUM_MIDI,
};
#define TUSB_DESCRIPTOR_TOTAL_LEN (TUD_CONFIG_DESC_LEN + CFG_TUD_MIDI * TUD_MIDI_DESC_LEN)
static const char* s_str_desc[5] = {
  (char[]){0x09, 0x04},  // 0: is supported language is English (0x0409)
  "Lingdong Huang",      // 1: Manufacturer
  "Lingcordion",         // 2: Product
  "123456",              // 3: Serials, should use chip ID
  "MIDI device",         // 4: MIDI
};

static const uint8_t s_midi_cfg_desc[] = {
  TUD_CONFIG_DESCRIPTOR(1, ITF_COUNT, 0, TUSB_DESCRIPTOR_TOTAL_LEN, 0, 100),
  TUD_MIDI_DESCRIPTOR(ITF_NUM_MIDI, 4, EPNUM_MIDI, (0x80 | EPNUM_MIDI), 64),
};

void midi_note_on(int pitch, int vel){
  if (tud_midi_mounted()) {
    static uint8_t const cable_num = 0;
    static uint8_t const channel = 0;
    uint8_t note_on[3] = {MIDI_NOTE_ON | channel, pitch, vel};
    tud_midi_stream_write(cable_num, note_on, 3);
  }
}
void midi_note_off(int pitch){
  if (tud_midi_mounted()) {
    static uint8_t const cable_num = 0;
    static uint8_t const channel = 0;
    uint8_t note_off[3] = {MIDI_NOTE_OFF | channel, pitch, 0};
    tud_midi_stream_write(cable_num, note_off, 3);
  }
}

void core1_task(){
  tinyusb_config_t const tusb_cfg = {
    .device_descriptor = NULL,
    .string_descriptor = s_str_desc,
    .external_phy = false,
    .configuration_descriptor = s_midi_cfg_desc,
  };
  ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

  int idx = 0;
  uint8_t btn_prev[N_TBTN] = {0};
  int pit_prev[N_TBTN] = {0};

  while (1){
    int op = gpio_get_level(PIN_OP);
    int om = gpio_get_level(PIN_OM);
    int ch = gpio_get_level(PIN_CH);
    int bo = gpio_get_level(PIN_BO);

    int touch_value = gpio_get_level(tbtn[idx]);

    if (!touch_value && btn_prev[idx]){
      int p = 48+idx;
      if (idx < 12){
        if (!ch){
          midi_note_on(p,96);
          midi_note_on(p+4,96);
          midi_note_on(p+7,96);
          pit_prev[idx] = 0x100|p;
        }else if (!bo){
          midi_note_on(p,96);
          midi_note_on(p+3,96);
          midi_note_on(p+7,96);
          pit_prev[idx] = 0x200|p;
        }else{
          midi_note_on(p,96);
          pit_prev[idx] = p;
        }
      }else{
        if (!op){
          p += 12;
        }else if (!om){
          p -= 12;
        }
        midi_note_on(p,127);
        pit_prev[idx] = p;
      }

    }else if (touch_value  && !btn_prev[idx]){
      int p  = pit_prev[idx];
      if (p & 0x100){
        midi_note_off(p);
        midi_note_off(p+4);
        midi_note_off(p+7);
      }else if (p & 0x200){
        midi_note_off(p);
        midi_note_off(p+3);
        midi_note_off(p+7);
      }else{
        midi_note_off(p);
      }
    }
    btn_prev[idx] = touch_value;
    idx = (idx+1)%N_TBTN;
  }
}


void app_main(void){  


  const esp_partition_t* datapart = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_UNDEFINED, "data");

  const void* salaman;
  esp_partition_mmap_handle_t handle;
  ESP_ERROR_CHECK(esp_partition_mmap(
      datapart,
      0,
      2848000,
      ESP_PARTITION_MMAP_DATA,
      &salaman,
      &handle
  ));

  for (int i = 0; i < N_TBTN; i++) {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << tbtn[i]);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;     
    gpio_config(&io_conf);

    tbtn_prev[i] = 1;
  }


  esp_err_t ret;
  spi_bus_config_t buscfg={
    .miso_io_num=-1,
    .mosi_io_num=PIN_MOSI,
    .sclk_io_num=PIN_CLK,
    .quadwp_io_num=-1,
    .quadhd_io_num=-1,
    .max_transfer_sz=32
  };
  spi_device_interface_config_t devcfg={
    .clock_speed_hz=20*1000*1000, 
    .mode=0,                               
    .spics_io_num=PIN_DACS,  
    .flags=0,
    .queue_size=7,                      
  };
  ret=spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
  ESP_ERROR_CHECK(ret);
  ret=spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
  ESP_ERROR_CHECK(ret);


  {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << PIN_OP);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;     
    gpio_config(&io_conf);
  }{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << PIN_OM);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;     
    gpio_config(&io_conf);
  }{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << PIN_CH);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;     
    gpio_config(&io_conf);
  }{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << PIN_BO);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;     
    gpio_config(&io_conf);
  }

  uint8_t buf[2];
  spi_transaction_t tr;
  memset(&tr,0,sizeof(tr));
  tr.tx_buffer = buf;
  tr.length = 16;
  tr.flags = 0;  

  int tidx = 0;

  for (int i = 0; i < 24; i++){
    keys[i].pitch = 48+i;
  }


  xTaskCreatePinnedToCore(core1_task, "Core 1", 16384, NULL, 6, NULL, 1);

  while (1){
    int64_t start_time = esp_timer_get_time();

    int op = gpio_get_level(PIN_OP);
    int om = gpio_get_level(PIN_OM);
    int ch = gpio_get_level(PIN_CH);
    int bo = gpio_get_level(PIN_BO);

    float mag = 0;
    for (int j = 0; j < N_KEY; j++){
      if (keys[j].onoff){
        int b = keys[j].plyhd;
        float y = 0;
        if (b < SAMP_PER_KEY){
          int p = keys[j].pitch;
          if (j >= 12){
            if (!op){
              p += 12;
            }
            if (!om){
              p -= 12;
            }
          }
          if (j < 12 && !ch){
            GETSAMP(p);
            y *= 0.66*(float)keys[j].onoff/(float)TAPER;
            mag += y;

            GETSAMP(p+4);
            y *= 0.66*(float)keys[j].onoff/(float)TAPER;
            mag += y;

            GETSAMP(p+7);
            y *= 0.66*(float)keys[j].onoff/(float)TAPER;
            mag += y;

          }else if (j < 12 && !bo){
            GETSAMP(p);
            y *= 0.66*(float)keys[j].onoff/(float)TAPER;
            mag += y;

            GETSAMP(p+3);
            y *= 0.66*(float)keys[j].onoff/(float)TAPER;
            mag += y;

            GETSAMP(p+7);
            y *= 0.66*(float)keys[j].onoff/(float)TAPER;
            mag += y;

          }else{
            GETSAMP(p);
            y *= (float)keys[j].onoff/(float)TAPER;
            if (j < 12){
              y *= 0.66;
            }else{
              y *= 0.9;
            }
            mag += y;
          }
          if (keys[j].onoff < TAPER){
            keys[j].onoff--;
          }
        }else{
          keys[j].onoff = 0;
        }

      }
      keys[j].plyhd++;
    }

    uint32_t vvv = (fmin(fmax(mag,-1),1))*2047+2047;
    buf[0] = 0b00110000 | ((vvv>>8)&0xf);
    buf[1] = vvv&0xff;
    spi_device_polling_transmit(spi, &tr);

    int touch_value = gpio_get_level(tbtn[tidx]);





    if (!touch_value && tbtn_prev[tidx]){
      keys[tidx].onoff = TAPER;
      keys[tidx].plyhd = 0;
    }else if (touch_value  && !tbtn_prev[tidx]){
      if (keys[tidx].onoff == TAPER){
        keys[tidx].onoff = TAPER-1;
      }
    }
    tbtn_prev[tidx] = touch_value;
    tidx = (tidx+1)%N_TBTN;

    int64_t dt = esp_timer_get_time() - start_time;
    // printf("%d\n",ar);
    int wait = 125;

    if (dt > wait){
      printf("violation %lld\n", dt);
    }else{
      esp_rom_delay_us(wait-dt);
    }
  }


}





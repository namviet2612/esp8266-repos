#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#define I2C_MASTER_SCL_IO           2                /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO           14               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0        /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE   0                /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                /*!< I2C master do not need buffer */
#define I2C_SLAVE_OLED_ADDR         0x3C
#define I2C_OLED_D_C_COMMAND        0               /* D/C# indicates that next bytes will be commands */
#define I2C_OLED_D_C_DATA           1               /* D/C# indicates that next bytes will be data */
#define I2C_OLED_C0_SINGLE_MODE     0               /* Co = 0: No more control bytes are expected in this I2C transmission */
#define I2C_OLED_C0_CONTINUOUS_MODE     1           /* C0 = 1: Another control byte will follow in the same I2C transmission */

// Command definition
// ------------------------------------------------------------------------------------
#define SSD1306_COMMAND 0x80        // Continuation bit=1, D/C=0; 1000 0000
#define SSD1306_COMMAND_STREAM 0x00 // Continuation bit=0, D/C=0; 0000 0000
#define SSD1306_DATA 0xC0           // Continuation bit=1, D/C=1; 1100 0000
#define SSD1306_DATA_STREAM 0x40    // Continuation bit=0, D/C=1; 0100 0000

#define SSD1306_SET_MUX_RATIO 0xA8    // Set MUX ratio to N+1 MUX, N=A[5:0] : from 16MUX to 64MUX
#define SSD1306_DISPLAY_OFFSET 0xD3   // Set Display Offset
#define SSD1306_DISPLAY_ON 0xAF       // Display ON in normal mode
#define SSD1306_DISPLAY_OFF 0xAE      // Display OFF (sleep mode)
#define SSD1306_DIS_ENT_DISP_ON 0xA4  // Entire Display ON, Output ignores RAM content
#define SSD1306_DIS_IGNORE_RAM 0xA5   // Resume to RAM content display, Output follows RAM content
#define SSD1306_DIS_NORMAL 0xA6       // Normal display, 0 in RAM: OFF in display panel, 1 in RAM: ON in display panel
#define SSD1306_DIS_INVERSE 0xA7      // Inverse display, 0 in RAM: ON in display panel, 1 in RAM: OFF in display panel
#define SSD1306_DEACT_SCROLL 0x2E     // Stop scrolling that is configured by command 26h/27h/29h/2Ah
#define SSD1306_ACTIVE_SCROLL 0x2F    // Start scrolling that is configured by the scrolling setup commands:26h/27h/29h/2Ah
#define SSD1306_SET_START_LINE 0x40   // Set Display Start Line
#define SSD1306_MEMORY_ADDR_MODE 0x20 // Set Memory, Addressing Mode
#define SSD1306_SET_COLUMN_ADDR 0x21  // Set Column Address
#define SSD1306_SET_PAGE_ADDR 0x22    // Set Page Address
#define SSD1306_SEG_REMAP 0xA0        // Set Segment Re-map, X[0]=0b column address 0 is mapped to SEG0
#define SSD1306_SEG_REMAP_OP 0xA1     // Set Segment Re-map, X[0]=1b: column address 127 is mapped to SEG0
#define SSD1306_COM_SCAN_DIR 0xC0     // Set COM Output, X[3]=0b: normal mode (RESET) Scan from COM0 to COM[N –1], e N is the Multiplex ratio
#define SSD1306_COM_SCAN_DIR_OP 0xC8  // Set COM Output, X[3]=1b: remapped mode. Scan from COM[N-1] to COM0, e N is the Multiplex ratio
#define SSD1306_COM_PIN_CONF 0xDA     // Set COM Pins Hardware Configuration,
                                      // A[4]=0b, Sequential COM pin configuration, A[4]=1b(RESET), Alternative COM pin configuration
                                      // A[5]=0b(RESET), Disable COM Left/Right remap, A[5]=1b, Enable COM Left/Right remap
#define SSD1306_SET_CONTRAST 0x81     // Set Contrast Control, Double byte command to select 1 to 256 contrast steps, increases as the value increases
#define SSD1306_SET_OSC_FREQ 0xD5     // Set Display Clock Divide Ratio/Oscillator Frequency
                                      // A[3:0] : Define the divide ratio (D) of the  display clocks (DCLK): Divide ratio= A[3:0] + 1, RESET is 0000b (divide ratio = 1)
                                      // A[7:4] : Set the Oscillator Frequency, FOSC. Oscillator Frequency increases with the value of A[7:4] and vice versa. RESET is 1000b
#define SSD1306_SET_CHAR_REG 0x8D     // Charge Pump Setting, A[2] = 0b, Disable charge pump(RESET), A[2] = 1b, Enable charge pump during display on
                                      // The Charge Pump must be enabled by the following command:
                                      // 8Dh ; Charge Pump Setting
                                      // 14h ; Enable Charge Pump
                                      // AFh; Display ON
#define SSD1306_SET_PRECHARGE 0xD9    // Set Pre-charge Period
#define SSD1306_VCOM_DESELECT 0xDB    // Set VCOMH Deselect Leve
#define SSD1306_NOP 0xE3              // No operation
#define SSD1306_RESET 0xE4            // Maybe SW RESET, @source https://github.com/SmingHub/Sming/issues/501

// AREA definition
// ------------------------------------------------------------------------------------
#define START_PAGE_ADDR 0
#define END_PAGE_ADDR 7 // 7 for 128x64, 3 for 128x32 version
#define START_COLUMN_ADDR 0
#define END_COLUMN_ADDR 127
#define RAM_X_END END_COLUMN_ADDR + 1
#define RAM_Y_END END_PAGE_ADDR + 1

#define CACHE_SIZE_MEM (1 + END_PAGE_ADDR) * (1 + END_COLUMN_ADDR)

#define MAX_X END_COLUMN_ADDR
#define MAX_Y (END_PAGE_ADDR + 1) * 8

// @var array Chache memory Lcd 8 * 128 = 1024
static uint8_t cacheMemLcd[CACHE_SIZE_MEM];
static uint8_t _counter;

typedef struct 
{
    uint8_t* data;
    uint8_t data_len;
}i2c_oled_command;

uint8_t ssd1306_display_off[1U] = {SSD1306_DISPLAY_OFF};
uint8_t ssd1306_set_mux_ratio[2U] = {SSD1306_SET_MUX_RATIO, 0x3F};
uint8_t ssd1306_memory_addr_mode[2U] = {SSD1306_MEMORY_ADDR_MODE, 0x00};
uint8_t ssd1306_set_start_line[1U] = {SSD1306_SET_START_LINE};
uint8_t ssd1306_display_offset[2U] = {SSD1306_DISPLAY_OFFSET, 0x00};
uint8_t ssd1306_seg_remap_op[1U] = {SSD1306_SEG_REMAP_OP};
uint8_t ssd1306_com_scan_dir_op[1U] = {SSD1306_COM_SCAN_DIR_OP};
uint8_t ssd1306_com_pin_conf[2U] = {SSD1306_COM_PIN_CONF, 0x12};
uint8_t ssd1306_set_constrast[2U] = {SSD1306_SET_CONTRAST, 0x7F};
uint8_t ssd1306_dis_ent_disp_on[1U] = {SSD1306_DIS_ENT_DISP_ON};
uint8_t ssd1306_dis_normal[1U] = {SSD1306_DIS_ENT_DISP_ON};
uint8_t ssd1306_set_osc_freq[2U] = {SSD1306_SET_OSC_FREQ, 0x80};
uint8_t ssd1306_set_precharge[2U] = {SSD1306_SET_PRECHARGE, 0xC2};
uint8_t ssd1306_vcom_deselect[2U] = {SSD1306_VCOM_DESELECT, 0x20};
uint8_t ssd1306_set_char_reg[2U] = {SSD1306_SET_CHAR_REG, 0x14};
uint8_t ssd1306_deact_scroll[1U] = {SSD1306_DEACT_SCROLL};
uint8_t ssd1306_display_on[1U] = {SSD1306_DISPLAY_ON};
uint8_t ssd1306_dis_inverse[1U] = {SSD1306_DIS_INVERSE};

i2c_oled_command i2c_oled_init_commands[] = {
    {   ssd1306_display_off,        1U  },
    {   ssd1306_set_mux_ratio,      2U  },
    {   ssd1306_memory_addr_mode,   2U  },
    {   ssd1306_set_start_line,     1U  },
    {   ssd1306_display_offset,     2U  },
    {   ssd1306_seg_remap_op,       1U  },          
    {   ssd1306_com_scan_dir_op,    1U  },
    {   ssd1306_com_pin_conf,       2U  },
    {   ssd1306_set_constrast,      2U  },
    {   ssd1306_dis_ent_disp_on,    1U  },
    {   ssd1306_dis_normal,         1U  },
    {   ssd1306_set_osc_freq,       2U  },
    {   ssd1306_set_precharge,      2U  },
    {   ssd1306_vcom_deselect,      2U  },
    {   ssd1306_set_char_reg,       2U  },
    {   ssd1306_deact_scroll,       1U  },
    {   ssd1306_display_on,         1U  }
};
/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init()
{
    int i2c_master_port = I2C_NUM_0;
    esp_err_t ret;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = 1;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = 1;
    conf.clk_stretch_tick = 300; // 300 ticks, Clock stretch is about 210us, you can make changes according to the actual situation.
    ret = i2c_driver_install(i2c_master_port, conf.mode);
    if (ret != ESP_OK)
    {
        printf("I2C driver installation failed, error code: [%#x]\n", ret);
        return ret;
    }
    ret = i2c_param_config(i2c_master_port, &conf);
    if (ret != ESP_OK)
    {
        printf("I2C driver param config failed, error code: [%#x]\n", ret);
        return ret;
    }
    return ESP_OK;
}

static esp_err_t oled_write_cmd(i2c_oled_command i2c_cmd)
{
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, I2C_SLAVE_OLED_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, SSD1306_COMMAND_STREAM, true);
    i2c_master_write(cmd, i2c_cmd.data, i2c_cmd.data_len, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    if (ret != ESP_OK)
    {
        printf("I2C command [%02x] failed, error code: [%#x]\n", i2c_cmd.data[0], ret);
    }
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t SSD1306_DrawPixel (uint8_t x, uint8_t y)
{
  uint8_t page = 0;
  uint8_t pixel = 0;
  
  if ((x > MAX_X) || (y > MAX_Y)) {                               // if out of range
    return ESP_ERR_INVALID_ARG;                                         // out of range
  }
  page = y >> 3;                                                  // find page (y / 8)
  pixel = 1 << (y - (page << 3));                                 // which pixel (y % 8)
  _counter = x + (page << 7);                                     // update counter
  cacheMemLcd[_counter++] |= pixel;                               // save pixel

  return ESP_OK;
}

esp_err_t SSD1306_DrawLine (uint8_t x1, uint8_t x2, uint8_t y1, uint8_t y2)
{
  int16_t D;                                                      // determinant
  int16_t delta_x, delta_y;                                       // deltas
  int16_t trace_x = 1, trace_y = 1;                               // steps

  delta_x = x2 - x1;                                              // delta x
  delta_y = y2 - y1;                                              // delta y
  
  if (delta_x < 0) {                                              // check if x2 > x1
    delta_x = -delta_x;                                           // negate delta x
    trace_x = -trace_x;                                           // negate step x
  }
  
  if (delta_y < 0) {                                              // check if y2 > y1
    delta_y = -delta_y;                                           // negate detla y
    trace_y = -trace_y;                                           // negate step y
  }

  // Bresenham condition for m < 1 (dy < dx)
  // -------------------------------------------------------------------------------------
  if (delta_y < delta_x) {
    D = (delta_y << 1) - delta_x;                                 // calculate determinant
    SSD1306_DrawPixel (x1, y1);                                   // draw first pixel
    while (x1 != x2) {                                            // check if x1 equal x2
      x1 += trace_x;                                              // update x1
      if (D >= 0) {                                               // check if determinant is positive
        y1 += trace_y;                                            // update y1
        D -= 2*delta_x;                                           // update determinant
      }
      D += 2*delta_y;                                             // update deteminant
      SSD1306_DrawPixel (x1, y1);                                 // draw next pixel
    }
  // for m > 1 (dy > dx)    
  // -------------------------------------------------------------------------------------
  } else {
    D = delta_y - (delta_x << 1);                                 // calculate determinant
    SSD1306_DrawPixel (x1, y1);                                   // draw first pixel
    while (y1 != y2) {                                            // check if y2 equal y1
      y1 += trace_y;                                              // update y1
      if (D <= 0) {                                               // check if determinant is positive
        x1 += trace_x;                                            // update y1
        D += 2*delta_y;                                           // update determinant
      }
      D -= 2*delta_x;                                             // update deteminant
      SSD1306_DrawPixel (x1, y1);                                 // draw next pixel
    }
  }

  return ESP_OK;
}

void SSD1306_ClearScreen (void)
{
  memset (cacheMemLcd, 0x00, CACHE_SIZE_MEM);                     // null cache memory lcd
}

esp_err_t oled1306_display_reverse()
{
    esp_err_t ret;
    i2c_oled_command i2c_oled_init_commands[] = 
    {
        {   ssd1306_dis_inverse,        1U  }
    };
    ret = oled_write_cmd(i2c_oled_init_commands[0U]);
    return ret;
}

esp_err_t oled1306_update_screen(void)
{
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, I2C_SLAVE_OLED_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, SSD1306_DATA_STREAM, true);
    i2c_master_write(cmd, cacheMemLcd, CACHE_SIZE_MEM, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    if (ret != ESP_OK)
    {
        printf("I2C update screen failed, error code: [%#x]\n", ret);
    }
    i2c_cmd_link_delete(cmd);
    return ret;
}

void oled1306_init(void)
{
    /* I2C master init */
    i2c_master_init();

    uint8_t numberOfSSD1306Cmds = sizeof(i2c_oled_init_commands)/sizeof(i2c_oled_init_commands[0]);

    for (uint8_t cnt = 0; cnt < numberOfSSD1306Cmds; cnt++)
    {
        oled_write_cmd(i2c_oled_init_commands[cnt]);
    }
}

void app_main(void)
{
    oled1306_init();
    SSD1306_ClearScreen(); 
    oled1306_update_screen();
    oled1306_display_reverse();
    while (1) {

    }
}

/*
 * ucglib Demo Application running on Nordic Semiconductors nRF52 DK with nRF SDK 17.02 and:
 *
 * SPIM and a Waveshare SSD1351 OLED 128x128 18 Bit color (https://www.waveshare.com/wiki/1.5inch_RGB_OLED_Module)
 * that can be found on Ebay/Amazon for few EUR.
 * 
 * Created 20200116 by @daubsi, based on the great ucglib library https://github.com/olikraus/ucglib
 * Probably not the cleanest code but it works :-D
 * 
 * SPI: Setup up in SPIM with 4 MHz in non-blocking mode
 *
 * Wiring SPI:
 * Connect VCC and GND of the diplay with VDD and GND on the NRF52 DK
 * Connect CLK (yellow wire) to pin 27 ("P0.27") and DIN (blue wire) to pin 26 ("P0.26") on the nRF52 DK
 * Connect CS (orange wire) to pin 12 ("P0.12") and DC (green wire) to pin 11 ("P0.11") on the nRF52 DK
 * Connect Reset (white wire) to pin 31 ("P0.31") on the nRF52 DK
 *
 * Settings in sdk_config.h
 *
 * SWI:
 * #define NRFX_SPIM_ENABLED 1
 * #define NRFX_SPIM0_ENABLED 0
 * #define NRFX_SPIM1_ENABLED 1
 * #define NRFX_SPIM_EXTENDED_ENABLED 0 // Only available on NRF52840 on SPIM3
 * #define NRFX_SPIM_MISO_PULL_CFG 1
 * #define NRFX_SPIM_DEFAULT_CONFIG_IRQ_PRIORITY 6
 * 
 * The ucglib sources have to be unpacked and the csrc subfolder of that archive should be placed in the main directory of this project in a folder ucglib.
 */


#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"

#include <nrfx_spim.h>

#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ucg.h"

#define SPI_INSTANCE 1

static const nrfx_spim_t m_spi = NRFX_SPIM_INSTANCE(SPI_INSTANCE);

#define NRFX_SPIM_SCK_PIN  27 // yellow wire (CLK)
#define NRFX_SPIM_MOSI_PIN 26 // blue wire (DIN)
#define NRFX_SPIM_CS_PIN 12 // orange wire (CS)
#define NRFX_SPIM_DC_PIN 11 // green wire (DC)
#define NRFX_SPIM_RESET_PIN 31 // white wire (Reset)

void spi_handler(nrfx_spim_evt_t const * p_event, void * p_context);
int16_t ucg_HW_com_spi_nrf52832(ucg_t *ucg, int16_t msg, uint16_t arg, uint8_t *data);

/* Indicates if operation has ended. */
static volatile bool m_xfer_done = false;
static uint8_t m_sample;

static ucg_t ucg;

static bool readsomething = false;

void spi_init (void)
{
    ret_code_t err_code;

    nrfx_spim_config_t spi_oled_config = NRFX_SPIM_DEFAULT_CONFIG;

    spi_oled_config.sck_pin   = NRFX_SPIM_SCK_PIN;
    spi_oled_config.mosi_pin  = NRFX_SPIM_MOSI_PIN;
    spi_oled_config.ss_pin    = NRFX_SPIM_CS_PIN;
    spi_oled_config.frequency = NRF_SPIM_FREQ_4M;
    spi_oled_config.mode      = NRF_SPIM_MODE_0;
    spi_oled_config.ss_active_high = false;
       
    err_code = nrfx_spim_init(&m_spi, &spi_oled_config, spi_handler, NULL);    
    APP_ERROR_CHECK(err_code);

    // Enable the out-of-band GPIOs
    nrf_gpio_cfg_output(NRFX_SPIM_DC_PIN);
    nrf_gpio_cfg_output(NRFX_SPIM_RESET_PIN);
    
}

void spi_handler(nrfx_spim_evt_t const * p_event, void * p_context)
{
    switch(p_event->type)
    {
        case NRFX_SPIM_EVENT_DONE:
                m_xfer_done = true;
                break;
    }
}

int16_t ucg_HW_com_spi_nrf52832(ucg_t *ucg, int16_t msg, uint16_t arg, uint8_t *data)
{
static uint8_t buffer[3];
static uint8_t buf_idx = 1;
ret_code_t err_code;    
  switch(msg)
  {
    case UCG_COM_MSG_POWER_UP:
      /* "data" is a pointer to ucg_com_info_t structure with the following information: */
      /*	((ucg_com_info_t *)data)->serial_clk_speed value in nanoseconds */
      /*	((ucg_com_info_t *)data)->parallel_clk_speed value in nanoseconds */
      /* "arg" is not used */

      /* This message is sent once at the uC startup and for power up. */
      /* setup i/o or do any other setup */
      // NRF_LOG_INFO("UCG_COM_MSG_POWER_UP");
      break;

    case UCG_COM_MSG_POWER_DOWN:
      /* "data" and "arg" are not used*/
      /* This message is sent for a power down request */
      // NRF_LOG_INFO("UCG_COM_MSG_POWER_DOWN");
      break;
    case UCG_COM_MSG_DELAY:
      /* "data" is not used */
      /* "arg" contains the number of microseconds for the delay */
      /* By receiving this message, the following code should delay by */
      /* "arg" microseconds. One microsecond is 0.000001 second */
      nrf_delay_us(arg);
      // NRF_LOG_INFO("UCG_COM_MSG_DELAY: %d ms", arg);
      break;
    case UCG_COM_MSG_CHANGE_RESET_LINE:
      /* "data" is not used */
      /* "arg" = 1: set the reset output line to 1 */
      /* "arg" = 0: set the reset output line to 0 */
      nrf_gpio_pin_write(NRFX_SPIM_RESET_PIN, arg);  
      // NRF_LOG_INFO("UCG_COM_MSG_CHANGE_RESET_LINE");
      break;
    case UCG_COM_MSG_CHANGE_CD_LINE:
      /* "ucg->com_status"  bit 0 contains the old level for the CD line */
      /* "data" is not used */
      /* "arg" = 1: set the command/data (a0) output line to 1 */
      /* "arg" = 0: set the command/data (a0) output line to 0 */
      nrf_gpio_pin_write(NRFX_SPIM_DC_PIN, arg);  
      // NRF_LOG_INFO("UCG_COM_MSG_CHANGE_CD_LINE: 0x%02x", arg);
      break;
    case UCG_COM_MSG_CHANGE_CS_LINE:
      /* "ucg->com_status"  bit 1 contains the old level for the CS line */
      /* "data" is not used */
      /* "arg" = 1: set the chipselect output line to 1 */
      /* "arg" = 0: set the chipselect output line to 0 */
      //NRF_LOG_INFO("UCG_COM_MSG_CHANGE_CS_LINE: 0x%02x", arg");
      break;
    case UCG_COM_MSG_SEND_BYTE:
      /* "data" is not used */
      /* "arg" contains one byte, which should be sent to the display */
      /* The current status of the CD line is available */
      /* in bit 0 of u8g->com_status */
        
      buffer[0] = arg;
      m_xfer_done = false;
      nrfx_spim_xfer_desc_t spim_xfer_desc = NRFX_SPIM_XFER_TX(&buffer, 1);
      err_code = nrfx_spim_xfer(&m_spi, &spim_xfer_desc,0);
      APP_ERROR_CHECK(err_code);
      while (!m_xfer_done)
      {
          __WFE();
      }
      //NRF_LOG_INFO("UCG_COM_MSG_SEND_BYTE: 0x%02x", arg);
      break;
    case UCG_COM_MSG_REPEAT_1_BYTE:
      /* "data[0]" contains one byte */
      /* repeat sending the byte in data[0] "arg" times */
      /* The current status of the CD line is available */
      /* in bit 0 of u8g->com_status */
      NRF_LOG_ERROR("UCG_COM_MSG_REPEAT_1_BYTE: Not implemented");
      break;
    case UCG_COM_MSG_REPEAT_2_BYTES:
      /* "data[0]" contains first byte */
      /* "data[1]" contains second byte */
      /* repeat sending the two bytes "arg" times */
      /* The current status of the CD line is available */
      /* in bit 0 of u8g->com_status */
      NRF_LOG_ERROR("UCG_COM_MSG_REPEAT_2_BYTES: Not implemented!");
      break;
    case UCG_COM_MSG_REPEAT_3_BYTES:
      /* "data[0]" contains first byte */
      /* "data[1]" contains second byte */
      /* "data[2]" contains third byte */
      /* repeat sending the three bytes "arg" times */
      /* The current status of the CD line is available */
      /* in bit 0 of u8g->com_status */
      buffer[0] = data[0];
      buffer[2] = data[1];
      buffer[2] = data[2];
      for (int idx=0; idx<arg; idx++)
      {
        m_xfer_done = false;
        nrfx_spim_xfer_desc_t spim_xfer_desc = NRFX_SPIM_XFER_TX(&buffer, 3);
        err_code = nrfx_spim_xfer(&m_spi, &spim_xfer_desc,0);
        APP_ERROR_CHECK(err_code);
        while (!m_xfer_done)
        {
            __WFE();
        }
      }
      // NRF_LOG_INFO("UCG_COM_MSG_REPEAT_3_BYTES: 0x%02x, 0x%02x, 0x%02x", buffer[0], buffer[1], buffer[2]);
      break;
    case UCG_COM_MSG_SEND_STR:
      /* "data" is an array with "arg" bytes */
      /* send "arg" bytes to the display */
      NRF_LOG_ERROR("UCG_COM_MSG_SEND_STR: Not implemented!");

      break;
    case UCG_COM_MSG_SEND_CD_DATA_SEQUENCE:
      /* "data" is a pointer to two bytes, which contain the cd line */
      /* status and display data */
      /* "arg" contains the number of these two byte tuples which need to */
      /* be analysed and sent. Bellow is a example sequence */
      /* The content of bit 0 in u8g->com_status is undefined for this message */

      //NRF_LOG_INFO("UCG_COM_MSG_SEND_CD_DATA_SEQUENCE");
      while(arg > 0)
      {
	if ( *data != 0 )
	{
	  if ( *data == 1 )
	  {
            nrf_gpio_pin_write(NRFX_SPIM_DC_PIN, 0);            
	  }
	  else
	  {
            nrf_gpio_pin_write(NRFX_SPIM_DC_PIN, 1);
	  }
	}
	data++;
        
        *buffer = *data;
        m_xfer_done = false;
        nrfx_spim_xfer_desc_t spim_xfer_desc = NRFX_SPIM_XFER_TX(&buffer, 1);
        err_code = nrfx_spim_xfer(&m_spi, &spim_xfer_desc,0);
        APP_ERROR_CHECK(err_code);
        while (!m_xfer_done)
        {
            __WFE();
        }
	data++;
	arg--;
      }
      break;
  }
  return 1;
}

void print_hello()
{
static int x = 30;
static int y= 30;
    
    //ucg_ClearBuffer(&ucg);
    ucg_SetColor(&ucg, 0, 255, 255, 255);
    ucg_SetFont(&ucg, ucg_font_ncenR12_tr);
    x = (++x % 20);
    y = (++y % 20);
    ucg_DrawString(&ucg, x,y, 0, "Hello world");
   
    //ucg_DrawStr(&ucg, y, x, "Hello World!");
    //ucg_SendBuffer(&ucg);
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{    
    ret_code_t err_code;
    uint8_t address;
    
    bool detected_device = false;

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    NRF_LOG_INFO("spi_init");

    spi_init();     
    ucg_Init(&ucg, ucg_dev_ssd1351_18x128x128_ft, ucg_ext_ssd1351_18, ucg_HW_com_spi_nrf52832);
    ucg_ClearScreen(&ucg); // Actually blanks out the screen
    
    
    ucg_SetFontMode(&ucg, UCG_FONT_MODE_TRANSPARENT);
    ucg_SetColor(&ucg, 0, 255, 255, 255);
    ucg_SetFont(&ucg, ucg_font_ncenR12_tr);
    ucg_DrawString(&ucg, 20, 20, 0, "Hello world");
    
    NRF_LOG_FLUSH();
}


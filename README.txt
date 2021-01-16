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
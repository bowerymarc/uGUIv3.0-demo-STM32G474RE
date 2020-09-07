/**
 * @file ssd1331.h
 *
 *  Created on: Jun 24, 2020
 *      @author  mlindahl
 */

#ifndef INC_SSD1331_H_
#define INC_SSD1331_H_

/**
 *  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 *
 *  @file     ssd1331.h
 *  @author   Tamas Harczos
 *  @since    2018-12-27
 *  @version  0.1.0
 *  @licence  GNU LGPL v3 (https://www.gnu.org/licenses/lgpl-3.0.txt)
 *
 *  @brief    SSD1331 OLED display driver for nRF52, based on the nRF52 SDK.
 *
 *  The display driver software is pretty much self-containing and only uses
 *  the SPI driver from the nRF5 SDK. It has been tested with version 15.2.
 *  (https://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v15.x.x/).
 *
 *  Project homepage:  https://nrf52-ssd1331.sourceforge.io/
 *  Youtube video:     http://y2u.be/JtX8uTtecR0
 *
 *  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published
 *  by the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 *
**/


#ifndef SSD1331_H
#define SSD1331_H



// *******************************************************************************************************************
// Included Files (from specific to general)
// *******************************************************************************************************************

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

#ifndef max
#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))
#endif

// *******************************************************************************************************************
// Provide C++ Compatibility
// *******************************************************************************************************************

#ifdef __cplusplus
extern "C"
{
#endif



// *******************************************************************************************************************
// Global declarations
// *******************************************************************************************************************

#define SSD1331_WIDTH		96			///< width of the physical display in pixels
#define SSD1331_HEIGHT		64			///< height of the physical display in pixels
#define SSD1331_BPP			2			///< 1 or 2 bytes per pixel, (total, not per channel)
#define SSD1331_MAX_BRIGHT	15			///< max. valid value for the ssd1331_set_brightness() function
#define SSD1331_MAX_CMDLEN	11			///< max bytes in any command

#define ssd1331_width           96UL      ///< width of the physical display in pixels
#define ssd1331_height          64UL      ///< height of the physical display in pixels
#define ssd1331_max_brightness  15        ///< max. valid value for the ssd1331_set_brightness() function


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
typedef struct ssd1331_config_t
{
//  drv_spi_t const * spi;

  uint8_t vcc_pin;                            ///< set to 0xFF to NOT use VCC pin from this driver
  uint8_t scl_pin;                            ///< number of nRF52 GPIO pin where the SCL (SPI clock) pin of the SSD1331 is attached to
  uint8_t sda_pin;                            ///< number of nRF52 GPIO pin where the SDA (SPI data) pin of the SSD1331 is attached to
  uint8_t res_pin;                            ///< number of nRF52 GPIO pin where the RES (reset) pin of the SSD1331 is attached to
  uint8_t dc_pin;                             ///< number of nRF52 GPIO pin where the DC (data/command) pin of the SSD1331 is attached to
  uint8_t cs_pin;                             ///< number of nRF52 GPIO pin where the CS (SPI chip select) pin of the SSD1331 is attached to

  uint8_t bit_depth;                          ///< 8 or 16 bit depth, (total, not per channel)

  bool turbo_mode;                            ///< false: 1 MHz SPI, 0.8 MHz display clock, 3 ms delay after draw commands (recommended at VDD=1.8V); true: 1.6 MHz SPI, 0.9 MHz display clock, 650 us delay after draw commands (recommended at VDD=3.3V)
  bool wait_for_iops;                         ///< if set, module functions making use of SSD1331 accelerated functions (like drawing a filled box) will delay execution to make sure the SSD1331 has the time to finish the job

} ssd1331_config_t;


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
typedef struct ssd1331_image_t
{
  uint8_t width;                              ///< width of the image[s] in pixels
  uint8_t height;                             ///< height of the image[s] in pixels
  uint8_t bit_depth;                          ///< bit depth of the image[s], 8 or 16
  uint8_t frames;                             ///< number of frames following in the data field
  uint8_t const* data;                        ///< pointer to the image data array

} ssd1331_image_t;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
typedef struct ssd1331_charset_t
{
  uint8_t width;                              ///< width of the glyphs in pixels
  uint8_t height;                             ///< height of the glyphs in pixels
  uint8_t ascii_map[128];                     ///< index of the array: ASCII code; value in the array: position of the corresponding glyph within 'data'
  uint8_t const* data;                        ///< pointer to the memory area, where the bitmap for the glyphs (characters), put directly next to each other, is stored
  uint8_t num_glyphs;                         ///< number of glyphs in the data array; every row of every glyph takes 'width' bits, the total number of bits is then rounded up to the next whole byte

} ssd1331_charset_t;


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
/// @defgroup SSD1331_CMD Command bytes of the SSD1331 controller
/// @{
#define SSD1331_CMD_SETCOLUMN                        0x15
#define SSD1331_CMD_DRAWLINE                         0x21
#define SSD1331_CMD_DRAWRECT                         0x22
#define SSD1331_CMD_CLEARWINDOW                      0x25
#define SSD1331_CMD_FILL                             0x26
#define SSD1331_CMD_SETROW                           0x75
#define SSD1331_CMD_CONTRASTA                        0x81
#define SSD1331_CMD_CONTRASTB                        0x82
#define SSD1331_CMD_CONTRASTC                        0x83
#define SSD1331_CMD_MASTERCURRENT                    0x87
#define SSD1331_CMD_PRECHARGEA                       0x8A
#define SSD1331_CMD_PRECHARGEB                       0x8B
#define SSD1331_CMD_PRECHARGEC                       0x8C
#define SSD1331_CMD_SETREMAP                         0xA0
#define SSD1331_CMD_NORMALDISPLAY                    0xA4
#define SSD1331_CMD_INVERTDISPLAY                    0xA7
#define SSD1331_CMD_DISPLAYDIMMED                    0xAC
#define SSD1331_CMD_SETMASTER                        0xAD
#define SSD1331_CMD_DISPLAYON                        0xAF
#define SSD1331_CMD_POWERMODE                        0xB0
#define SSD1331_CMD_PHASEADJUST                      0xB1
#define SSD1331_CMD_SETOSCFREQ                       0xB3
#define SSD1331_CMD_PRECHARGELEVEL                   0xBB
/// @}

#define SSD1331_REQUIRED_DELAY_US_AT_3V3             650u     ///< time (in us) required for the SSD1331 to clear the whole screen @ VDD=3.3V (this is an empirically found value; unfortunately, the documentation does not specify it; higher is safer)
#define SSD1331_REQUIRED_DELAY_US_AT_1V8             3000u    ///< time (in us) required for the SSD1331 to clear the whole screen @ VDD=1.8V (this is an empirically found value; unfortunately, the documentation does not specify it; higher is safer)


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
/// @defgroup SSD1331_COLORS Color conversion macros and color palette enums (holding the Commodore 64 original colors, as a tribute to C64)
/// @{
#define SSD1331_RGB_TO_RED_16BIT_MODE(RGB_24BIT)     ((RGB_24BIT & 0b111110000000000000000000) >> 18)
#define SSD1331_RGB_TO_GRN_16BIT_MODE(RGB_24BIT)     ((RGB_24BIT & 0b000000001111110000000000) >> 10)
#define SSD1331_RGB_TO_BLU_16BIT_MODE(RGB_24BIT)     ((RGB_24BIT & 0b000000000000000011111000) >> 2)

#define SSD1331_RGB_TO_16BIT_COLOR_BYTE1(RGB_24BIT)  (((RGB_24BIT & 0b111110000000000000000000) >> 16) | ((RGB_24BIT & 0b000000001110000000000000) >> 13))
#define SSD1331_RGB_TO_16BIT_COLOR_BYTE2(RGB_24BIT)  (((RGB_24BIT & 0b000000000001110000000000) >> 5) | ((RGB_24BIT & 0b000000000000000011111000) >> 3))

#define SSD1331_RGB_TO_16BIT_COLOR(RGB_24BIT)  		 ((SSD1331_RGB_TO_16BIT_COLOR_BYTE1(RGB_24BIT)) | (SSD1331_RGB_TO_16BIT_COLOR_BYTE2(RGB_24BIT) << 8 ))


#define SSD1331_RGB_TO_RED_8BIT_MODE(RGB_24BIT)      ((RGB_24BIT & 0b111000000000000000000000) >> 18)
#define SSD1331_RGB_TO_GRN_8BIT_MODE(RGB_24BIT)      ((RGB_24BIT & 0b000000001110000000000000) >> 10)
#define SSD1331_RGB_TO_BLU_8BIT_MODE(RGB_24BIT)      ((RGB_24BIT & 0b000000000000000011000000) >> 2)

#define SSD1331_RGB_TO_8BIT_COLOR(RGB_24BIT)         (((RGB_24BIT & 0b111000000000000000000000) >> 16) | ((RGB_24BIT & 0b000000001110000000000000) >> 11) | ((RGB_24BIT & 0b000000000000000011000000) >> 6))

#define SSD1331_RGB_COLOR_BLACK                      0x000000
#define SSD1331_RGB_COLOR_WHITE                      0xFFFFFF
#define SSD1331_RGB_COLOR_RED                        0x880000
#define SSD1331_RGB_COLOR_CYAN                       0xAAFFEE
#define SSD1331_RGB_COLOR_VIOLET                     0xCC44CC
#define SSD1331_RGB_COLOR_GREEN                      0x00CC55
#define SSD1331_RGB_COLOR_BLUE                       0x0000AA
#define SSD1331_RGB_COLOR_YELLOW                     0xEEEE77
#define SSD1331_RGB_COLOR_ORANGE                     0xDD8855
#define SSD1331_RGB_COLOR_BROWN                      0x664400
#define SSD1331_RGB_COLOR_LIGHTRED                   0xFF7777
#define SSD1331_RGB_COLOR_DARKGREY                   0x333333
#define SSD1331_RGB_COLOR_GREY                       0x777777
#define SSD1331_RGB_COLOR_LIGHTGREEN                 0xAAFF66
#define SSD1331_RGB_COLOR_LIGHTBLUE                  0x0088FF
#define SSD1331_RGB_COLOR_LIGHTGREY                  0xBBBBBB
/// @}



// the following must match the SPI peripheral used
extern SPI_HandleTypeDef hspi2;

// image frame
extern ssd1331_image_t ssd1331_frame;

#if (SSD1331_BPP == 1)
extern uint8_t img_buf[];	///< data buf for frame
#endif
#if (SSD1331_BPP == 2)
extern uint16_t img_buf[];	///< data buf for frame
#endif


// *******************************************************************************************************************
// Public functions
// *******************************************************************************************************************

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @brief Event handler for the SPI in use. Only clears the 'spi_transfer_in_progress' flag, which is set by the
///        'spi_transfer()' and 'ssd1331_write_command()' functions every time data is sent over the SPI.
///
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void spi_event_handler(void);


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @brief Initialize or reinitialize the GPIOs and the SPI module of the nRF52 as well as the SSD1331 OLED display.
///
/// @param[in]  ssd1331_cfg  Pointer to the display module configuration structure.
///
/// @return NRFX_SUCCESS if everything went OK, or NRFX error codes upon failure.
///
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef ssd1331_init(void);



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @brief Set the master current (i.e., the brightness) of the SSD1331.
///
/// @param[in]  brightness  Unsigned integer value, max value is 'ssd1331_max_brightness'.
///
/// @note This function should only be called after successful init.
///
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef ssd1331_set_brightness(uint8_t brightness);


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @brief Tune pre-charge voltage and speeds of the SSD1331.
///
/// @param[in]  voltage  Pre-charge voltage. Unsigned integer value in range of 0 to 15.
/// @param[in]  speedA   Pre-charge speed for color A (red).   Unsigned integer value in range of 0 to 255.
/// @param[in]  speedB   Pre-charge speed for color B (green). Unsigned integer value in range of 0 to 255.
/// @param[in]  speedC   Pre-charge speed for color C (blue).  Unsigned integer value in range of 0 to 255.
///
/// @note Refer to SSD1331 specification for details on pre-charge parameters.
/// @note This function should only be called after successful init.
///
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef ssd1331_set_precharge(uint8_t voltage, uint8_t speedA, uint8_t speedB, uint8_t speedC);


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @brief Tune contrast values of the SSD1331.
///
/// @param[in]  contrastA  Contrast value for color A (red).   Unsigned integer value in range of 0 to 255.
/// @param[in]  contrastB  Contrast value for color B (green). Unsigned integer value in range of 0 to 255.
/// @param[in]  contrastC  Contrast value for color C (blue).  Unsigned integer value in range of 0 to 255.
///
/// @note Refer to SSD1331 specification for details on contrast parameters.
/// @note This function should only be called after successful init.
///
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef ssd1331_set_contrast(uint8_t contrastA, uint8_t contrastB, uint8_t contrastC);


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @brief Dim (blank) or undim (unblank) display.
///
/// @note This function should only be called after successful init.
///
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef ssd1331_dim_display(bool dim);


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @brief Clear the area of a specified rectangle of the display, i.e., overwrite contents with black color.
///
/// @note If 'wait_for_iops' was specified in the configuration, this function will delay execution to give the
///       SSD1331 enough time finish drawing.
///
/// @note This function should only be called after successful init.
///
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef ssd1331_clear_window(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @brief Clear the whole screen.
///
/// @note If 'wait_for_iops' was specified in the configuration, this function will delay execution to give the
///       SSD1331 enough time finish drawing.
///
/// @note This function should only be called after successful init.
///
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef ssd1331_clear();


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @brief Draw an (optionally filled) rectangle using SSD1331 hardware acceleration.
///
/// @param[in]  x1          X coordinate (number of column) of the upper left corner of the rectangle.
/// @param[in]  y1          Y coordinate (number of row) of the upper left corner of the rectangle.
/// @param[in]  x2          X coordinate of the bottom right corner of the rectangle.
/// @param[in]  x2          Y coordinate of the bottom right corner of the rectangle.
/// @param[in]  framecolor  24-bit RGB color of the frame of the rectangle.
/// @param[in]  fill        Flag, wether the rectangle should be filled.
/// @param[in]  fillcolor   24-bit RGB color of the filling of the rectangle.
///                         If 'fill' is false, this parameter is ignored.
///
/// @note If 'wait_for_iops' was specified in the configuration, this function will delay execution to give the
///       SSD1331 enough time finish drawing.
///
/// @note This function should only be called after successful init.
///
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef ssd1331_draw_rectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint32_t framecolor, bool fill, uint32_t fillcolor);


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @brief Draw a line using SSD1331 hardware acceleration.
///
/// @param[in]  x1     X coordinate (number of column) of the upper left endpoint of the line.
/// @param[in]  y1     Y coordinate (number of row) of the upper left endpoint of the line.
/// @param[in]  x2     X coordinate of the bottom right endpoint of the line.
/// @param[in]  x2     Y coordinate of the bottom right endpoint of the line.
/// @param[in]  color  24-bit RGB color of the line.
///
/// @note This function should only be called after successful init.
///
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef ssd1331_draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint32_t color);


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @brief Draw an image or a frame of a video by writing into the graphic display data RAM (GDDRAM) of the SSD1331.
///
/// @param[in]  image           Image or image sequence of type 'ssd1331_image_t'.
/// @param[in]  x               X coordinate (number of column) of the upper left corner of the image.
/// @param[in]  y               Y coordinate (number of row) of the upper left corner of the image.
/// @param[in]  num_skip_frame  In case of an image sequence (video), the number of frames to skip. E.g., to display
///                             the 10th frame of a video, set it to 9. In case of a single image, leave it at 0.
///
/// @note The image data to be displayed must match the bit-depth of the currently set color mode.
/// @note This function should only be called after successful init.
///
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef ssd1331_draw_image(ssd1331_image_t const image, uint8_t x, uint8_t y, uint8_t num_skip_frame);


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @brief Draw text by writing into the graphic display data RAM (GDDRAM) of the SSD1331.
///
/// @param[in]  str        Pointer to the C string to be displayed. Will be truncated to the part that fits.
/// @param[in]  x          X coordinate (number of column) of the upper left corner of the text.
/// @param[in]  y          Y coordinate (number of row) of the upper left corner of the text.
/// @param[in]  facecolor  24-bit RGB color of the text face.
/// @param[in]  bg_color   24-bit RGB color of the text background.
/// @param[in]  charset    The character set to be used, e.g. 'ssd1331_charset_5x3'.
///
/// @note This function should only be called after successful init.
///
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef ssd1331_draw_string(uint8_t const* str, uint8_t x, uint8_t y, uint32_t facecolor, uint32_t bg_color, ssd1331_charset_t const charset);


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @brief Helper function to convert a color specified as HSV [Hue/Saturation/Value] parameters into a 24-bit RGB code.
///
/// @note Based on https://stackoverflow.com/questions/3018313/algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb-in-range-0-255-for-both
///
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint32_t ssd1331_hsv2rgb(uint8_t h, uint8_t s, uint8_t v);


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @brief interface to uGUI
///
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void pset(int16_t x, int16_t y, uint32_t col);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @brief send frame buffer to display
///
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ssd1331_update(void);

// *******************************************************************************************************************
// Provide C++ Compatibility
// *******************************************************************************************************************
#ifdef __cplusplus
}
#endif


#endif // SSD1331_H



// *******************************************************************************************************************
// End of File
// *******************************************************************************************************************


#endif /* INC_SSD1331_H_ */

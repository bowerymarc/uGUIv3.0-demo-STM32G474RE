/**
 * @file ssd1331.c
 *
 *  Created on: Jun 24, 2020
 *      @author mlindahl
 *
 *  BASED ON CODE FROM TAMAS HARCZOS:
 *
 *  @author   Tamas Harczos (2018-)
 *  @licence  GNU LGPL v3 (https://www.gnu.org/licenses/lgpl-3.0.txt)
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




// *******************************************************************************************************************
// Included Files (from specific to general)
// *******************************************************************************************************************


#include "ssd1331.h"
#include "main.h"
#include "cmsis_os.h"
#include "stm32g4xx_hal.h"
#include "string.h"


// *******************************************************************************************************************
// Definitions
// *******************************************************************************************************************


#define ssd1331_buffers 2                                           ///< number of TX buffers for the SPI transfer (so that while the data from the first is being transferred by the DMA, we can already fill up the second one)

static bool spi_is_initialized= false;                                  ///< gets true during successful init(), becomes false during destroy()
static volatile bool spi_transfer_in_progress= false;                            ///< every SPI transfers sets this flag and the 'SPI finished' callback clears it

static uint8_t tx_buf[ssd1331_buffers][SSD1331_WIDTH*SSD1331_BPP]= {0};   	///< each large enough to hold a full row of 16-bit pixels

static uint8_t last_tx_buf= 0;                                          ///< which TX buffer has been written last

#define IMG_BUF_SIZE (SSD1331_WIDTH*SSD1331_HEIGHT*1)

#if (SSD1331_BPP == 1)
uint8_t img_buf[IMG_BUF_SIZE];	///< data buf for frame
#endif
#if (SSD1331_BPP == 2)
uint16_t img_buf[IMG_BUF_SIZE];	///< data buf for frame
#endif

ssd1331_image_t ssd1331_frame =
{
		SSD1331_WIDTH,
		SSD1331_HEIGHT,
		(SSD1331_BPP * 8),
		1,	// frame
		(uint8_t *)img_buf
};


static ssd1331_config_t config =
{
  .bit_depth    = SSD1331_BPP * 8,
  .turbo_mode   = true,
  .wait_for_iops= true
};

// *******************************************************************************************************************
// Module internal functions
// *******************************************************************************************************************




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @brief Transfer bytes to the SSD1331 via SPI.
///
/// @param[in]  static_tx_buffer  Pointer to buffer to transfer data from.
/// @param[in]  num_bytes         Number of bytes to transfer from the buffer.
/// @param[in]  is_command        Flag to inficate whether buffer contents belong to an SSD1331 command
///                               or raw data to be written into the SSD1331 memory.
///
/// @note If a previous SPI transaction is ongoing, this function will delay execution and wait for the
///       ongoing SPI transaction to finish first. Should it never finish, the code will hang here.
///
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static HAL_StatusTypeDef spi_transfer(const uint8_t * static_tx_buffer, uint16_t num_bytes, bool is_command)
{
  // wait until previous transfer is in progress
  while (spi_transfer_in_progress);

  // set flag that we are now about to use the SPI
  spi_transfer_in_progress= true;

  // assert or deassert DC pin, depending on whether the bytes we send belong to command[s]
  if (is_command)
	  HAL_GPIO_WritePin(SSD1331_DC_GPIO_Port, SSD1331_DC_Pin, GPIO_PIN_RESET);
  else
	  HAL_GPIO_WritePin(SSD1331_DC_GPIO_Port, SSD1331_DC_Pin, GPIO_PIN_SET);

  // transfer bytes
  return HAL_SPI_Transmit_DMA( &hspi2, static_tx_buffer, num_bytes);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @brief Transfer one command byte to the SSD1331 via SPI.
///
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static HAL_StatusTypeDef ssd1331_write_command(uint8_t command)
{
  static uint8_t tx_command= 0;
  tx_command= command;

  return spi_transfer(&tx_command, 1, true);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @brief Set write pointer of the SSD1331 to a specific pixel specified by 'row' (y) and 'col' (x) on the SSD1331.
/// @var row start row
/// @var col start col
/// @var row_end ending row
/// @var c ending column
///
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static HAL_StatusTypeDef goto_pixel(uint8_t row, uint8_t col, uint8_t row_end, uint8_t col_end)
{
  static uint8_t txb[]= { SSD1331_CMD_SETROW, 0, SSD1331_HEIGHT-1,
                          SSD1331_CMD_SETCOLUMN, 0, SSD1331_WIDTH-1 };

  txb[1]=min(row, SSD1331_HEIGHT-1);
  txb[2]=min(row_end, SSD1331_HEIGHT-1);
  txb[4]=min(col, SSD1331_WIDTH-1);
  txb[5]=min(col_end, SSD1331_WIDTH-1);
  return spi_transfer((const uint8_t *) txb, sizeof(txb), true);
}


// *******************************************************************************************************************
// Public functions
// *******************************************************************************************************************


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @brief Event handler for the SPI in use. Only clears the 'spi_transfer_in_progress' flag, which is set by the
///        'spi_transfer()' and 'ssd1331_write_command()' functions every time data is sent over the SPI.
///
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void spi_event_handler(void)
{
  spi_transfer_in_progress= false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef ssd1331_init(void)
{
  HAL_StatusTypeDef retCode;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  // hard reset SSD1331
  HAL_GPIO_WritePin(SSD1331_RST_GPIO_Port, SSD1331_RST_Pin, GPIO_PIN_RESET);
  osDelay(1);
  HAL_GPIO_WritePin(SSD1331_RST_GPIO_Port, SSD1331_RST_Pin, GPIO_PIN_SET);
  osDelay(1);

  // initialize SSD1331
  ssd1331_write_command(SSD1331_CMD_SETMASTER);     // write magic value to turn on chip
  ssd1331_write_command(0x8E);

  ssd1331_dim_display(true);                        // dim (blank) display during configuration

  ssd1331_write_command(SSD1331_CMD_SETREMAP);      // set bit depth (8 or 16 bit total)
  ssd1331_write_command((config.bit_depth == 8)
                        ?(0x32)
                        :(0x72));

  ssd1331_write_command(SSD1331_CMD_POWERMODE);     // disable power save mode
  ssd1331_write_command(0x0B);

  ssd1331_write_command(SSD1331_CMD_PHASEADJUST);   // adjust phase 1 and 2 of segment waveform of the driver (with the below values the display makes less audible HF noise)
  ssd1331_write_command(0b0000<<4 | 0b0001);

  ssd1331_write_command(SSD1331_CMD_SETOSCFREQ);
  ssd1331_write_command((config.turbo_mode)
                        ?(0b1111<<4 | 0b0000)           // in turbo mode go for 900 kHz
                        :(0b1101<<4 | 0b0000));         // in normal mode use default 800 kHz

  ssd1331_write_command(SSD1331_CMD_NORMALDISPLAY); // configure display for normal mode (no inversion etc.)

  //ssd1331_set_precharge(0x3A, 0x64, 0x78, 0x64); // tune precharge voltage and speeds per color channel

  ssd1331_set_brightness(8);                        // set medium brightness (can be adjusted later)

  //ssd1331_set_contrast(0x91, 0x50, 0x7D);       // adjust contrast per color channel

  ssd1331_dim_display(false);                       // now turn on display panel

  spi_is_initialized = true;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  LOG_INFO("SSD1331 powered up.");
  return HAL_OK;
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef ssd1331_set_brightness(uint8_t brightness)
{
  static uint8_t txb[]= {SSD1331_CMD_MASTERCURRENT,0};

  if (brightness > ssd1331_max_brightness)
  {
    return HAL_ERROR;
  }

  txb[1]= brightness;
  return spi_transfer((const uint8_t *) txb, sizeof(txb), true);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef ssd1331_set_precharge(uint8_t voltage, uint8_t speedA, uint8_t speedB, uint8_t speedC)
{
  static uint8_t txb[]= { SSD1331_CMD_PRECHARGEA, 0,
                          SSD1331_CMD_PRECHARGEB, 0,
                          SSD1331_CMD_PRECHARGEC, 0,
                          SSD1331_CMD_PRECHARGELEVEL, 0 };

  txb[1]= speedA;
  txb[3]= speedB;
  txb[5]= speedC;
  txb[7]= voltage;

  return spi_transfer((const uint8_t *) txb, sizeof(txb), true);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef ssd1331_set_contrast(uint8_t contrastA, uint8_t contrastB, uint8_t contrastC)
{
  static uint8_t txb[]= { SSD1331_CMD_CONTRASTA, 0,
                          SSD1331_CMD_CONTRASTB, 0,
                          SSD1331_CMD_CONTRASTC, 0 };

  txb[1]= contrastA;
  txb[3]= contrastB;
  txb[5]= contrastC;

  return spi_transfer((const uint8_t *) txb, sizeof(txb), true);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef ssd1331_dim_display(bool dim)
{
  static uint8_t dim1= SSD1331_CMD_DISPLAYDIMMED;
  static uint8_t dim0= SSD1331_CMD_DISPLAYON;

  return spi_transfer((dim)?((const uint8_t *) &dim1):((const uint8_t *) &dim0), 1, true);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef ssd1331_clear_window(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2)
{
  return ssd1331_draw_rectangle(x1, y1, x2, y2, SSD1331_RGB_COLOR_BLACK, true, SSD1331_RGB_COLOR_BLACK);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef ssd1331_clear()
{
  return ssd1331_clear_window(0,0,SSD1331_WIDTH-1,SSD1331_HEIGHT-1);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef ssd1331_draw_rectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint32_t framecolor, bool fill, uint32_t fillcolor)
{
  HAL_StatusTypeDef retCode;

  uint8_t txb[]= { SSD1331_CMD_FILL, (uint8_t) fill,
                   SSD1331_CMD_DRAWRECT, x1, y1, x2, y2,
                  (config.bit_depth == 8)?SSD1331_RGB_TO_RED_8BIT_MODE(framecolor):SSD1331_RGB_TO_RED_16BIT_MODE(framecolor),
                  (config.bit_depth == 8)?SSD1331_RGB_TO_GRN_8BIT_MODE(framecolor):SSD1331_RGB_TO_GRN_16BIT_MODE(framecolor),
                  (config.bit_depth == 8)?SSD1331_RGB_TO_BLU_8BIT_MODE(framecolor):SSD1331_RGB_TO_BLU_16BIT_MODE(framecolor),
                  (config.bit_depth == 8)?SSD1331_RGB_TO_RED_8BIT_MODE(fillcolor):SSD1331_RGB_TO_RED_16BIT_MODE(fillcolor),
                  (config.bit_depth == 8)?SSD1331_RGB_TO_GRN_8BIT_MODE(fillcolor):SSD1331_RGB_TO_GRN_16BIT_MODE(fillcolor),
                  (config.bit_depth == 8)?SSD1331_RGB_TO_BLU_8BIT_MODE(fillcolor):SSD1331_RGB_TO_BLU_16BIT_MODE(fillcolor) };

  last_tx_buf= (last_tx_buf+1) % ssd1331_buffers;
  memcpy(tx_buf[last_tx_buf], txb, sizeof(txb));

  retCode= spi_transfer(tx_buf[last_tx_buf], sizeof(txb), true);

  // give the SSD1331 some time (proportional to the are to be cleared) to finish internal drawing operations
  if (config.wait_for_iops)
  {
	  osDelay((config.turbo_mode)?1:3);
  }

  return retCode;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef ssd1331_draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint32_t color)
{
  uint8_t txb[]= { SSD1331_CMD_DRAWLINE, x1, y1, x2, y2,
                  (config.bit_depth == 8)?SSD1331_RGB_TO_RED_8BIT_MODE(color):SSD1331_RGB_TO_RED_16BIT_MODE(color),
                  (config.bit_depth == 8)?SSD1331_RGB_TO_GRN_8BIT_MODE(color):SSD1331_RGB_TO_GRN_16BIT_MODE(color),
                  (config.bit_depth == 8)?SSD1331_RGB_TO_BLU_8BIT_MODE(color):SSD1331_RGB_TO_BLU_16BIT_MODE(color) };

  last_tx_buf= (last_tx_buf+1) % ssd1331_buffers;
  memcpy(tx_buf[last_tx_buf], txb, sizeof(txb));

  return spi_transfer(tx_buf[last_tx_buf], sizeof(txb), true);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef ssd1331_draw_image(ssd1331_image_t const image, uint8_t x, uint8_t y, uint8_t num_skip_frame)
{
  HAL_StatusTypeDef retCode;
  uint8_t bytePerPixel= image.bit_depth / 8;
  uint32_t frame_size = image.height * image.width * bytePerPixel;

  retCode = goto_pixel(y, x, y + image.height, x + image.width);

  if( retCode == HAL_OK )
  {
	  retCode = spi_transfer( &image.data[ num_skip_frame * frame_size ], frame_size, false );
  }

  return retCode;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef ssd1331_draw_string(uint8_t const* str, uint8_t x, uint8_t y, uint32_t facecolor, uint32_t bg_color, ssd1331_charset_t const charset)
{
  uint8_t const char_spacing= 1;  ///< space to leave free between characters (in pixels)

  // if NULL pointer or empty string was passed, return now
  if ((!str) || (str[0] == 0))
  {
    return HAL_ERROR;
  }

  // limit string length based on display and charset dimensions as well as drawing position
  uint8_t max_chars_in_a_row= ((SSD1331_WIDTH-x) + char_spacing) / (charset.width + char_spacing);
  uint8_t const str_length= (strlen(str) < max_chars_in_a_row)?(strlen(str)):max_chars_in_a_row;

  // precalculate and cache a few important parametes
  uint16_t const charset_row_length_in_bits= ((uint16_t) charset.num_glyphs) * ((uint16_t) charset.width);
  uint8_t const charset_row_length_in_bytes= (charset_row_length_in_bits/8u) + ((charset_row_length_in_bits % 8u)?(1u):(0u));

  // preallocate
  HAL_StatusTypeDef retCode;
  uint8_t ch, current_bit_within_byte, idx;
  uint16_t start_bit, current_bit_num, current_byte_num;
  bool current_pixel_on;

  for (uint8_t r=0; r<charset.height; ++r)
  {
    idx= 0;
    last_tx_buf= (last_tx_buf+1) % ssd1331_buffers;

    for (uint8_t s=0; s<str_length; ++s)
    {
      ch= str[s];
      start_bit= charset.width * charset.ascii_map[ch];

      for (uint8_t c=0; c<charset.width; ++c)
      {
        current_bit_num= start_bit+c;
        current_byte_num= current_bit_num/8;
        current_bit_within_byte= current_bit_num%8;
        current_pixel_on= charset.data[current_byte_num + charset_row_length_in_bytes*r] & (0x80 >> current_bit_within_byte);

        if (config.bit_depth == 8)
        {
          tx_buf[last_tx_buf][idx++]= current_pixel_on ? SSD1331_RGB_TO_8BIT_COLOR(facecolor) : SSD1331_RGB_TO_8BIT_COLOR(bg_color);
        }
        else
        {
          tx_buf[last_tx_buf][idx++]= current_pixel_on ? SSD1331_RGB_TO_16BIT_COLOR_BYTE1(facecolor) : SSD1331_RGB_TO_16BIT_COLOR_BYTE1(bg_color);
          tx_buf[last_tx_buf][idx++]= current_pixel_on ? SSD1331_RGB_TO_16BIT_COLOR_BYTE2(facecolor) : SSD1331_RGB_TO_16BIT_COLOR_BYTE2(bg_color);
        }
      }

      if (s < (str_length-1))  ///< if this is not the last character in the string, add space after it
      {
        for (uint8_t cs=0; cs<char_spacing; ++cs)
        {
          if (config.bit_depth == 8)
          {
            tx_buf[last_tx_buf][idx++]= SSD1331_RGB_TO_8BIT_COLOR(bg_color);
          }
          else
          {
            tx_buf[last_tx_buf][idx++]= SSD1331_RGB_TO_16BIT_COLOR_BYTE1(bg_color);
            tx_buf[last_tx_buf][idx++]= SSD1331_RGB_TO_16BIT_COLOR_BYTE2(bg_color);
          }
        }
      }
    }

    // now a whole row of the text is in the buffer, let's position cursor and put data onto the screen
    retCode= goto_pixel(y+r,x, SSD1331_HEIGHT-1, SSD1331_WIDTH-1);
    if (retCode != HAL_OK) { return retCode; }

    // write data to GDDRAM
    retCode= spi_transfer((const uint8_t *) tx_buf[last_tx_buf], idx, false);
    if (retCode != HAL_OK) { return retCode; }
  }
  return retCode;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint32_t ssd1331_hsv2rgb(uint8_t h, uint8_t s, uint8_t v)
{
  uint8_t r, g, b, p, q, t, region, remainder;

  if (s == 0)
  {
    r=g=b=v;
    goto assemble_rgb;
  }

  region= h / 43;
  remainder= (h - (region * 43)) * 6;

  p= (v * (255 - s)) >> 8;
  q= (v * (255 - ((s * remainder) >> 8))) >> 8;
  t= (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

  switch (region)
  {
    case 0: r=v; g=t; b=p; break;
    case 1: r=q; g=v; b=p; break;
    case 2: r=p; g=v; b=t; break;
    case 3: r=p; g=q; b=v; break;
    case 4: r=t; g=p; b=v; break;
    default: r=v; g=p; b=q; break;
  }

assemble_rgb:

  return ((((uint32_t) r) << 16) | (((uint32_t) g) << 8) | ((uint32_t) b));
}


/// interface to uGUI

void pset(int16_t x, int16_t y, uint32_t col)
{
   uint32_t addr;

   if ( x<0 ) return;
   if ( y<0 ) return;

   addr = x + y * SSD1331_WIDTH;
//   addr<<=1;

   addr = min( addr, IMG_BUF_SIZE);

#if (SSD1331_BPP == 1)
   img_buf[addr] = SSD1331_RGB_TO_8BIT_COLOR(col);
#else
   img_buf[addr] = SSD1331_RGB_TO_16BIT_COLOR(col);

#endif
}

void ssd1331_update(void)
{
	if(spi_is_initialized)
		ssd1331_draw_image(ssd1331_frame, 0, 0, 0);
}


// *******************************************************************************************************************
// End of File
// *******************************************************************************************************************

#pragma once

#define SSD1327_I2C_ADDRESS_LOW     0x3C
#define SSD1327_I2C_ADDRESS_HIGH 	0x3D

#define SSD1327_SET_COLUMN_ADDRESS 0x15
#define SSD1327_SET_ROW_ADDRESS 0x75
#define SSD1327_SET_CONTRAST 0x81
#define SSD1327_SET_REMAP 0xA0
#define SSD1327_SET_DISPLAY_STARTLINE 0xA1
#define SSD1327_SET_DISPLAY_OFFSET 0xA2
#define SSD1327_NORMALDISPLAY 0xA4
#define SSD1327_DISPLAYALLON 0xA5
#define SSD1327_DISPLAYALLOFF 0xA6
#define SSD1327_INVERVSEDISPLAY 0xA7
#define SSD1327_SET_MULTIPLEX_RATIO 0xA8
#define SSD1327_FUNCTION_SELA 0xAB
#define SSD1327_DISPLAYOFF 0xAE
#define SSD1327_DISPLAYON 0xAF
#define SSD1327_SET_PHASE_LENGTH 0xB1
#define SSD1327_SET_CLK_OSC 0xB3
#define SSD1327_SET_GPIO 0xB5
#define SSD1327_SET_PRECHARGE2 0xB6
#define SSD1327_SET_GRAYSCALE_TABLE 0xB8
#define SSD1327_SELECT_DEF_LINEAR_GRAYSCALE 0xB9
#define SSD1327_SET_PRECHARGE 0xBC
#define SSD1327_SET_VCOM_VOLTAGE 0xBE
#define SSD1327_SET_FUNCTION_SELB 0xD5
#define SSD1327_SET_CMD_LOCK 0xFD

#define SSD1327_DEACTIVATE_SCROLL 0x2E
#define SSD1327_ACTIVATE_SCROLL 0x2F

/* time constants in ms */
#define SSD1327_RESET_DELAY			1
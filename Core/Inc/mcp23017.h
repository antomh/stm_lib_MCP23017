/*
 * mcp23017.h
 *
 *  Created on: Feb 24, 2021
 *  Author: Anton Shein<anton-shein2008@yandex.ru>
 *  ----------------------------------------------------------------------
 *  Copyright (C) Anton Shein, 2021
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  any later Bversion.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *  ----------------------------------------------------------------------
 */

#ifndef INC_MCP23017_H_
#define INC_MCP23017_H_

#include "main.h"
#include "stdio.h"
#include "string.h"
#include "stdarg.h"

#define DEBUGMODE

/* I2C used in this lib */
#define I2Cx            hi2c1
#define MCP_ADDR        ( 0x20 << 1 )

#define MCP_PIN_0       (uint8_t)(1 << 0)
#define MCP_PIN_1       (uint8_t)(1 << 1)
#define MCP_PIN_2       (uint8_t)(1 << 2)
#define MCP_PIN_3       (uint8_t)(1 << 3)
#define MCP_PIN_4       (uint8_t)(1 << 4)
#define MCP_PIN_5       (uint8_t)(1 << 5)
#define MCP_PIN_6       (uint8_t)(1 << 6)
#define MCP_PIN_7       (uint8_t)(1 << 7)

#define MCP_PORT_A      0xF0
#define MCP_PORT_B      0xF1

/* Register adress define for IOCON.BANK = 0 */
#define MCP_IODIRA      0x00        /* Controls the direction of the data IO */
#define MCP_IODIRB      0x01
#define MCP_IPOLA       0x02        /* Configure the polarity on the corresponding GPIO */
#define MCP_IPOLB       0x03
#define MCP_GPINTENA    0x04        /* Controls the interrupt-on-change feature for each pin */
#define MCP_GPINTENB    0x05
#define MCP_DEFVALA     0x06        /* Set default comparison value to compare in case of interrupt */
#define MCP_DEFVALB     0x07
#define MCP_INTCONA     0x08        /* Controls how the associated pin value is compared for the interrupt-on-change feature */
#define MCP_INTCONB     0x09
#define MCP_IOCON       0x0A        /* Config device */
#define MCP_GPPUA       0x0C        /* Controls the pull-up resistors for the port pins */
#define MCP_GPPUB       0x0D
#define MCP_INTFA       0x0E        /* Reflects the interrupt condition on the port pins of any pin that is enabled for interrupts via the GPINTEN register */
#define MCP_INTFB       0x0F
#define MCP_INTCAPA     0x10        /* Captures the GPIO port value at the time the interrupt occurred */
#define MCP_INTCAPB     0x11
#define MCP_GPIOA       0x12        /* Reflects the value on the port, and modify the output latches */
#define MCP_GPIOB       0x13

#define MCP_PIN_Mode_Out                0x14        /* Config GPIOx_Type */
#define MCP_PIN_Mode_In                 0x15
#define MCP_PIN_MemBank_Divide_Off      0x16        /* GPIO register not paired */
#define MCP_PIN_MemBank_Divide_On       0x17

#define MCP_PIN_In_Polar_Same           0x18        /* GPIO register bit reflects the same logic state of the input pin */
#define MCP_PIN_In_Polar_Reverse        0x19
#define MCP_PIN_In_PullUp_Off           0x1A        /* Controls the pull-up resistors (100 kOhm) for the port pins */
#define MCP_PIN_In_PullUp_On            0x1B

#define MCP_PIN_In_Int_State_Off        0x1C        /* Turn on/off interrupt */
#define MCP_PIN_In_Int_State_On         0x1D
#define MCP_PIN_In_Int_Mirror_Off       0x1E        /* Turn on/off mirroring INTA and INTB refisters */
#define MCP_PIN_In_Int_Mirror_On        0x1F
#define MCP_PIN_In_Int_Comp_Prev        0x20        /* Set interrupt compare value */
#define MCP_PIN_In_Int_Comp_DefVal      0x21
#define MCP_PIN_In_Int_DefVal_Low       0x22
#define MCP_PIN_In_Int_DefVal_High      0x23
#define MCP_PIN_In_Int_Mode_OpenDrain   0x24        /* Config the INT pin as an open-drain output */
#define MCP_PIN_In_Int_Mode_ActDriver   0x25
#define MCP_PIN_In_Int_PinLvl_Low       0x26        /* Config the polarity of the INT output pin */
#define MCP_PIN_In_Int_PinLvl_High      0x27

#define MCP_PIN_Out_PinVal_0            0x28        /* Set corresponding GPIO pin value */
#define MCP_PIN_Out_PinVal_1            0x29

enum STATE
{
    MCP_OK    = 0,
    MCP_ERR   = 1
};


typedef struct
{
    uint8_t state;
    uint8_t mirror;
    uint8_t compare;
    uint8_t defval;        // not necessary to fill if compare with previous value
    uint8_t mode;
    uint8_t pinlvl;
} MCP_InterruptTypeDef;


typedef struct
{
    uint8_t pinval;
} MCP_PinOutConfig;

typedef struct
{
    uint8_t polar;
    uint8_t pullup;
    MCP_InterruptTypeDef interrupt;
} MCP_PinInConfig;


typedef struct
{
    uint8_t mode;
    uint8_t membank;
    MCP_PinInConfig in;
    MCP_PinOutConfig out;
} MCP_PinConfig;


/**
 * @brief   Print some text to UART
 */
void uart_print(char *print_data, ...);

/*
 * @brief   Set data to redister
 */
uint8_t mcp_write_reg(uint8_t reg_addr, uint8_t reg_data);

/*
 * @brief   Read data from redister
 * @retval  *reg_data : value of this register
 */
uint8_t mcp_read_reg(uint8_t reg_addr);

/*
 * @brief   Config MCP pin
 */
uint8_t mcp_pin_config(MCP_PinConfig *mcp, uint8_t port, uint8_t pin);

/*
 * @brief   Init MCP
 */
uint8_t mcp_init(void);


#endif /* INC_MCP23017_H_ */

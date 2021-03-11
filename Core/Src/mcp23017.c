/*
 * mcp23017.c
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

#include "mcp23017.h"


extern I2C_HandleTypeDef I2Cx;
extern UART_HandleTypeDef huart1;


void uart_print(char *print_data, ...)
{
    char sbuf[1024];
    va_list argptr;
    va_start(argptr, print_data);
    vsprintf(sbuf, print_data, argptr);
    va_end(argptr);

    HAL_UART_Transmit(&huart1, (uint8_t*)sbuf, strlen(sbuf), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart1, (uint8_t*)"\n\r", 2, HAL_MAX_DELAY);
}


uint8_t mcp_write_reg(uint8_t reg_addr, uint8_t reg_data)
{
    uint8_t data[2] = { reg_addr,
                        reg_data };

    /* Send register address, then data */
    if ( HAL_I2C_Master_Transmit( &I2Cx, MCP_ADDR, data, 2, HAL_MAX_DELAY) != HAL_OK )
            return MCP_ERR;

    return MCP_OK;
}


uint8_t mcp_read_reg(uint8_t reg_addr)
{
    uint8_t reg_data = 0;

    /* Send register address */
    if ( HAL_I2C_Master_Transmit( &I2Cx, MCP_ADDR, &reg_addr, 1, HAL_MAX_DELAY) != HAL_OK )
        return MCP_ERR;
    /* Send data to register */
    if ( HAL_I2C_Master_Receive( &I2Cx, MCP_ADDR, &reg_data, 1, HAL_MAX_DELAY) != HAL_OK )
        return MCP_ERR;

    return reg_data;
}


static uint8_t mcp_pin_config_mode(uint8_t mode, uint8_t port, uint8_t pin)
{
    uint8_t regval = 0x00;
    switch (mode)
    {
    case MCP_PIN_Mode_Out:
        switch (port)
        {
        case MCP_PORT_A:
            regval = mcp_read_reg(MCP_IODIRA);                                  // read current register value
            if ( mcp_write_reg(MCP_IODIRA, ( regval & (~pin) ) ) == MCP_ERR )   // clear corresponding bit
                return MCP_ERR;
            break;
        case MCP_PORT_B:
            regval = mcp_read_reg(MCP_IODIRB);                                  // read current register value
#ifdef DEBUGMODE
    uart_print("mcp_pin_config_mode; regval = %x; pin = %x; ~pin = %x; regval & (~pin) = %x", regval, pin, (~pin), ( regval & (~pin) ) );
#endif
            if ( mcp_write_reg(MCP_IODIRB, ( regval & (~pin) ) ) == MCP_ERR )   // clear corresponding bit
                return MCP_ERR;
            break;
        default:
            return MCP_ERR;
            break;
        }
        break;

    case MCP_PIN_Mode_In:
        switch (port)
        {
        case MCP_PORT_A:
            regval = mcp_read_reg(MCP_IODIRA);                                  // read current register value
            if ( mcp_write_reg(MCP_IODIRA, ( regval & pin ) ) == MCP_ERR )      // add corresponding bit
                return MCP_ERR;
            break;
        case MCP_PORT_B:
            regval = mcp_read_reg(MCP_IODIRB);                                  // read current register value
            if ( mcp_write_reg(MCP_IODIRB, ( regval & pin ) ) == MCP_ERR )      // add corresponding bit
                return MCP_ERR;
            break;
        default:
            return MCP_ERR;
            break;
        }
        break;

    default:
        return MCP_ERR;
        break;
    }
    return MCP_OK;
}


static uint8_t mcp_pin_config_membank(uint8_t mode)
{
    uint8_t regval = 0x00;
    switch (mode)
    {
    case MCP_PIN_MemBank_Divide_Off:
        regval = mcp_read_reg(MCP_IOCON);
        if ( mcp_write_reg(MCP_IOCON, ( regval & (~MCP_PIN_7) ) ) == MCP_ERR )
            return MCP_ERR;
        break;

    case MCP_PIN_MemBank_Divide_On:
        regval = mcp_read_reg(MCP_IOCON);
        if ( mcp_write_reg(MCP_IOCON, ( regval | MCP_PIN_7 ) ) == MCP_ERR )
            return MCP_ERR;
        break;

    default:
        return MCP_ERR;
        break;
    }
    return MCP_OK;
}


static uint8_t mcp_pin_config_out_pinval(uint8_t port, uint8_t pin, uint8_t pinval)
{
    uint8_t regval = 0x00;
    switch (pinval)
    {
    case MCP_PIN_Out_PinVal_0:
        switch (port)
        {
        case MCP_PORT_A:
            regval = mcp_read_reg(MCP_GPIOA);                                   // read current register value
            if ( mcp_write_reg(MCP_GPIOA, ( regval & (~pin) ) ) == MCP_ERR )    // clear corresponding bit
                return MCP_ERR;
            break;
        case MCP_PORT_B:
            regval = mcp_read_reg(MCP_GPIOB);                                   // read current register value
            if ( mcp_write_reg(MCP_GPIOB, ( regval & (~pin) ) ) == MCP_ERR )    // clear corresponding bit
                return MCP_ERR;
            break;
        default:
            return MCP_ERR;
            break;
        }
        break;
    case MCP_PIN_Out_PinVal_1:
        switch (port)
        {
        case MCP_PORT_A:
            regval = mcp_read_reg(MCP_GPIOA);                                   // read current register value
            if ( mcp_write_reg(MCP_GPIOA, ( regval | pin ) ) == MCP_ERR )       // clear corresponding bit
                return MCP_ERR;
            break;
        case MCP_PORT_B:
            regval = mcp_read_reg(MCP_GPIOB);                                   // read current register value
#ifdef DEBUGMODE
    uart_print("mcp_pin_config_pinval; MCP_GPIOB = %x; pin = %x; val_to_write = %x", regval, pin, ( regval | pin ) );
#endif
            if ( mcp_write_reg(MCP_GPIOB, ( regval | pin ) ) == MCP_ERR )       // clear corresponding bit
                return MCP_ERR;
#ifdef DEBUGMODE
    regval = mcp_read_reg(MCP_GPIOB);
    uart_print("mcp_pin_config_pinval; MCP_GPIOB = %x", regval );
#endif
            break;
        default:
            return MCP_ERR;
            break;
        }
        break;
    default:
        return MCP_ERR;
        break;
    }
    return MCP_OK;
}


static uint8_t mcp_pin_config_in_polar(uint8_t port, uint8_t pin, uint8_t polar)
{
    uint8_t regval = 0x00;
    switch (polar)
    {
    case MCP_PIN_In_Polar_Same:
    switch (port)
    {
    case MCP_PORT_A:
        regval = mcp_read_reg(MCP_IPOLA);                                  // read current register value
        if ( mcp_write_reg(MCP_IPOLA, ( regval & (~pin) ) ) == MCP_ERR )   // clear corresponding bit
            return MCP_ERR;
        break;
    case MCP_PORT_B:
        regval = mcp_read_reg(MCP_IPOLB);                                  // read current register value
        if ( mcp_write_reg(MCP_IPOLB, ( regval & (~pin) ) ) == MCP_ERR )   // clear corresponding bit
            return MCP_ERR;
        break;
    default:
        return MCP_ERR;
        break;
    }
    break;
    case MCP_PIN_In_Polar_Reverse:
    switch (port)
    {
    case MCP_PORT_A:
        regval = mcp_read_reg(MCP_IPOLA);                                  // read current register value
        if ( mcp_write_reg(MCP_IPOLA, ( regval | pin ) ) == MCP_ERR )      // clear corresponding bit
            return MCP_ERR;
        break;
     case MCP_PORT_B:
        regval = mcp_read_reg(MCP_IPOLB);                                  // read current register value
        if ( mcp_write_reg(MCP_IPOLB, ( regval | pin ) ) == MCP_ERR )      // clear corresponding bit
            return MCP_ERR;
        break;
     default:
        return MCP_ERR;
        break;
     }
     break;
    default:
        return MCP_ERR;
        break;
    }
    return MCP_OK;
}


static uint8_t mcp_pin_config_in_pullup(uint8_t port, uint8_t pin, uint8_t pullup)
{
    uint8_t regval = 0x00;
    switch (pullup)
    {
    case MCP_PIN_In_PullUp_On:
        switch (port)
        {
        case MCP_PORT_A:
            regval = mcp_read_reg(MCP_GPPUA);                                  // read current register value
            if ( mcp_write_reg(MCP_GPPUA, ( regval | pin ) ) == MCP_ERR )   // clear corresponding bit
                return MCP_ERR;
            break;
        case MCP_PORT_B:
            regval = mcp_read_reg(MCP_GPPUB);                                  // read current register value
            if ( mcp_write_reg(MCP_GPPUB, ( regval | pin ) ) == MCP_ERR )   // clear corresponding bit
                return MCP_ERR;
            break;
        default:
            return MCP_ERR;
            break;
        }
        break;
    case MCP_PIN_In_PullUp_Off:
        switch (port)
        {
        case MCP_PORT_A:
            regval = mcp_read_reg(MCP_GPPUA);                                  // read current register value
            if ( mcp_write_reg(MCP_GPPUA, ( regval & (~pin) ) ) == MCP_ERR )   // clear corresponding bit
                return MCP_ERR;
            break;
        case MCP_PORT_B:
            regval = mcp_read_reg(MCP_GPPUB);                                  // read current register value
            if ( mcp_write_reg(MCP_GPPUB, ( regval & (~pin) ) ) == MCP_ERR )   // clear corresponding bit
                return MCP_ERR;
            break;
        default:
            return MCP_ERR;
            break;
        }
        break;
    default:
        return MCP_ERR;
        break;
    }
    return MCP_OK;
}


static uint8_t mcp_pin_config_in_int(MCP_PinConfig *mcp, uint8_t port, uint8_t pin)
{
    uint8_t regval = 0x00;
    /* Turn on / off interrupt */
    switch (mcp->in.interrupt.state)
    {
    case MCP_PIN_In_Int_State_Off:
        switch (port)
        {
        case MCP_PORT_A:
            regval = mcp_read_reg(MCP_GPINTENA);                                // read current register value
            if ( mcp_write_reg(MCP_GPINTENA, ( regval & (~pin) ) ) == MCP_ERR ) // clear corresponding bit
                return MCP_ERR;
            break;
        case MCP_PORT_B:
            regval = mcp_read_reg(MCP_GPINTENB);                                // read current register value
            if ( mcp_write_reg(MCP_GPINTENB, ( regval & (~pin) ) ) == MCP_ERR ) // clear corresponding bit
                return MCP_ERR;
            break;
        default:
            return MCP_ERR;
            break;
        }
        return MCP_OK;                                                          // Turn off interrupt and return from function
        break;
    case MCP_PIN_In_Int_State_On:
        switch (port)
        {
        case MCP_PORT_A:
            regval = mcp_read_reg(MCP_GPINTENA);                                // read current register value
            if ( mcp_write_reg(MCP_GPINTENA, ( regval | pin ) ) == MCP_ERR )    // set corresponding bit
                return MCP_ERR;
            break;
        case MCP_PORT_B:
            regval = mcp_read_reg(MCP_GPINTENB);                                // read current register value
            if ( mcp_write_reg(MCP_GPINTENB, ( regval | pin ) ) == MCP_ERR )    // set corresponding bit
                return MCP_ERR;
            break;
        default:
            return MCP_ERR;
            break;
        }
        break;
    default:
        return MCP_ERR;
        break;
    }

    /* Continue to setup  */
    /* Config mirroring INT pins */
    switch (mcp->in.interrupt.mirror)
    {
    case MCP_PIN_In_Int_Mirror_Off:
        switch (port)
        {
        case MCP_PORT_A:
        case MCP_PORT_B:
            regval = mcp_read_reg(MCP_IOCON);                                   // read current register value
            if ( mcp_write_reg(MCP_IOCON, ( regval & (~MCP_PIN_6) ) ) == MCP_ERR )    // clear corresponding bit
                return MCP_ERR;
            break;
        default:
            return MCP_ERR;
            break;
        }
        break;
    case MCP_PIN_In_Int_Mirror_On:
        switch (port)
        {
        case MCP_PORT_A:
        case MCP_PORT_B:
            regval = mcp_read_reg(MCP_IOCON);                                   // read current register value
            if ( mcp_write_reg(MCP_IOCON, ( regval | MCP_PIN_6 ) ) == MCP_ERR )       // set corresponding bit
                return MCP_ERR;
            break;
        default:
            return MCP_ERR;
            break;
        }
        break;
    default:
        return MCP_ERR;
        break;
    }

    /* Set interrupt compare value */
    switch (mcp->in.interrupt.compare)
    {
    case MCP_PIN_In_Int_Comp_Prev:
        switch (port)
        {
        case MCP_PORT_A:
            regval = mcp_read_reg(MCP_INTCONA);                                 // read current register value
            if ( mcp_write_reg(MCP_INTCONA, ( regval & (~pin) ) ) == MCP_ERR )  // clear corresponding bit
                return MCP_ERR;
            break;
        case MCP_PORT_B:
            regval = mcp_read_reg(MCP_INTCONB);                                 // read current register value
            if ( mcp_write_reg(MCP_INTCONB, ( regval & (~pin) ) ) == MCP_ERR )  // clear corresponding bit
                return MCP_ERR;
            break;
        default:
            return MCP_ERR;
            break;
        }
        break;
    case MCP_PIN_In_Int_Comp_DefVal:

        switch (port)
        {
        case MCP_PORT_A:
            regval = mcp_read_reg(MCP_INTCONA);                                 // read current register value
            if ( mcp_write_reg(MCP_INTCONA, ( regval | pin ) ) == MCP_ERR )     // set corresponding bit
                return MCP_ERR;
            break;
        case MCP_PORT_B:
            regval = mcp_read_reg(MCP_INTCONB);                                 // read current register value
            if ( mcp_write_reg(MCP_INTCONB, ( regval | pin ) ) == MCP_ERR )     // set corresponding bit
                return MCP_ERR;
            break;
        default:
            return MCP_ERR;
            break;
        }

        /* Config DEFVAL value */
        switch (mcp->in.interrupt.defval)
        {
        case MCP_PIN_In_Int_DefVal_Low:
            switch (port)
            {
            case MCP_PORT_A:
                regval = mcp_read_reg(MCP_DEFVALA);                                 // read current register value
                if ( mcp_write_reg(MCP_DEFVALA, ( regval & (~pin) ) ) == MCP_ERR )  // clear corresponding bit
                    return MCP_ERR;
                break;
            case MCP_PORT_B:
                regval = mcp_read_reg(MCP_DEFVALB);                                 // read current register value
                if ( mcp_write_reg(MCP_DEFVALB, ( regval & (~pin) ) ) == MCP_ERR )  // clear corresponding bit
                    return MCP_ERR;
                break;
            default:
                return MCP_ERR;
                break;
            }
            break;
        case MCP_PIN_In_Int_DefVal_High:
            switch (port)
            {
            case MCP_PORT_A:
                regval = mcp_read_reg(MCP_DEFVALA);                                 // read current register value
                if ( mcp_write_reg(MCP_DEFVALA, ( regval | pin ) ) == MCP_ERR )     // set corresponding bit
                    return MCP_ERR;
                break;
            case MCP_PORT_B:
                regval = mcp_read_reg(MCP_DEFVALB);                                 // read current register value
                if ( mcp_write_reg(MCP_DEFVALB, ( regval | pin ) ) == MCP_ERR )     // set corresponding bit
                    return MCP_ERR;
                break;
            default:
                return MCP_ERR;
                break;
            }
            break;
        default:
            return MCP_ERR;
            break;
        }

        break;
    default:
        return MCP_ERR;
        break;
    }

    /* Config INT pin as open-drain output / as active driver */
    switch (mcp->in.interrupt.mode)
    {
    case MCP_PIN_In_Int_Mode_OpenDrain:
        switch (port)
        {
        case MCP_PORT_A:
        case MCP_PORT_B:
            regval = mcp_read_reg(MCP_IOCON);                                       // read current register value
            if ( mcp_write_reg(MCP_IOCON, ( regval | MCP_PIN_2 ) ) == MCP_ERR )     // set corresponding bit
                return MCP_ERR;
            break;
        default:
            return MCP_ERR;
            break;
        }
        break;
    case MCP_PIN_In_Int_Mode_ActDriver:
        switch (port)
        {
        case MCP_PORT_A:
        case MCP_PORT_B:
            regval = mcp_read_reg(MCP_IOCON);                                       // read current register value
            if ( mcp_write_reg(MCP_IOCON, ( regval & (~MCP_PIN_2) ) ) == MCP_ERR )  // clear corresponding bit
                return MCP_ERR;
            break;
        default:
            return MCP_ERR;
            break;
        }
        break;

        /* Set INT pin polarity */
        switch (mcp->in.interrupt.pinlvl)
        {
        case MCP_PIN_In_Int_PinLvl_Low:
            switch (port)
            {
            case MCP_PORT_A:
            case MCP_PORT_B:
                regval = mcp_read_reg(MCP_IOCON);                                       // read current register value
                if ( mcp_write_reg(MCP_IOCON, ( regval & (~MCP_PIN_1) ) ) == MCP_ERR )  // set corresponding bit
                    return MCP_ERR;
                break;
            default:
                return MCP_ERR;
                break;
            }
            break;
        case MCP_PIN_In_Int_PinLvl_High:
            switch (port)
            {
            case MCP_PORT_A:
            case MCP_PORT_B:
                regval = mcp_read_reg(MCP_IOCON);                                       // read current register value
                if ( mcp_write_reg(MCP_IOCON, ( regval | MCP_PIN_1 ) ) == MCP_ERR )     // set corresponding bit
                    return MCP_ERR;
                break;
            default:
                return MCP_ERR;
                break;
            }
            break;
        default:
            return MCP_ERR;
            break;
        }

        break;
    default:
        return MCP_ERR;
        break;
    }

    return MCP_OK;
}




static uint8_t mcp_pin_config_out(MCP_PinConfig *mcp, uint8_t port, uint8_t pin)
{
    /* Config pin value as output */
    switch (mcp->out.pinval)
    {
    case MCP_PIN_Out_PinVal_0:
        if ( mcp_pin_config_out_pinval(port, pin, MCP_PIN_Out_PinVal_0) == MCP_ERR )
            return MCP_ERR;
        break;
    case MCP_PIN_Out_PinVal_1:
        if ( mcp_pin_config_out_pinval(port, pin, MCP_PIN_Out_PinVal_1) == MCP_ERR )
            return MCP_ERR;
#ifdef DEBUGMODE
    uart_print("Set PinVal successfully");
#endif
        break;
    default:
        return MCP_ERR;
        break;
    }
    return MCP_OK;          // MCP pin fully config as output
}


static uint8_t mcp_pin_config_in(MCP_PinConfig *mcp, uint8_t port, uint8_t pin)
{
    /* Config pin polarisation as input */
    switch (mcp->in.polar)
    {
    case MCP_PIN_In_Polar_Same:
        if ( mcp_pin_config_in_polar(port, pin, MCP_PIN_In_Polar_Same) == MCP_ERR )
            return MCP_ERR;
        break;
    case MCP_PIN_In_Polar_Reverse:
        if ( mcp_pin_config_in_polar(port, pin, MCP_PIN_In_Polar_Reverse) == MCP_ERR )
            return MCP_ERR;
        break;
    default:
        return MCP_ERR;
        break;
    }

    /* Config pin pull-up */
    switch (mcp->in.pullup)
    {
    case MCP_PIN_In_PullUp_Off:
        if ( mcp_pin_config_in_pullup(port, pin, MCP_PIN_In_PullUp_Off) == MCP_ERR )
            return MCP_ERR;
        break;
    case MCP_PIN_In_PullUp_On:
        if ( mcp_pin_config_in_pullup(port, pin, MCP_PIN_In_PullUp_On) == MCP_ERR )
            return MCP_ERR;
        break;
    default:
        return MCP_ERR;
        break;
    }

    /* Config pin interrupt */
    switch (mcp->in.interrupt.state)
    {
    case MCP_PIN_In_Int_State_Off:
    case MCP_PIN_In_Int_State_On:
        if ( mcp_pin_config_in_int(mcp, port, pin) == MCP_ERR )
            return MCP_ERR;
        break;
    default:
        return MCP_ERR;
        break;
    }

    return MCP_OK;          // MCP pin fully config as input
}


uint8_t mcp_pin_config(MCP_PinConfig *mcp, uint8_t port, uint8_t pin)
{
#ifdef DEBUGMODE
    uart_print("Start pin config");
#endif
    /* Memory bank config */
    if ( mcp_pin_config_membank(mcp->membank) == MCP_ERR )
        return MCP_ERR;
#ifdef DEBUGMODE
    uart_print("Membank set successfully");
#endif
    /* Mode config */
    switch (mcp->mode)
    {
    case MCP_PIN_Mode_Out:
#ifdef DEBUGMODE
    uart_print("Select out pin mode");
#endif
        if ( mcp_pin_config_mode(MCP_PIN_Mode_Out, port, pin) == MCP_ERR)
            return MCP_ERR;
#ifdef DEBUGMODE
    uart_print("Set mode successfully");
#endif
        if ( mcp_pin_config_out(mcp, port, pin) == MCP_ERR)
            return MCP_ERR;
#ifdef DEBUGMODE
    uart_print("Config pin out successfully");
#endif
        break;
    case MCP_PIN_Mode_In:
        if ( mcp_pin_config_mode(MCP_PIN_Mode_In, port, pin) == MCP_ERR)
            return MCP_ERR;
        if ( mcp_pin_config_in(mcp, port, pin) == MCP_ERR)
            return MCP_ERR;
        break;
    default:
        return MCP_ERR;
        break;
    }
    return MCP_OK;
}


/*
 * @brief   MCP23017 initialize function. Call it before any other.
 */
uint8_t mcp_init(void)
{
    /* Reset MCP23017 */
    HAL_GPIO_WritePin(MCP_RST_GPIO_Port, MCP_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(500);
    HAL_GPIO_WritePin(MCP_RST_GPIO_Port, MCP_RST_Pin, GPIO_PIN_SET);

    return MCP_OK;
}






















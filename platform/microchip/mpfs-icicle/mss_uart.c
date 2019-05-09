/*******************************************************************************
 * (c) Copyright 2018 Microsemi SoC Products Group.  All rights reserved.
 *
 * PolarFire SoC(PSE) Microcontroller Subsystem MMUART bare metal software driver
 * implementation.
 *
 * This driver is based on SmartFusion2 MMUART driver v2.1.100
 *
 *
 */
//#ifdef PSE

#include "mss_uart.h"
#include "mss_uart_regs.h"
#include "mss_sysreg.h"


#ifdef __cplusplus
extern "C" {
#endif

#define MMUART0_E51_INT   0
#define MMUART0_PLIC_77   77


#define EXT_IRQ_KEEP_ENABLED 1
#define MSS_AXI_SWITCH_CLK 70000000 //50000000

/*These addresses are derived from http://homestead/asic/regmap/mss/html/g5soc_mss_regmap_AXID.html*/
#define MSS_UART0_LO_BASE           (UART_TypeDef*)0x20000000
#define MSS_UART1_LO_BASE           (UART_TypeDef*)0x20100000
#define MSS_UART2_LO_BASE           (UART_TypeDef*)0x20102000
#define MSS_UART3_LO_BASE           (UART_TypeDef*)0x20104000
#define MSS_UART4_LO_BASE           (UART_TypeDef*)0x20106000

#define MSS_UART0_HI_BASE           (UART_TypeDef*)0x28000000
#define MSS_UART1_HI_BASE           (UART_TypeDef*)0x28100000
#define MSS_UART2_HI_BASE           (UART_TypeDef*)0x28002000
#define MSS_UART3_HI_BASE           (UART_TypeDef*)0x28004000
#define MSS_UART4_HI_BASE           (UART_TypeDef*)0x28006000


mss_uart_instance_t g_mss_uart0_lo;
mss_uart_instance_t g_mss_uart1_lo;
mss_uart_instance_t g_mss_uart2_lo;
mss_uart_instance_t g_mss_uart3_lo;
mss_uart_instance_t g_mss_uart4_lo;

mss_uart_instance_t g_mss_uart0_hi;
mss_uart_instance_t g_mss_uart1_hi;
mss_uart_instance_t g_mss_uart2_hi;
mss_uart_instance_t g_mss_uart3_hi;
mss_uart_instance_t g_mss_uart4_hi;

#define ASSERT(CHECK)

/* This variable tracks if the UART peripheral is located on S5 or S6 on AXI switch
 * This will be used to determine which uart instance to be passed to UART
 * interrupt handler. value 0 = S5(low). value 1 = S6(high)
 * Bit positions:
 * 	0 ==> MMUART0
 * 	1 ==> MMUART1
 * 	2 ==> MMUART2
 * 	3 ==> MMUART3
 * 	4 ==> MMUART4

 */
uint8_t g_uart_axi_pos = 0x0;

/*******************************************************************************
 * Defines
 */
#define TX_COMPLETE            0u
#define TX_FIFO_SIZE           16u

#define FCR_TRIG_LEVEL_MASK    0xC0u

#define IIRF_MASK              0x0Fu

#define INVALID_INTERRUPT      0u
#define INVALID_IRQ_HANDLER    ((mss_uart_irq_handler_t) 0)
#define NULL_HANDLER           ((mss_uart_irq_handler_t) 0)

#define MSS_UART_DATA_READY    ((uint8_t) 0x01)

#define SYNC_ASYNC_MODE_MASK   (0x7u)

/*******************************************************************************
 * Possible values for Interrupt Identification Register Field.
 */
#define IIRF_MODEM_STATUS   0x00u
#define IIRF_THRE           0x02u
#define IIRF_MMI            0x03u
#define IIRF_RX_DATA        0x04u
#define IIRF_RX_LINE_STATUS 0x06u
#define IIRF_DATA_TIMEOUT   0x0Cu

/*******************************************************************************
 * Receiver error status mask.
 */
#define STATUS_ERROR_MASK    ( MSS_UART_OVERUN_ERROR | MSS_UART_PARITY_ERROR | \
                               MSS_UART_FRAMING_ERROR  | MSS_UART_BREAK_ERROR | \
                               MSS_UART_FIFO_ERROR)

/*******************************************************************************
 * Local functions.
 */
static void global_init(mss_uart_instance_t * this_uart, uint32_t baud_rate, 
                        uint8_t line_config, uint8_t intr_type);

static void config_baud_divisors
(
    mss_uart_instance_t * this_uart,
    uint32_t baudrate
);

/*******************************************************************************
 * Global initialization for all modes
 */
static void global_init
(
    mss_uart_instance_t * this_uart,
    uint32_t baud_rate,
    uint8_t line_config,
	uint8_t intr_type
)
{
	this_uart->intr_type = intr_type;

    if((&g_mss_uart0_lo == this_uart))
    {
        SYSREG->SOFT_RESET_CR |= (0x00000001UL << 5u);
        this_uart->hw_reg = MSS_UART0_LO_BASE;
        g_uart_axi_pos &= ~0x01;

        if(MSS_UART0_INTR_LOCAL == intr_type)
        	this_uart->irqn = MMUART0_E51_INT;
        else
        	this_uart->irqn = MMUART0_PLIC_77;

    	SYSREG->SOFT_RESET_CR &= ~(0x00000001UL << 5u);
    }
#if 0
    else if(&g_mss_uart1_lo == this_uart)
    {
    	SYSREG->SOFT_RESET_CR |= (0x00000001UL << 6u);
        this_uart->hw_reg = MSS_UART1_LO_BASE;
        this_uart->irqn = MMUART1_PLIC;
        g_uart_axi_pos &= ~0x02;
        SYSREG->SOFT_RESET_CR &= ~(0x00000001UL << 6u);
    }
    else if(&g_mss_uart2_lo == this_uart)
    {
    	SYSREG->SOFT_RESET_CR |= (0x00000001UL << 7u);
        this_uart->hw_reg = MSS_UART2_LO_BASE;
        this_uart->irqn = MMUART2_PLIC;
        g_uart_axi_pos &= ~0x04;
        SYSREG->SOFT_RESET_CR &= ~(0x00000001UL << 7u);
    }
    else if(&g_mss_uart3_lo == this_uart)
    {
    	SYSREG->SOFT_RESET_CR |= (0x00000001UL << 8u);
        this_uart->hw_reg = MSS_UART3_LO_BASE;
        this_uart->irqn = MMUART3_PLIC;
        g_uart_axi_pos &= ~0x08;
        SYSREG->SOFT_RESET_CR &= ~(0x00000001UL << 8u);
    }
    else if(&g_mss_uart4_lo == this_uart)
    {
    	SYSREG->SOFT_RESET_CR |= (0x00000001UL << 9u);
        this_uart->hw_reg = MSS_UART4_LO_BASE;
        this_uart->irqn = MMUART4_PLIC;
        g_uart_axi_pos &= ~0x10;
        SYSREG->SOFT_RESET_CR &= ~(0x00000001UL << 9u);
    }
    else if((&g_mss_uart0_hi == this_uart))
    {
    	SYSREG->SOFT_RESET_CR |= (0x00000001UL << 5u);
        this_uart->hw_reg = MSS_UART0_HI_BASE;
    	g_uart_axi_pos |= 0x01;

        if(MSS_UART0_INTR_LOCAL == intr_type)
        	this_uart->irqn = MMUART0_E51_INT;
        else
        	this_uart->irqn = MMUART0_PLIC_77;

        SYSREG->SOFT_RESET_CR &= ~(0x00000001UL << 5u);
    }
    else if(&g_mss_uart1_hi == this_uart)
    {
    	SYSREG->SOFT_RESET_CR |= (0x00000001UL << 6u);
        this_uart->hw_reg = MSS_UART1_HI_BASE;
        this_uart->irqn = MMUART1_PLIC;
        g_uart_axi_pos |= 0x02;
        SYSREG->SOFT_RESET_CR &= ~(0x00000001UL << 6u);
    }
    else if(&g_mss_uart2_hi == this_uart)
    {
    	SYSREG->SOFT_RESET_CR |= (0x00000001UL << 7u);
        this_uart->hw_reg = MSS_UART2_HI_BASE;
        this_uart->irqn = MMUART2_PLIC;
        g_uart_axi_pos |= 0x04;
        SYSREG->SOFT_RESET_CR &= ~(0x00000001UL << 7u);
    }
    else if(&g_mss_uart3_hi == this_uart)
    {
    	SYSREG->SOFT_RESET_CR |= (0x00000001UL << 8u);
        this_uart->hw_reg = MSS_UART3_HI_BASE;
        this_uart->irqn = MMUART3_PLIC;
        g_uart_axi_pos |= 0x08;
        SYSREG->SOFT_RESET_CR &= ~(0x00000001UL << 8u);
    }
    else if(&g_mss_uart4_hi == this_uart)
    {
    	SYSREG->SOFT_RESET_CR |= (0x00000001UL << 9u);
        this_uart->hw_reg = MSS_UART4_HI_BASE;
        this_uart->irqn = MMUART4_PLIC;
        g_uart_axi_pos |= 0x10;
        SYSREG->SOFT_RESET_CR &= ~(0x00000001UL << 9u);
    }
    else
    {
        ASSERT(0);
    }
#endif
    /* disable interrupts */
    this_uart->hw_reg->IER = 0u;

    /* FIFO configuration */
    this_uart->hw_reg->FCR = (uint8_t)MSS_UART_FIFO_SINGLE_BYTE;

    /* clear receiver FIFO */
    this_uart->hw_reg->FCR |= CLEAR_RX_FIFO_MASK;

    /* clear transmitter FIFO */
    this_uart->hw_reg->FCR |= CLEAR_TX_FIFO_MASK;

    /* set default READY mode : Mode 0*/
    /* enable RXRDYN and TXRDYN pins. The earlier FCR write to set the TX FIFO
     * trigger level inadvertently disabled the FCR_RXRDY_TXRDYN_EN bit. */
    this_uart->hw_reg->FCR |= RXRDY_TXRDYN_EN_MASK;

    /* disable loopback : local * remote */
    this_uart->hw_reg->MCR &= ~LOOP_MASK;

    this_uart->hw_reg->MCR &= ~RLOOP_MASK;

    /* set default TX endian */
    this_uart->hw_reg->MM1 &= ~E_MSB_TX_MASK;

    /* set default RX endian */
    this_uart->hw_reg->MM1 &= ~E_MSB_RX_MASK;

    /* default AFM : disabled */
    this_uart->hw_reg->MM2 &= ~EAFM_MASK;

    /* disable TX time guard */
    this_uart->hw_reg->MM0 &= ~ETTG_MASK;

    /* set default RX timeout */
    this_uart->hw_reg->MM0 &= ~ERTO_MASK;

    /* disable fractional baud-rate */
    this_uart->hw_reg->MM0 &= ~EFBR_MASK;

    /* disable single wire mode */
    this_uart->hw_reg->MM2 &= ~ESWM_MASK;

    /* set filter to minimum value */
    this_uart->hw_reg->GFR = 0u;

    /* set default TX time guard */
    this_uart->hw_reg->TTG = 0u;

    /* set default RX timeout */
    this_uart->hw_reg->RTO = 0u;

    /*
     * Configure baud rate divisors. This uses the fractional baud rate divisor
     * where possible to provide the most accurate baud rat possible.
     */
    config_baud_divisors(this_uart, baud_rate);

    /* set the line control register (bit length, stop bits, parity) */
    this_uart->hw_reg->LCR = line_config;

    /* Instance setup */
    this_uart->baudrate = baud_rate;
    this_uart->lineconfig = line_config;
    this_uart->tx_buff_size = TX_COMPLETE;
    this_uart->tx_buffer = (const uint8_t *)0;
    this_uart->tx_idx = 0u;

    /* Default handlers for MSS UART interrupts */
    this_uart->rx_handler       = NULL_HANDLER;
    this_uart->tx_handler       = NULL_HANDLER;
    this_uart->linests_handler  = NULL_HANDLER;
    this_uart->modemsts_handler = NULL_HANDLER;
    this_uart->rto_handler      = NULL_HANDLER;
    this_uart->nack_handler     = NULL_HANDLER;
    this_uart->pid_pei_handler  = NULL_HANDLER;
    this_uart->break_handler    = NULL_HANDLER;
    this_uart->sync_handler     = NULL_HANDLER;

    /* Initialize the sticky status */
    this_uart->status = 0u;
}

/***************************************************************************//**
 * Configure baud divisors using fractional baud rate if possible.
 */
static void
config_baud_divisors
(
    mss_uart_instance_t * this_uart,
    uint32_t baudrate
)
{
    uint32_t baud_value;
    uint32_t baud_value_by_64;
    uint32_t baud_value_by_128;
    uint32_t fractional_baud_value;
    uint32_t pclk_freq;

    this_uart->baudrate = baudrate;

    /* Use the system clock value from hw_platform.h */
    pclk_freq = MSS_AXI_SWITCH_CLK;

    /*
     * Compute baud value based on requested baud rate and PCLK frequency.
     * The baud value is computed using the following equation:
     *      baud_value = PCLK_Frequency / (baud_rate * 16)
     */
    baud_value_by_128 = (8u * pclk_freq) / baudrate;
    baud_value_by_64 = baud_value_by_128 / 2u;
    baud_value = baud_value_by_64 / 64u;
    fractional_baud_value = baud_value_by_64 - (baud_value * 64u);
    fractional_baud_value += (baud_value_by_128 - (baud_value * 128u)) - (fractional_baud_value * 2u);

    /* Assert if integer baud value fits in 16-bit. */
    ASSERT(baud_value <= UINT16_MAX);

    if(baud_value <= (uint32_t)UINT16_MAX)
    {
        if(baud_value > 1u)
        {
            /*
             * Use Frational baud rate divisors
             */
            /* set divisor latch */
            this_uart->hw_reg->LCR |= DLAB_MASK;

            /* msb of baud value */
            this_uart->hw_reg->DMR = (uint8_t)(baud_value >> 8);
            /* lsb of baud value */
            this_uart->hw_reg->DLR = (uint8_t)baud_value;

            /* reset divisor latch */
            this_uart->hw_reg->LCR &= ~DLAB_MASK;

            /* Enable Fractional baud rate */
            this_uart->hw_reg->MM0 |= EFBR_MASK;

            /* Load the fractional baud rate register */
            ASSERT(fractional_baud_value <= (uint32_t)UINT8_MAX);
            this_uart->hw_reg->DFR = (uint8_t)fractional_baud_value;
        }
        else
        {
            /*
             * Do NOT use Fractional baud rate divisors.
             */
            /* set divisor latch */
            this_uart->hw_reg->LCR |= DLAB_MASK;

            /* msb of baud value */
            this_uart->hw_reg->DMR = (uint8_t)(baud_value >> 8u);

            /* lsb of baud value */
            this_uart->hw_reg->DLR = (uint8_t)baud_value;

            /* reset divisor latch */
            this_uart->hw_reg->LCR &= ~DLAB_MASK;

            /* Disable Fractional baud rate */
            this_uart->hw_reg->MM0 &= ~EFBR_MASK;
        }
    }
}

/*******************************************************************************
 * Public Functions
 *******************************************************************************/
/***************************************************************************//**
 * See mss_uart.h for details of how to use this function.
 */
void 
MSS_UART_init
(
    mss_uart_instance_t* this_uart, 
    uint32_t baud_rate,
    uint8_t line_config,
	uint8_t intr_type
)
{
    /* Perform generic initialization */
    global_init(this_uart, baud_rate, line_config, intr_type);

    /* Disable LIN mode */
    this_uart->hw_reg->MM0 &= ~ELIN_MASK;

    /* Disable IrDA mode */
    this_uart->hw_reg->MM1 &= ~EIRD_MASK;

    /* Disable SmartCard Mode */
    this_uart->hw_reg->MM2 &= ~EERR_MASK;

    /* set default tx handler for automated TX using interrupt in USART mode */
    //this_uart->tx_handler = default_tx_handler;
}



/***************************************************************************//**
 * See mss_uart.h for details of how to use this function.
 */
void
MSS_UART_polled_tx
(
    mss_uart_instance_t * this_uart,
    const uint8_t * pbuff,
    uint32_t tx_size
)
{
	uint32_t char_idx = 0u;
	uint32_t size_sent;
	uint8_t status;

	ASSERT(pbuff != ( (uint8_t *)0));
	ASSERT(tx_size > 0u);

	if((pbuff != ((uint8_t *)0)) && (tx_size > 0u))
	{
		/* Remain in this loop until the entire input buffer
		 * has been transferred to the UART.
		 */
		do {
			/* Read the Line Status Register and update the sticky record */
			status = this_uart->hw_reg->LSR;
			this_uart->status |= status;

			/* Check if TX FIFO is empty. */
			if(status & MSS_UART_THRE)
			{
				uint32_t fill_size = TX_FIFO_SIZE;

				/* Calculate the number of bytes to transmit. */
				if(tx_size < TX_FIFO_SIZE)
				{
					fill_size = tx_size;
				}

				/* Fill the TX FIFO with the calculated the number of bytes. */
				for(size_sent = 0u; size_sent < fill_size; ++size_sent)
				{
					/* Send next character in the buffer. */
					this_uart->hw_reg->THR = pbuff[char_idx];
					char_idx++;
				}

				/* Calculate the number of untransmitted bytes remaining. */
				tx_size -= size_sent;
			}
		} while(tx_size);
	}
}

/***************************************************************************//**
 * See mss_uart.h for details of how to use this function.
 */
void
MSS_UART_polled_tx_string
(
    mss_uart_instance_t * this_uart,
    const uint8_t * p_sz_string
)
{
    uint32_t char_idx = 0u;
    uint32_t fill_size;
    uint8_t data_byte;
    uint8_t status;

    ASSERT(p_sz_string != ((uint8_t *)0));

    if(p_sz_string != ((uint8_t *)0))
    {
        /* Get the first data byte from the input buffer */
        data_byte = p_sz_string[char_idx];

        /* First check for the NULL terminator byte.
         * Then remain in this loop until the entire string in the input buffer
         * has been transferred to the UART.
         */
        while(0u != data_byte)
        {
            /* Wait until TX FIFO is empty. */
            do {
                status = this_uart->hw_reg->LSR;
                this_uart->status |= status;
            } while (0u == (status & MSS_UART_THRE));

            /* Send bytes from the input buffer until the TX FIFO is full
             * or we reach the NULL terminator byte.
             */
            fill_size = 0u;
            while((0u != data_byte) && (fill_size < TX_FIFO_SIZE))
            {
                /* Send the data byte */
                this_uart->hw_reg->THR = data_byte;
                ++fill_size;
                char_idx++;
                /* Get the next data byte from the input buffer */
                data_byte = p_sz_string[char_idx];
            }
        }
    }
}

/***************************************************************************//**
 * See mss_uart.h for details of how to use this function.
 */
int8_t
MSS_UART_tx_complete
(
    mss_uart_instance_t * this_uart
)
{
    int8_t ret_value = 0;
    uint8_t status = 0u;

    /* Read the Line Status Register and update the sticky record. */
    status = this_uart->hw_reg->LSR;
    this_uart->status |= status;

    if((TX_COMPLETE == this_uart->tx_buff_size) &&
       ((status & MSS_UART_TEMT) != 0u))
    {
        ret_value = (int8_t)1;
    }

    return ret_value;
}

/***************************************************************************//**
 * See mss_uart.h for details of how to use this function.
 */
size_t
MSS_UART_get_rx
(
    mss_uart_instance_t * this_uart,
    uint8_t * rx_buff,
    size_t buff_size
)
{
    size_t rx_size = 0u;
    uint8_t status = 0u;

    ASSERT(rx_buff != ((uint8_t *)0));
    ASSERT(buff_size > 0u);

    if((rx_buff != (uint8_t *)0) && (buff_size > 0u))
    {
        status = this_uart->hw_reg->LSR;
        this_uart->status |= status;

        while(((status & MSS_UART_DATA_READY) != 0u) &&
              (rx_size < buff_size))
        {
            rx_buff[rx_size] = this_uart->hw_reg->RBR;
            ++rx_size;
            status = this_uart->hw_reg->LSR;
            this_uart->status |= status;
        }
    }
    return rx_size;
}

/***************************************************************************//**
 * See mss_uart.h for details of how to use this function.
 */
size_t
MSS_UART_fill_tx_fifo
(
    mss_uart_instance_t * this_uart,
    const uint8_t * tx_buffer,
    size_t tx_size
)
{
    uint8_t status = 0u;
    size_t size_sent = 0u;

    ASSERT(tx_buffer != ( (uint8_t *)0));
    ASSERT(tx_size > 0);

    /* Fill the UART's Tx FIFO until the FIFO is full or the complete input
     * buffer has been written. */
    if((tx_buffer != ((uint8_t *)0))   &&
       (tx_size > 0u))
    {
        status = this_uart->hw_reg->LSR;
        this_uart->status |= status;

        if(status & MSS_UART_THRE)
        {
            uint32_t fill_size = TX_FIFO_SIZE;

            if(tx_size < TX_FIFO_SIZE)
            {
                fill_size = tx_size;
            }
            /* Fill up FIFO */
            for(size_sent = 0u; size_sent < fill_size; ++size_sent)
            {

                /* Send next character in the buffer. */
                this_uart->hw_reg->THR = tx_buffer[size_sent];
            }
        }
    }
    return size_sent;
}

/***************************************************************************//**
 * See mss_uart.h for details of how to use this function.
 */
uint8_t
MSS_UART_get_rx_status
(
    mss_uart_instance_t * this_uart
)
{
    uint8_t status = MSS_UART_INVALID_PARAM;

    /*
     * Extract UART receive error status.
     * Bit 1 - Overflow error status
     * Bit 2 - Parity error status
     * Bit 3 - Frame error status
     * Bit 4 - Break interrupt indicator
     * Bit 7 - FIFO data error status
     */
    this_uart->status |= (this_uart->hw_reg->LSR);
    status = (this_uart->status & STATUS_ERROR_MASK);
    /* Clear the sticky status after reading */
    this_uart->status = 0u;

    return status;
}

/***************************************************************************//**
 * See mss_uart.h for details of how to use this function.
 */
uint8_t
MSS_UART_get_modem_status
(
    mss_uart_instance_t * this_uart
)
{
    uint8_t status = MSS_UART_INVALID_PARAM;

    /*
     * Extract UART modem status and place in lower bits of "status".
     * Bit 0 - Delta Clear to Send Indicator
     * Bit 1 - Delta Clear to Receive Indicator
     * Bit 2 - Trailing edge of Ring Indicator detector
     * Bit 3 - Delta Data Carrier Detect indicator
     * Bit 4 - Clear To Send
     * Bit 5 - Data Set Ready
     * Bit 6 - Ring Indicator
     * Bit 7 - Data Carrier Detect
     */
    status = this_uart->hw_reg->MSR;

    return status;
}

/***************************************************************************//**
 * MSS_UART_get_tx_status.
 * See mss_uart.h for details of how to use this function.
 */
uint8_t
MSS_UART_get_tx_status
(
    mss_uart_instance_t * this_uart
)
{
    uint8_t status = MSS_UART_TX_BUSY;

    /* Read the Line Status Register and update the sticky record. */
    status = this_uart->hw_reg->LSR;
    this_uart->status |= status;
    /*
     * Extract the transmit status bits from the UART's Line Status Register.
     * Bit 5 - Transmitter Holding Register/FIFO Empty (THRE) status. (If = 1, TX FIFO is empty)
     * Bit 6 - Transmitter Empty (TEMT) status. (If = 1, both TX FIFO and shift register are empty)
     */
    status &= (MSS_UART_THRE | MSS_UART_TEMT);

    return status;
}

#ifdef __cplusplus
}
#endif

//#endif     /* PSE*/

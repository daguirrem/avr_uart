/*
 * Archivo : usart.c
 * Autor   : David A. Aguirre Morales 
 * Contacto: daguirre.m@outlook.com
 *
 * Fecha de creación  : 03/08/2020
 * Última modificación: 20/10/2020
 * 						-Commit: First full functional release:
 * 						-uart_write_array_ISR Implemented
 * 						-uart_write_array fixed
 * 						-RX ISR Actualized
 * 						-Refactor #ifdef USART_LED structure in some
 * 						 functions.
 * 						-uart_read_data and uart_read array implemented
 * 						-uartdt implemented
 * 						-Format
 * 						-TX IRS implemented
 *
 * Descripción:
 * Provee métodos para el uso del periférico USART en modo asíncrono de los 
 * MCU's AVR compatibles.
 * 
 * Nota: 
 * MODO 9 bits no implementado.
 * 
 * Estado:
 * Funcional.
 * 
 * TODO:
 * Refactorización general - en progreso
 * 
 */

#include "uart.h"
#include <util/delay.h>
#include <avr/interrupt.h>

/* Force Inline macro*/
#define __INLINE __attribute__((always_inline)) inline
/*Prototipos de funciones privadas*/
static __INLINE uint8_t usart_flush (void);

/* Periférico USART en uso */
static uart_t *cUSART; 

/*Funciones*/
void uart_init(uart_t *iUSART, uart_bdr_t baudrate, uart_fb_t format_bits, 
		uart_pb_t parity_bits, uart_sb_t stop_bits, uart_mode_t mode)
{
	cUSART = iUSART;
	cUSART->UBRRHn  = (uint8_t)(baudrate >> 8);
	cUSART->UBRRLn  = (uint8_t)(baudrate);
	cUSART->UCSRAn  = _BV(U2X);
	cUSART->UCSRCn  = _BV(URSEL)			|
					 (parity_bits << UPM0)	|
					 (stop_bits   << USBS)	|
					 (format_bits << UCSZ0)	;
	cUSART->UCSRBn |= (mode << TXEN);
	#if USART_LED
	cUSART->LDPORT &= ~(_BV(LEDRX) | _BV(LEDTX));
	cUSART->LDDDR  |= _BV(LEDRX) | _BV(LEDTX);
	#endif /* USART_LED */
}

__INLINE void uart_change_curret_usart(uart_t *iUSART)
{
	cUSART = iUSART;
}

void uart_close(uart_mode_t mode)
{
	usart_flush();
	cUSART->UCSRBn &= ~((mode << TXEN) | (1 << RXCIE));
	#ifdef USART_LED
	cUSART->LDDDR &= ~(_BV(LEDRX) | _BV(LEDTX));
	cUSART->LDPORT &= ~(_BV(LEDRX) | _BV(LEDTX));
	#endif /* USART_LED */
}

static inline uint8_t usart_flush(void)
{
	uint8_t flush = 0;
	while (cUSART->UCSRAn & (1 << RXC)) {
		flush = cUSART->UDRn;
	}
	return flush;
}

uint8_t uart_read_byte(uint16_t timeout)
{
	#ifdef USART_LED
	if (timeout) {
		while (!(cUSART->UCSRAn & (1 << RXC)) && (--timeout)) {
			_delay_ms(1);
		}
		cUSART->LDPORT |= _BV(LEDRX);
		if (!timeout) {
			cUSART->LDPORT ^= _BV(LEDRX);
			return (uint8_t)(-1);
		}
	}
	else {
		while (!(cUSART->UCSRAn & (1 << RXC))){}		
		cUSART->LDPORT |= _BV(LEDRX);
		cUSART->LDPORT ^= _BV(LEDRX);
	}
	#else
	if (timeout) {
		while (!(cUSART->UCSRAn & (1 << RXC)) && (--timeout)) {
			_delay_ms(1);
		}
		if (!timeout) {
			return (uint8_t)(-1);
		}
	} else {
		while (!(cUSART->UCSRAn & (1 << RXC))){}
	}
	#endif /* USART_LED */
	return cUSART->UDRn;
}

uartd_t uart_read_data(uint16_t timeout, databits_t datatype)
{
	uartd_t data = 0;
	databits_t datatype_bak = datatype;

	do {
		data <<= 8;
		data |= uart_read_byte(timeout);
	} while(--datatype);
	
	switch (datatype_bak) {
        default:
        #ifdef UART_REG_8
        case BIT8:
        return (uint8_t) data;
        break;
        #endif /* UART_REG_8 */

        #ifdef UART_REG_16
        case BIT16:
        return (uint16_t) ((data<<8) | (data>>8));
        break;
        #endif /* UART_REG_16 */

        #ifdef UART_REG_32
        case BIT32:
        return (uint32_t)
            ((data&0x000000FF)<<24)|((data&0xFF000000)>>24)|
            ((data&0x0000FF00)<< 8)|((data&0x00FF0000)>> 8);
        break;
        #endif /* UART_REG_32 */

        #ifdef UART_REG_64
        case BIT64:
        return (uint64_t)
            ((data&0x00000000000000FF)<<56)|((data&0xFF00000000000000)>>56)|
            ((data&0x000000000000FF00)<<48)|((data&0x00FF000000000000)>>48)|
            ((data&0x0000000000FF0000)<<24)|((data&0x0000FF0000000000)>>24)|
            ((data&0x00000000FF000000)<< 8)|((data&0x000000FF00000000)>> 8);
        break;
		#endif /* UART_REG_64 */
	}
}

void uart_read_array(uint16_t timeout, void *array, size_t size,
		databits_t datatype)
{
	timeout /= datatype;
	switch (datatype) {
		default:
		#ifdef UART_REG_8
		case BIT8:
			do {
				*((uint8_t*)(array)) = uart_read_data(timeout,BIT8);
				array += 1;
			} while (--size);
			break;
		#endif /* UART_REG_8 */
		#ifdef UART_REG_16
		case BIT16:
			do {
				*((uint16_t*)(array)) = uart_read_data(timeout,BIT16);
				array += 2;
			} while (--size);
			break;
		#endif /* UART_REG_16 */
		#ifdef UART_REG_32
		case BIT32:
			do {
				*((uint32_t*)(array)) = uart_read_data(timeout,BIT32);
				array += 4;
			} while (--size);
			break;
		#endif /* UART_REG_32 */
		#ifdef UART_REG_64
		case BIT64:
			do {
				*((uint64_t*)(array)) = uart_read_data(timeout,BIT64);
				array += 8;
			} while (--size);}
			break;
		#endif /* UART_REG_64 */
	}
}

void uart_read_string(char *array)
{
	char *reference_dir = array;
	while (1) {
		uint8_t byte_in = uart_read_byte(0);
		/* Fin de la trasmisión de la cadena de carácteres */
		if (byte_in == 0xd)
			break;
		/* En caso de que se reciba una solicitud de 'borrar' */
		else if ((byte_in == 0x8) && (array > reference_dir))
			*array-- = 0;
		/* Verificar que no se exceda el tamaño reservado en memoria */
		else if ((array - reference_dir) >= UART_BUFF_SIZE)
			uart_write_byte(0x8);
		else
			*array++ = byte_in;
	}
}

void uart_write_byte(int8_t byte_to_send)
{
	#ifdef USART_LED
	cUSART->LDPORT |= _BV(LEDTX);
	while (!(cUSART->UCSRAn & (1 << UDRE))){}
	cUSART->UDRn = byte_to_send;
	cUSART->LDPORT ^= _BV(LEDTX);
	#else
	while (!(cUSART->UCSRAn & (1 << UDRE))){}
	cUSART->UDRn = byte_to_send;
	#endif
}

void uart_write_data(uartd_t data, databits_t datatype)
{
	do {
		uart_write_byte(data);
		data >>= 8;
	} while (--datatype);
}

void uart_write_array(const void *data, size_t size, databits_t datatype)
{
	do {
		uart_write_data (*((uartd_t*)(data)), datatype);
		data += datatype;
	} while (--size);
}

void uart_write_string(const char *string, uint8_t new_line)
{
	while (*string != 0) {
		uart_write_byte(*(string++));
	}
	if (new_line) {
		uart_write_byte(13); /*CR*/
		uart_write_byte(10); /*LF*/
	}
	else{}
}

void uart_write_string_P(PGM_P string, uint8_t new_line)
{
	uint8_t byte = pgm_read_byte(string);
	while (byte != 0) {
		uart_write_byte(byte);
		byte = pgm_read_byte(++string);
	}
	if (new_line) {
		uart_write_byte(13); /*CR*/
		uart_write_byte(10); /*LF*/
	}
	else{}
}

void clear_string_buffer(char *array)
{
	do {
		*array++ = 0;
	} while (*array != 0);
}

void clear_data_buffer(uint32_t *array, size_t size)
{
	do {
		*array++ = 0;
	} while (--size);
}

uint8_t uart_printf(const char *format, ...)
{
	char buffer[UART_BUFF_SIZE];
	uint8_t rm = 0;
	va_list args;
	va_start(args, format);
	rm = vsnprintf(buffer, UART_BUFF_SIZE - 1, format, args);
	va_end(args);
	uart_write_string(buffer, 0);
	return rm;
}

uint8_t uart_printf_P(PGM_P format, ...)
{
	char buffer[UART_BUFF_SIZE];
	uint8_t rm = 0;
	va_list args;
	va_start(args, format);
	rm = vsnprintf_P(buffer, UART_BUFF_SIZE - 1, format, args);
	va_end(args);
	uart_write_string(buffer, 1);
	return rm;
}

/*Implementaciones ISR--------------------------------------------------------*/
#if defined(UART_ISR)

/* FIXME: 
 * Remove when refactor is done
 */
/* static void (*interrupt_rx0_function)(uint8_t);
#if defined(__AVR_ATmega2560__)
void (*interrupt_rx1_function)(uint8_t);
void (*interrupt_rx2_function)(uint8_t);
void (*interrupt_rx3_function)(uint8_t);
#endif
 */

/* isr_txType
 * Tipo de envío ISR
 */
enum isr_txType
{
	TX_ISR_STRING = 1,
	TX_ISR_DATA
};

/* Custom type interrupt function*/
typedef void (*i_function)(uint8_t);
/* Estructura para manejar IRQ por parte del periférico*/
static struct uart_isr
{
	uint8_t 		status;
	uint8_t 		tx_mode;
	size_t 			tx_lenght;
	databits_t		tx_datatype;
	size_t			rx_cPos;
	i_function 		rx_funct;
	const uint8_t  *tx_buffer;
	uint8_t 		rx_buffer[UART_BUFF_SIZE];
} 	
uart_isr = {0,0,0,0,0,NULL,NULL,{}};

void uart_mem_full(void);
void uart_mem_full(void)
{
	uart_isr.rx_cPos--;
	for (size_t i = 0; i < UART_BUFF_SIZE-2; i++) {
		uart_isr.rx_buffer[i] = uart_isr.rx_buffer[i+1];
	}
}

#ifndef USART_CUST_RXISR
/*Interrupcion RX*/
ISR(USART_RXC_vect)
{
	//uart_printf("current_RXPOS = %d",uart_getRX_POS());
	#ifdef USART_LED
	cUSART->LDPORT ^= _BV(LEDRX);
	*(uart_isr.rx_buffer + uart_isr.rx_cPos) = UDR;
	uart_isr.rx_funct(*(uart_isr.rx_buffer + uart_isr.rx_cPos++));
	if (uart_isr.rx_cPos >= UART_BUFF_SIZE-1)
		uart_mem_full();
	cUSART->LDPORT ^= _BV(LEDRX);
	#else
	*(uart_isr.rx_buffer + uart_isr.rx_cPos) = UDR;
	uart_isr.rx_funct(*(uart_isr.rx_buffer + uart_isr.rx_cPos++));
	if (uart_isr.rx_cPos >= UART_BUFF_SIZE-1)
		uart_mem_full();
	#endif
}
#endif

/*Interrupcion TX*/
ISR(USART_TXC_vect) 
{
	#ifdef USART_LED
	cUSART->LDPORT ^= _BV(LEDTX);

	uint8_t byte_out = *(++uart_isr.tx_buffer);
	if (uart_isr.tx_mode == TX_ISR_STRING) {
		if(byte_out) {
			UDR = byte_out;
		} else{
			UCSRB &= ~_BV(TXCIE);
			uart_isr.status = 0;
			cUSART->LDPORT &= _BV(LEDTX);
		}
	}
	else {
		if (uart_isr.tx_lenght--) {
			UDR = byte_out;
		} else {
			UCSRB &= ~_BV(TXCIE);
			uart_isr.status = 0;
			cUSART->LDPORT &= _BV(LEDTX);
		}
	}
	#else
	if (uart_isr.tx_mode == TX_ISR_STRING) {
		uint8_t byte_out = *(++uart_isr.tx_buffer);
		if(byte_out) {
			UDR = byte_out;
		}
		else {
			UCSRB &= ~_BV(TXCIE);
			uart_isr.status = 0;
			uart_isr.tx_buffer = NULL;
		}
	}
	else {
		uint8_t byte_out = *(uart_isr.tx_buffer++);
		if (uart_isr.tx_lenght--) {
			UDR = byte_out;
		}
		else {
			UCSRB &= ~_BV(TXCIE);
			uart_isr.status = 0;
			uart_isr.tx_datatype = 0;
			uart_isr.tx_buffer = NULL;
		}
	}
	#endif /* USART_LED */
}

/* Funciones ISR*/
/* FIXME:
 * Reformat ATMEGA2560
 */
void uart_enable_RXISR (uart_t *iUSART, void (*target_interrupt)(uint8_t))
{
	usart_flush();
	iUSART->UCSRBn |= (1 << RXCIE);
	#if defined(__AVR_ATmega2560__)
	if 		(iUSART == USART0) interrupt_rx0_function = target_interrupt;
	else if (iUSART == UART1) interrupt_rx1_function = target_interrupt;
	else if (iUSART == UART2) interrupt_rx2_function = target_interrupt;
	else if (iUSART == UART3) interrupt_rx3_function = target_interrupt;
	#else
	uart_isr.rx_funct = target_interrupt;
	#endif /* defined(__AVR_ATmega2560__) */
	sei();
}

uint8_t *uart_getRX_BUFF(void){
	return uart_isr.rx_buffer;
}
uint8_t uart_getRX_POS(void){
	return uart_isr.rx_cPos;
}

/* TODO: 
 * Variante PROGMEM
 */
void uart_write_string_ISR (const char *string)
{
	while(uart_isr.status);
	uart_isr.status = 1;
	uart_isr.tx_buffer = (uint8_t*) string;
	uart_isr.tx_mode = TX_ISR_STRING;
	if(*string) {
		cUSART->UDRn = *string;
		cUSART->UCSRBn |= _BV(TXCIE);
	}
	else {}
}

void uart_write_array_ISR (const void *data, size_t size, databits_t datatype)
{
	size *= datatype;
	while(uart_isr.status);
	uart_isr.status = 1;
	uart_isr.tx_buffer = data;
	uart_isr.tx_lenght = size;
	uart_isr.tx_datatype = datatype;
	uart_isr.tx_mode = TX_ISR_DATA;
	cUSART->UDRn = *((uint8_t*)(data));
	cUSART->UCSRBn |= _BV(TXCIE);
}

#endif /* defined(UART_ISR) */
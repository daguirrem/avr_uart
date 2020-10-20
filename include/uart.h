/*
 * Archivo:  usart.h
 * Autor:  	 David A. Aguirre Morales 
 * Contacto: daguirre.m@outlook.com

 * Fecha de creación:   03/08/2020
 * Última modificación: 14/10/2020
 * 						-UART_REG_xx, uartdt y uartd_t añadidos.
 * 						-Nuevas funciones: TX_ISR, data read.
 * Descripción:
 *  Libreria para hacer uso del periférico USART en modo asíncrono
 *  ATmega y ATtiny compatibles,
 *  ( Diseñado específicamente para:
 *   -ATmega2560 y compatibles
 *   -ATmega16(L)(A) y compatibles
 *   -ATmega8(A) y comptibles
 *   -ATtiny2313(A) y compatibles )
 *
 * Estado:
 *  Refactorización en progreso.
 */

/* TODO:
 * Refactorizar comentarios de todo el archivo.
 * Añadir funciones para manejar flotantes
 */

#ifndef _USART_H_
#define _USART_H_

#include <system.h>

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

#ifndef F_CPU
#define F_CPU 1000000UL
#endif

/* definición USART_LED
 * Descripción:
 *  Define el uso o no (1 o 0 respectivamente) de leds para indicar el estado de
 *  la trasmisión, #NOTA: modificar en "system.h", por defecto no hace uso de LEDs
 */
//#define USART_LED 'B'
#ifdef  USART_LED
#define LEDTX PINB0 /*# PIN Tx*/
#define LEDRX PINB1 /*# PIN Rx*/
#endif /* USART_LED */

/* definición UART_BUFF_SIZE
 * Descripción:
 *  Define el tamaño del buffer a usar, si no es definido con anterioridad
 *  por defecto se inicializa en 100. */
#ifndef UART_BUFF_SIZE
#define UART_BUFF_SIZE 100
#endif

/* TODO:*/
//#define UART_ISR

/* TODO:*/
#define UART_REG_8
#define UART_REG_16
#define UART_REG_32
/* #define UART_REG_64 */
/* #define UART_REG_FL */

#define rdata_c(v) uint##v##_t

#if 	defined(UART_REG_64)
#define 	uartdt rdata_c(64)
#elif 	defined(UART_REG_32)
#define 	uartdt rdata_c(32)
#elif 	defined(UART_REG_16)
#define 	uartdt rdata_c(16)
#elif 	defined(UART_REG_8)
#define 	uartdt rdata_c(8)
#endif /* defined(I2C_REG_64)*/
typedef uartdt uartd_t;

/*--------------------------------------------------------------------------------*/
#if (defined(__AVR_ATmega16__) || defined(__AVR_ATmega16a__) || \
	 defined(__AVR_ATmega8A__))

typedef struct __attribute__((packed)) usart_s
{
	volatile uint8_t 		UBRRLn;
	volatile uint8_t 		UCSRBn;
	volatile uint8_t 		UCSRAn;
	volatile uint8_t 		UDRn;

#if defined(USART_LED)

#if (USART_LED == 'D')

#ifndef PORTD
#error "PORTD NOT AVAILABLE"
#endif

	const volatile uint8_t 	RSVED0[3];
	volatile uint8_t 		LDPIN;
	volatile uint8_t 		LDDDR;
	volatile uint8_t 		LDPORT;
	const volatile uint8_t 	RSVED1[13];

#elif (USART_LED == 'C')

#ifndef PORTC
#error "PORTC NOT AVAILABLE"
#endif

	const volatile uint8_t 	RSVED0[6];
	volatile uint8_t 		LDPIN;
	volatile uint8_t 		LDDDR;
	volatile uint8_t 		LDPORT;
	const volatile uint8_t 	RSVED1[10];

#elif (USART_LED == 'B')

#ifndef PORTB
#error "PORTB NOT AVAILABLE"
#endif

	const volatile uint8_t 	RSVED0[9];
	volatile uint8_t 		LDPIN;
	volatile uint8_t 		LDDDR;
	volatile uint8_t 		LDPORT;
	const volatile uint8_t 	RSVED1[7];

#elif (USART_LED == 'A')

#ifndef PORTA
#error "PORTA NOT AVAILABLE"
#endif

	const volatile uint8_t 	RSVED0[12];
	volatile uint8_t 		LDPIN;
	volatile uint8_t 		LDDDR;
	volatile uint8_t 		LDPORT;
	const volatile uint8_t 	RSVED1[4];
#endif

#else
	const volatile uint8_t 	RSVED0[19];
#endif
	union
	{
		volatile uint8_t 	UCSRCn;
		volatile uint8_t 	UBRRHn;
	};
} uart_t;

#define USART0 (uart_t *)(__SFR_OFFSET + 0x09)

#elif (defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) || \
	   defined(__AVR_ATmega328P__))
typedef struct __attribute__((packed)) usart_s
{
	volatile uint8_t 		UCSRAn;
	volatile uint8_t 		UCSRBn;
	volatile uint8_t 		UCSRCn;
	const volatile uint8_t 	RSVED0;
	volatile uint8_t 		UBRRLn;
	volatile uint8_t 		UBRRHn;
	volatile uint8_t 		UDRn;
} uart_t;

#if defined(__AVR_ATmega328P__)
#define UART0 (uart_t *)(0xC0)
#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
#define UART1 (uart_t *)(0xC8)
#define UART2 (uart_t *)(0xD0)
#define UART3 (uart_t *)(0x130)
#endif
#endif

#elif (defined(__AVR_ATtiny2313a__))
typedef struct __attribute__((packed)) usart_s
{
	volatile uint8_t 		UBRRHn;
	volatile uint8_t 		UCSRCn;
	const volatile uint8_t 	RSVED0[5];
	volatile uint8_t 		UBRRLn;
	volatile uint8_t 		UCSRBn;
	volatile uint8_t 		UBRRAn;
	volatile uint8_t 		UDRn;
} uart_t;

#define UART0 (uart_t *)(__SFR_OFFSET + 0x02)
#endif

/*--------------------------------------------------------------------------------*/
/*enums*/

/* enum uart_bdr_t
 * Descripción:
 *  enum que contiene los valores calculados para UBRR para generar los distintos
 *  baudrates:
 *	baud_max : máximo baud posible con la frecuencia a trabajar del MCU
 *	           (125Kbps para 1MHZ 2.5Mbps para 20MHz, ver datasheet)
 *	baud_xxxx, xxxx : velocidades estándares (2.4kbps-1Mbps)
 */
typedef enum uart_bdr
{
	UART_BAUD_MAX   = 0,
	UART_BAUD_2400  = (uint16_t)(F_CPU / (8.0f * 2400)) - 1,
	UART_BAUD_4800  = (uint16_t)(F_CPU / (8.0f * 4800)) - 1,
	UART_BAUD_9600  = (uint16_t)(F_CPU / (8.0f * 9600)) - 1,
	UART_BAUD_15625 = (uint16_t)(F_CPU / (8.0f * 15625)) - 1,
	UART_BAUD_32250 = (uint16_t)(F_CPU / (8.0f * 31250)) - 1,
	UART_BAUD_62500 = (uint16_t)(F_CPU / (8.0f * 62500)) - 1,
#if (F_CPU >= 2000000UL)
	UART_BAUD_19200 = (uint16_t)(F_CPU / (8.0f * 19200)) - 1,
	UART_BAUD_125000 = (uint16_t)(F_CPU / (8.0f * 125000)) - 1,
#if (F_CPU >= 4000000UL)
	UART_BAUD_38400 = (uint16_t)(F_CPU / (8.0f * 38400)) - 1,
	UART_BAUD_57600 = (uint16_t)(F_CPU / (8.0f * 57600)) - 1,
	UART_BAUD_250000 = (uint16_t)(F_CPU / (8.0f * 250000)) - 1,
#if (F_CPU >= 8000000UL)
	UART_BAUD_115200 = (uint16_t)(F_CPU / (8.0f * 115200)) - 1,
	UART_BAUD_500000 = (uint16_t)(F_CPU / (8.0f * 500000)) - 1,
#if (F_CPU >= 16000000UL)
	UART_BAUD_230400 = (uint16_t)(F_CPU / (8.0f * 234000)) - 1,
	UART_BAUD_1000000 = (uint16_t)(F_CPU / (8.0f * 1000000)) - 1,
#endif /*(F_CPU >= 16000000UL)*/
#endif /*(F_CPU >= 8000000UL)*/
#endif /*(F_CPU >= 4000000UL)*/
#endif /*(F_CPU >= 2000000UL)*/
	BAUD_ENUM_END
} uart_bdr_t;

/* enum uart_fb_t
 * Descripción:
 *  enum para determinar el formato de bits del frame:
 *	UART_FB_x, x : 5 a 8 bits.
 */
typedef enum uart_fb
{
	UART_FB_5 = 0,
	UART_FB_6,
	UART_FB_7,
	UART_FB_8
} uart_fb_t;

/* enum uart_pb_t
 * Descripción:
 *  enum para determinar los bits de paridad.
 *	UART_PB_xxxx, xxxx -> -DISABLE: Sin paridad
 *			              -EVEN:    Paridad par
 *			              -ODD:     Paridad impar
 */
typedef enum uart_pb
{
	UART_PB_DISABLE = 0,
	UART_PB_EVEN = 2,
	UART_PB_ODD
} uart_pb_t;

/* enum uart_sb_t
 * Descripción:
 *  enum para determinar los bits de parada.
 *	UART_SB_x, x -> -1: 1 bit de parada
 *	                -2: 2 bits de parada
 */
typedef enum uart_sb
{
	UART_SB_1 = 0,
	UART_SB_2
} uart_sb_t;

/* enum uart_mode
 * Descripción:
 *  enum para determinar el modo de funcionamiento del periférico.
 *	mode_xxxx, xxxx : tx:   solo modo de trasmisión
 *			  rx:   solo modo de recepción
 *			  txrx: ambos modos
 */
typedef enum uart_mode
{
	UART_MODE_TX = 1,
	UART_MODE_RX,
	UART_MODE_TXRX
} uart_mode_t;


#ifndef DATABITS_E
#define DATABITS_E
/* enum datatype
 * Descripción:
 *  enum para determnar cantidad de bytes de un tipo de datos
 *	intx, x -> cantidad de bits de la variable. (8, 16 o 32)
 */
typedef enum databits
{
	BIT8 = 1,
	BIT16 = 2,
	BIT32 = 4,
	BIT64 = 8
} databits_t;
#endif /* DATABITS_E */

/*--------------------------------------------------------------------------------*/
/*Funciones*/

/* uart_init()
 * Descripción:
 *  Inicializa el periférico /iUSART en modo asícrono con los argumentos de entrada:
 * Argumentos:
 *  -> iUSART: Módulo específico a inicializar.
 *  -> baudrate: baudrate deseado. (ver ENUM)
 *  -> format_bits: formato de bits deseado. (ver ENUM)
 *  -> parity_bits: formato de paridad deseado. (ver ENUM)
 *  -> stop_bits: formato de bits de parada. (ver ENUM)
 *  -> mode: modo de funcionamiento. (ver ENUM)
 * Retorno:
 *  <- Ninguno.
 * */
void uart_init (uart_t *iUSART, uart_bdr_t baudrate, uart_fb_t format_bits,
		uart_pb_t parity_bits, uart_sb_t stop_bits, uart_mode_t mode);

/* uart_change_curret_usart()
 * Descripción:
 *  Cambia el periférico USART actual a /iUSART
 * Argumentos:
 *  -> iUSART: USART deseado.
 * Retorno:
 *  <- Ninguno.
 * */
void uart_change_curret_usart(uart_t *iUSART);

/* uart_close()
 * Descripción:
 *  Apaga el modo /mode de funcionamiento del UART actual
 * Argumentos:
 *  -> mode: Modo que se desea cerrar. (ver ENUM)
 * Retorno:
 *  <- Ninguno.
 */
void uart_close(uart_mode_t mode);

/* uart_read_byte()
 * Descripción:
 *  Recepción de un byte por medio del periférico.
 * Argumentos:
 *  -> timeout (ms): Tiempo de espera antes de retornar error,
 *		             (0) para no activar el timeout.
 * Retorno:
 *  <- uint8_t, byte leído.
 */
uint8_t uart_read_byte(uint16_t timeout);

/* TODO:uart_read_data()
 * Descripción:
 *  
 * Argumentos:
 *  ->
 * Retorno:
 *  <-
 */
uartd_t uart_read_data(uint16_t timeout, databits_t datatype);

/* TODO:uart_read_data()
 * Descripción:
 *  
 * Argumentos:
 *  ->
 * Retorno:
 *  <-
 */
void uart_read_array(uint16_t timeout, void *array, size_t size,
		databits_t datatype);

/* uart_read_string()
 * Descripción:
 *  Recepción de una cadena de carácteres.
 * Argumentos:
 *  -> *string: Arreglo donde se almacenará la cadena leída.
 * Retorno:
 *  <- Ninguno.
 */
void uart_read_string(char *string);

/* uart_write_byte()
 * Descripción:
 *  Envío de un byte /byte_to_send por medio del periférico.
 * Argumentos:
 *  -> byte_to_send, byte a eviar.
 * Retorno:
 *  <- Ninguno.
 */
void uart_write_byte(int8_t byte_to_send);

/* uart_write_data()
 * Descripción:
 *  Envió de una varibale de hasta 32 bits, donde los bytes menos significativos
 *  se envían primero.
 * Argumentos:
 *  -> data: variable a enviar.
 *  -> datatype: tipo de dato de la variable. (ver ENUM datatype)
 * Retorno:
 *  <- Ninguno
 */
void uart_write_data(uartd_t data, databits_t datatype);

/* uart_write_array()
 * Descripción:
 *  Envío de un arreglo que contiene variables de hasta 32 bits.
 * Argumentos:
 *  -> *data: Arreglo que se desea enviar.
 *  -> size: Tamaño del arreglo a enviar.
 *  -> datatype: Tipo de datos almacenados en el arreglo. (ver ENUM datatype)
 * Retorno:
 *  <- Ninguno
 */
void uart_write_array(const void *data, size_t size, databits_t datatype);

/* uart_write_string()
 * Descripción:
 *  Envío de una cadena /(*string) por medio del periférico
 * Argumentos:
 *  -> *string: Arreglo donde se encuentra almacenado el string a enviar.
 *  -> new_line: define el envío o no (1 o 0) de CR LF al final del string.
 * Retorno:
 *  <- Ninguno
 * 
 */
void uart_write_string(const char *string, uint8_t new_line);
void uart_write_string_P(PGM_P string, uint8_t new_line);

/* clear_string_buffer()
 * Descripción:
 *  Borra en un arreglo que almacene una cadena de carácteres.
 * Argumentos:
 *  -> *string: Arreglo a limpiar.
 * Retorno:
 *  <- Ninguno
 * 
 */
void clear_string_buffer(char *string);

/* clear_data_buffer()
 * Descripción:
 *  Limpia todo un arreglo que almacene variables.
 * Argumentos:
 *  -> *array: Arreglo a limpiar.
 *  -> size: Tamaño del arreglo.
 * Retorno:
 *  <- Ninguno
 * */
void clear_data_buffer(uint32_t *array, size_t size);

/* uart_printf()
 * Descripción:
 *  Implementación de printf para uso con el periférico UART
 * Argumentos:
 *  ->*format: Texto a enviar 
 *  ->...: Opcionales dependientes de *format
 * Retorno:
 *  <- (uint8_t) Cantidad de bytes enviados
 */
uint8_t uart_printf (const char *format, ...);
uint8_t uart_printf_P (PGM_P format, ...);

#ifdef UART_ISR
/* uart_enable_RXISR()
 * Descripción:
 *  Establece /(*target_interrupt) como función de interrupcíon RX UART
 * Argumentos:
 *  -> void target_interrupt: función que se desea ejecutar cuando se produzca
 *			                  la interrupción, dicha función debe tener un
 *			                  argumento tipo uint8_t en el cual se almacena
 *			                  el byte leído por la interrupción. (ver ejemplo)
 * Retorno:
 *  <- Ninguno
 */
void uart_enable_RXISR(uart_t *iUSART, void (*target_interrupt)(uint8_t));

// TODO:
uint8_t *uart_getRX_BUFF(void);
uint8_t uart_getRX_POS(void);

/* TODO:uart_write_string_ISR)
 * Descripción:
 *  
 * Argumentos:
 *  ->
 * Retorno:
 *  <-
 */
void uart_write_string_ISR (const char *string);

/* TODO:uart_write_string_ISR)
 * Descripción:
 *  
 * Argumentos:
 *  ->
 * Retorno:
 *  <-
 */
void uart_write_array_ISR (const void *data, size_t size, databits_t datatype);

#endif /* UART_ISR */
#endif /* _USART_H_ */
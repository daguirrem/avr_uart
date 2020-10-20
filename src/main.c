/* 
 * Archivo : main.c
 * Autor   : David A. Aguirre Morales
 * Contacto: daguirre.m@outlook.com
 * 
 * Fecha de creación:    10/10/2020
 * Última modificiación: 20/10/2020
 *
 * Descripción: 
 * Ejemplo de implementación de todas las funciones disponibles en
 * la librería
 * Estado: 
 * Desarrollo, Fin.
 * Pendiente: 
 * 
 */

#include "system.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "uart.h"

#define PC_TEST 1

FUSES = {
	.low = LFUSE_DEFAULT,
	.high = HFUSE_DEFAULT & FUSE_OCDEN
};

const char el_cuervo[] PROGMEM = {
"Una vez en una medianoche triste, mientras meditaba, débil y cansada,\n"
"Durante muchos, un pintoresco y curioso volumen de historias olvidadas.\n"
"Mientras asentía, casi dormitando, de repente vino un tapping,\n"
"A partir de alguien golpeando suavemente, golpeando la puerta de mi cámara.\n"
"\"'Es un visitante\", murmuré, \"llamando a la puerta de mi cámara—\n"
"Solo esto y nada más \".\n"
"\"Ah, claramente recuerdo que fue en el sombrío diciembre;\n"
"Y cada brasa moribunda separaba al fantasma sobre el suelo.\n"
"Deseé ansiosamente el día siguiente;\n"
"De mis libros surge el dolor, el dolor por la perdida Lenore,\n"
"Por la rara y radiante doncella a quien los ángeles llaman Lenore:\n"
"Sin nombre aquí para siempre.\n"
"\"Y el crujido sedoso, triste, incierto de cada cortina púrpura\n"
"Me emocionó, me llenó de terrores fantásticos que nunca antes había sentido;\n"
"Así que ahora, para aquietar los latidos de mi corazón, me paraba repitiendo.\n"
"\"\" Hay un visitante que pide la entrada a la puerta de mi cámara:\n"
"Algún visitante tardío entrando por la puerta de mi habitación;\n"
"Esto es y nada más ”\n."
"\"En ese momento mi alma se hizo más fuerte; no dudando más,\n"
"\"Señor\", dije yo, “o señora, le pido sinceramente su perdón;\n"
"Pero el hecho es que estaba durmiendo la siesta, y tan gentilmente viniste rapeando,\n"
"Y tan débilmente viniste haciendo tapping, tocando en la puerta de mi cámara,\n"
"Que casi no estaba seguro de haberte oído \", allí abrí la puerta de par en par;\n"
"Oscuridad allí y nada más.\n"
"\"Profundamente en esa oscuridad mirando, por mucho tiempo estuve allí preguntándome, temiendo,\n"
"Dudando, soñando sueños, ningún mortal se ha atrevido a soñar antes;\n"
"\"Pero el silencio fue ininterrumpido, y la quietud no dio señal,\"\n"
"Y la única palabra que se habló allí fue la palabra susurrada, \"¿Lenore?\"\n"
"Esto lo susurré, y un eco murmuró de nuevo la palabra: \"¡Lenore!\"\n"
"Simplemente esto y nada más.\n"
"\"De vuelta a la cámara girando, toda mi alma dentro de mí arde,\n"
"Pronto volví a escuchar un golpeteo algo más fuerte que antes.\n"
"\"Seguramente\", dije yo, \"seguramente eso es algo en la celosía de mi ventana;\n"
"Déjame ver, entonces, qué hay allí, y explorar este misterio ...\n"
"Que mi corazón se quede quieto un momento y que este misterio explore;\n"
"\"¡Es el viento y nada más!\"\n"
"\"Abrir aquí tiré el obturador, cuando, con muchos flirteando y flirteando,\n"
"Allí pisó un majestuoso cuervo de los santos días de antaño;\n"
"No hizo la menor reverencia; ni un minuto se detuvo o se quedó;\n"
"Pero, con el mando de un señor o una dama, encaramado sobre la puerta de mi habitación ...\n"
"Encaramado en un busto de Pallas justo encima de la puerta de mi habitación ...\n"
"Encaramado, y sentado, y nada más.\n"
"\"Entonces, esta ave de ébano transforma mi triste fantasía en una sonrisa,\n"
"Por el grave y severo decoro del rostro que llevaba,\n"
"\"Aunque tu cresta sea cortada y afeitada, tú\", dije, \"no estás seguro de nada,\n"
"Asqueroso, sombrío y antiguo Cuervo que vaga desde la costa nocturna.\n"
"¡Dime cuál es tu nombre señorial en la orilla plutoniana de la noche! \"\n"
"El cuervo dijo \"Nunca más\".\n"
"\"Mucho me maravillé de esta avidez sin sentido para escuchar el discurso tan claramente,\n"
"Aunque su respuesta tiene poco significado, poca relevancia aburre;\n"
"Porque no podemos dejar de estar de acuerdo en que ningún ser humano vivo\n"
"Alguna vez ha sido bendecido con ver un pájaro sobre la puerta de su cámara ...\n"
"Pájaro o bestia sobre el busto esculpido sobre la puerta de su cámara,\n"
"Cuyo nombre era \"Nunca Más\".\n"
"\"Pero el Cuervo, sentado solo en el plácido busto, habló solo\n"
"Esa única palabra, como si su alma en esa única palabra se derramara.\n"
"Nada más allá de lo que él pronunciaba, no una pluma, entonces él revoloteaba ...\n"
"Hasta que apenas más que murmuré: \"Otros amigos han volado antes ...\n"
"Al día siguiente me dejará, ya que mis esperanzas han volado antes \".\n"
"Entonces el pájaro dijo \"Nunca más\".\n"
"\"Sobresaltado por el silencio roto por la respuesta tan acertadamente hablada,\n"
"\"Sin duda\", dije, \"lo que pronuncia es su único stock y tienda\".\n"
"Atrapado por un infeliz maestro a quien despiadado desastre\n"
"Siguió rápido y siguió más rápido hasta que sus canciones una carga llevó:\n"
"Hasta las aflicciones de su esperanza que soportaba la carga melancólica.\n"
"De 'Nunca, nunca más' \".\n"
"\"Pero el Cuervo sigue engañando a todos mis gustos en sonreír,\n"
"En línea recta giré un asiento acolchado frente a un ave, un busto y una puerta;\n"
"Luego, al hundirse el terciopelo, me volví a vincular\n"
"Fancy to fancy, pensando en lo que esta ominosa ave de antaño ...\n"
"¿Qué es este ave sombría, desgarbada, espantosa, demacrada y ominosa de antaño?\n"
"Significado en croar \"Nunca más\".\n"
"\"Me senté ocupado en adivinar, pero sin una sílaba expresando\n"
"A las aves cuyos ojos ardientes ahora ardían en el núcleo de mi pecho;\n"
"Esto y más me senté adivinando, con la cabeza relajada reclinada\n"
"En el forro de terciopelo del cojín, la luz de la lámpara tiene un aspecto sombrío,\n"
"Pero cuyo forro de terciopelo violeta con la luz de la lámpara se regodeaba,\n"
"Ella presionará, ¡ah, nunca más!\n"
"\"Entonces, pensé, el aire se volvió más denso, perfumado de un incensario invisible.\n"
"Balanceado por serafines, cuyas pisadas tintinearon en el piso con mechones.\n"
"\"Desgraciado\", exclamé, \"tu Dios te prestó; por estos ángeles te envió\n"
"Respiro: respiro y nepente de tus recuerdos de Lenore;\n"
"Quaff, oh, taff este amable nepenthe y olvida a este Lenore perdido! \"\n"
"El cuervo dijo \"Nunca más\".\n"
"“¡Profeta!” Dije yo, “¡cosa del mal! ¡Profeta todavía, si eres un pájaro o un demonio!”\n"
"Ya sea que Tempter lo haya enviado o si te ha tirado la tempestad aquí en tierra,\n"
"Desolado pero sin desanimarse, en esta tierra desértica encantada:\n"
"En esta casa de Horror encantada, dime con sinceridad, te lo suplico.\n"
"¿Hay, hay bálsamo en Galaad? —Dime, dime, te lo imploro ».\n"
"El cuervo dijo \"Nunca más\".\n"
"“¡Profeta!” Dije yo, “¡cosa del mal! ¡Profeta todavía, si es pájaro o diablo!\n"
"Por ese Cielo que se inclina sobre nosotros, por ese Dios que ambos adoramos,\n"
"Dígale a esta alma cargada de dolor si, dentro del lejano Aidenn,\n"
"Agarrará a una doncella santificada a quien los ángeles llaman Lenore:\n"
"Abrocha una doncella rara y radiante a quien los ángeles llaman Lenore.\n"
"El cuervo dijo \"Nunca más\".\n"
"\"¡Sea esa palabra nuestro signo de despedida, pájaro o demonio!\", Grité, levantándome.\n"
"\"¡Vuelve a la tempestad y la orilla plutoniana de la noche!\n"
"¡No dejes ninguna pluma negra como señal de esa mentira que tu alma ha hablado!\n"
"¡Dejen mi soledad intacta! ¡Quitemos el busto sobre mi puerta!\n"
"¡Saca tu pico de mi corazón, y quita tu forma de mi puerta!\n"
"El cuervo dijo \"Nunca más\".\n"
"\"Y el Cuervo, nunca revoloteando, todavía está sentado, todavía está sentado\n"
"En el pálido busto de Pallas, justo encima de la puerta de mi habitación;\n"
"Y sus ojos tienen todo el aspecto de un demonio que está soñando,\n"
"Y la luz de la lámpara lo apaga, arroja su sombra al suelo;\n"
"Y mi alma de esa sombra que yace flotando en el suelo.\n"
"Se levantará, ¡nunca más!\n\n"
"El cuervo - Edgar Allan Poe\n"
};

void uart_error_handler(void);
void uart_rx_handler(uint8_t byte_in);
void do_test(uint8_t index);

int main (void) {
	uint8_t test = 0;
	uart_init(USART0,UART_BAUD_2400,UART_FB_8,UART_PB_DISABLE,UART_SB_1,
			UART_MODE_TXRX);
		
	while(1) {
		uart_write_string("Ingrese la prueba a realizar: ",0);
		test = uart_read_byte(0);
		uart_write_string(" ",1);
		do_test (test);
	}
}

void uart_error_handler(void) {
	uart_write_string("Error, no se ha enviado lo esperado",1);
	DDRB |= 0x1;
	while(1){
		PORTB ^= 0x1;
		_delay_ms(250);
		uint8_t rst = uart_read_byte(1);
		if (rst == 'R') {
			uart_write_byte(0x8);
			WDTCR |= _BV(WDE);
			while(1);
		}
	}
}

void do_test(uint8_t index) {	
	uint8_t uart_error;
	switch (index){
		case '1':
			/* 1 */
			/* Prueba de envío de string en flash */
			uart_write_string_P(PSTR("Enviando poema en flash"),1);
			uart_write_string_P(el_cuervo,1);
			goto ERROR_TEST;
		case '2':
			/* 2 */
			/* Prueba de envío de string en RAM y envío de una variable uint16_t */
			uart_write_string("Enviando 0xFCAB",1);
			uart_write_data(0xFCAB,BIT16);
			uart_write_string(" ",1);
			goto ERROR_TEST;
		case '3':
			/* 3 */
			/* Prueba de envío de un arreglo de 3 datos de tipo uint32_t */
			uart_write_string("Enviando arreglo tipo 'uint32_t':\r\n"
							  "{0xF32819AB,0x00000082,0x00008193}",1);

			uint32_t array3[] = {0xF32819AB,0x82,0x8193};
			uart_write_array(array3,3,BIT32);
			uart_write_string(" ",1);
			goto ERROR_TEST;
		case '4':
			/* 4 */
			/* Prueba de Lectrua de una variable tipo uint32_t */
			uart_write_string("Esperando valor: 0x42FA8C10",1);
			uint32_t data_in;
			data_in = uart_read_data(0,BIT32);
			uart_printf("Datos leídos: 0x%04X%04X",(uint16_t)(data_in>>16),
						(uint16_t)(data_in));

			if (data_in == 0x42FA8C10)
				uart_write_string("correcto",1);
			else
				uart_error_handler();
			break;
		case '5':
			/* 5 */
			/* Prueba de Lectura de un arreglo de 3 datos tipo uint16_t */
			uart_write_string("Esperando arreglo de 16bits:\r\n"
					"0xACBF\r\n"
					"0x9182\r\n"
					"0x1249",1);

			uint16_t array_in[3] = {};
			uint16_t array_c[3] = {0xACBF,0x9182,0x1249};

			uart_read_array(0,array_in,3,BIT16);
			uart_printf("Arreglo recibido: { %4X, %4X, %4X}",
					array_in[0],array_in[1],array_in[2]);
			
			size_t i;
			for (i = 0; i < 3; i++) {
				if (array_in[i] != array_c[i])
					break;
			}

			if (i == 3) 
				uart_write_string("Correcto",1);
			else
				uart_error_handler();
			break;
		case '6':
			/* 6 */
			/* Prueba de Lectura por ISR e Historial (RX Buffer) */
			uart_write_string("Activando interrupciones",1);
			uart_enable_RXISR(USART0,uart_rx_handler);
			while(1);
			break;
		case '7':
			/* 7 */
			/* Prueba de envío de string por ISR */
			sei();
			_delay_ms(100);
			uart_write_string_ISR("ISR TX en prueba\r\n");
			_delay_ms(1000);
			goto ERROR_TEST;
		case '8':
			/* 8 */
			/* Prueba de envío de arreglo de 3 posiciones tipo uint16_t por ISR */
			sei();
			_delay_ms(100);
			uart_write_string_ISR("Enviando arreglo:");
			_delay_ms(1000);
			uint16_t array8[] = { 0x001F,0x3212,0xA82F};
			uart_write_array_ISR(array8,3,BIT16);
			_delay_ms(1000);
			goto ERROR_TEST;
		default:
ERROR_TEST: // Escribe 'O' para indicar que la pueba ha sido superada
			// o cualquier otra cosa para indicar que hay algún error
			uart_error = uart_read_byte(0);
			uart_write_byte(0x8);
			if(uart_error != 'O') {
				uart_write_string("Error",1);
				uart_error_handler();
			}
			break;
	}
}

void uart_rx_handler(uint8_t byte_in){
	//uart_write_byte(byte_in);
	switch (byte_in) {
	case 'X':
		;
		uint8_t *ptrBuff = uart_getRX_BUFF();
		uart_write_string("Enviando historial RX:\r\n{",0);
		for ( uint8_t i = 0; i < uart_getRX_POS() ; i++ ) {
			uart_printf(" %c,",*(ptrBuff+i));
		}
		uart_write_string("}",1);
		break;
	default:
		break;
	}
}
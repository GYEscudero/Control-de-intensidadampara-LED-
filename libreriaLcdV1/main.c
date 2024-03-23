/*************************************
 * main.c                            *
 *                                   *
 * Created: 2/26/2024 10:42:51 PM    *
 * Author: Gerson Yaser              *
 * Libreria de pantalla              *
 * LCD 2x16  Versión 1               *
 *                                   *
 * Dispositivo: ATmega328P           *
 * Frecuencia: 8 MHz a 5V            *
 * Pruebas: Realizadas en Protoboard *
 *                                   *
 *************************************/
/*************************************************************************************************************
*************************************************************************************************************/
/***										  DIRECTIVAS                                                  ***/
/*************************************************************************************************************
*************************************************************************************************************/
/**/ #define	F_CPU 8000000UL																			  /**/
/**/ #include	<xc.h>																					  /**/
/**/ #include	<avr/io.h>																				  /**/
/**/ #include	<util/delay.h>																			  /**/
/**/ /* Pines pantalla de LCD 2x16 */																	  /**/
/**/ #define	RS	0																					  /**/
/**/ #define	RW	1																					  /**/
/**/ #define	E	2																					  /**/
/**/ #define	D4	3																					  /**/
/**/ #define	D5	4																					  /**/
/**/ #define	D6	5																					  /**/
/**/ #define	D7	6																					  /**/
/*************************************************************************************************************
*************************************************************************************************************/
/***										  PROTOCOLOS DE LA FUNCION.                                   ***/
/*************************************************************************************************************
*************************************************************************************************************/
/**/ void inicializarLCD ();																			  /**/
/**/ void habilitar ();																					  /**/
/**/ void envioNibble (uint8_t datosBits);																  /**/
/**/ void borrarLCD ();																					  /**/
/**/ void inicioRetorno ();																				  /**/
/**/ void enviarComando (uint8_t datoBits);																  /**/
/**/ void enciendeDisplay ();																			  /**/
/**/ void enciendeCursor();																				  /**/
/**/ void escrituraIzqDer();																			  /**/
/**/ void posicionLCD(uint8_t linea, uint8_t columna);													  /**/
/**/ void cadenaCaracteresLCD (uint8_t* cadena, uint8_t tamanio);										  /**/
/**/ void enviarCadena (uint8_t datoBits);																  /**/
/*************************************************************************************************************
*************************************************************************************************************/
/***												VARIABLES											  ***/
/*************************************************************************************************************
*************************************************************************************************************/
/**/ uint8_t dato = 0;						/* Variable para configurar o mostrar textos en la LCD*/      /**/
/**/ uint8_t linea0LCD = 0x00;				/* Direccion en memoria DDRAM, primer linea de la LCD */	  /**/													/**/
/**/ uint8_t linea1LCD = 0x40;				/* Direccion en memoria DDRAM, segunda linea de la LCD */	  /**/
/**/ uint8_t DB7 = 7;						/* Bit para habilitar el acceso en memoria DDRAM */			  /**/
/**/ uint8_t texto1[] = {"GY"};				/* Texto uno a mostrar por pantalla LCD */					  /**/
/**/ uint8_t texto2[] = {"Electronica"};	/* Texto dos a mostrar por pantalla LCD */					  /**/
/*************************************************************************************************************
*************************************************************************************************************/
/***									     FUNCION PRINCIPAL			     							  ***/
/*************************************************************************************************************
*************************************************************************************************************/
/**/ int main(void)																						  /**/
/**/ {	/* Inicio de funcion principal "main" */														  /**/
/**/																									  /**/
/**/	 /* Llamado a funciones */																		  /**/
/**/  	 inicializarLCD ();									/* Inicializa LCD */						  /**/							
/**/	 borrarLCD ();										/* Borra caracteres, deja en blanco LCD */    /**/
/**/	 inicioRetorno();									/* Pone en Linea: 0 y Columna: 0 el cursor*/  /**/
/**/	 enciendeDisplay ();								/* Enciende la pantalla LCD */				  /**/
/**/	 enciendeCursor();									/* Enciende cursor de LCD */				  /**/
/**/	 escrituraIzqDer();									/* Configura escritura izquierda-derecha */	  /**/
/**/	 posicionLCD (linea0LCD, 7);						/* Posiciona texto en ubicacion indicada */   /**/
/**/	 cadenaCaracteresLCD (texto1, (sizeof(texto1))-1);	/* Imprime texto en LCD */					  /**/
/**/	 posicionLCD (linea1LCD, 3);						/* Posiciona texto en ubicacion indicada */   /**/
/**/	 cadenaCaracteresLCD (texto2, (sizeof(texto2))-1);	/* Envia un texto a la LCD a imprimir */	  /**/
/**/																									  /**/	
/**/	 while(1)																						  /**/
/**/	 {																								  /**/
/**/		/* Bucle infinito */																		  /**/
/**/	 }																								  /**/
/**/																									  /**/
/**/ }	/* Final de la funcion principal "main" */														  /**/
/*************************************************************************************************************
*************************************************************************************************************/
/***									   FUNCION "inicializarLCD"										  ***/
/***					- Configura el puerto D para la utilización de la LCD 16x2						  ***/
/*** 			 - Realiza la secuencia de inicializacion para utilizar la LCD 16x2 (4 Bits)			  ***/
/*************************************************************************************************************
*************************************************************************************************************/
/**/ void inicializarLCD ()																				  /**/
/**/ {	/* Inicia funcion "inicializarLCD" */															  /**/
/**/																									  /**/
/**/	 /* Configuración de Puerto D como salida */													  /**/
/**/	 DDRD |= ((1 << RS) | (1 << RW) | (1 << E) | (1 << D4) | (1 << D5) | (1 << D6) | (1 << D7));	  /**/
/**/	 /* Configuracion de pines del Puerto D en bajo */												  /**/
/**/	 PORTD &= (~(1 << RS) & ~(1 << RW) & ~(1 << E) & ~(1 << D4) & ~(1 << D5) & ~(1 << D6) &			  /**/
/**/			   ~(1 << D7));																			  /**/
/**/																									  /**/
/**/	 PORTD &= ~(1 << RS);	/* RS = 0: Los bits D0 a D7 son los comandos para ajustar la LCD */		  /**/	
/**/																									  /**/
/**/		    /* Encendido */																			  /**/
/**/	 /****************************/		/* El bit BF no puede ser comprobado antes de esta */		  /**/
/**/	 /**/               	   /**/		/* intrucción *//* Vizualizador se ajusta en modo 8 bits. */  /**/
/**/	 /**/envioNibble (0b0011); /**/		/* Manda comandos de 4 bit (Nibble) para inicializar LCD*/    /**/
/**/     /****************************/																	  /**/	
/**/				 /**/																				  /**/
/**/				 /**/habilitar ();		/* Habilita (Enable) comandos enviados por (D4 a D7) */		  /**/
/**/				 /**/																				  /**/
/**/	 /****************************/																	  /**/
/**/	 /**/     _delay_ms(4);    /**/		/* No esperar mas de 4.1ms */								  /**/
/**/	 /****************************/																	  /**/
/**/				 /**/																				  /**/
/**/	 /****************************/		/* El bit BF no puede ser comprobado antes de esta */		  /**/		
/**/	 /**/envioNibble (0b0011); /**/		/* intrucción */											  /**/
/**/     /****************************/		/* Vizualizador se ajusta en modo a 8 bits */			      /**/
/**/				 /**/																			      /**/
/**/				 /**/habilitar ();		/* Habilita (Enable) comandos enviados por (D4 a D7) */		  /**/
/**/				 /**/																				  /**/
/**/	 /****************************/																	  /**/
/**/	 /**/     _delay_us(90);   /**/		/* No esperar mas de 100us */								  /**/
/**/	 /****************************/																	  /**/
/**/				 /**/																				  /**/
/**/	 /****************************/		/* El bit BF no puede ser comprobado antes de esta */		  /**/
/**/	 /**/envioNibble (0b0011); /**/		/* intrucción */											  /**/
/**/     /****************************/		/* Vizualizador se ajusta en modo a 8 bits */				  /**/
/**/				 /**/																				  /**/
/**/				 /**/habilitar ();		/* Habilita (Enable) comandos enviados por (D4 a D7) */		  /**/
/**/				 /**/																				  /**/
/**/				 /**/	/* El bit BF se puede comprobar despues de las siguientes  */				  /**/
/**/				 /**/	/* Instrucciones */															  /**/
/**/				 /**/																				  /**/
/**/	 /****************************/		/* Inicio del funcionamiento en modo de 4 bits */			  /**/
/**/     /**/envioNibble (0b0010); /**/		/* Desde este punto, primero se escriben los 4 bits mas */	  /**/
/**/	 /****************************/		/* altos, y luego los 4 bits mas bajos */					  /**/
/**/				 /**/																				  /**/
/**/				 /**/habilitar ();		/* Habilita (Enable) comandos enviados por (D4 a D7) */	      /**/
/**/				 /**/																				  /**/
/**/	 /****************************/																	  /**/
/**/     /**/envioNibble (0b0010); /**/		/* El numero de lineas del vizualizador y la fuente de */	  /**/
/**/	 /**/    habilitar ();     /**/		/* caracteres deben ser definidos. Estos valores no pueden */ /**/
/**/	 /**/envioNibble (0b1100); /**/		/* cambiar mas tarde */										  /**/
/**/	 /****************************/																	  /**/
/**/				 /**/																				  /**/
/**/				 /**/habilitar ();		/* Habilita (Enable) comandos enviados por (D4 a D7) */	      /**/
/**/				 /**/																				  /**/
/**/	 /****************************/																	  /**/
/**/	 /**/envioNibble (0b0000); /**/															          /**/
/**/	 /**/    habilitar ();     /**/		/* Vizualizador apagado */									  /**/
/**/	 /**/envioNibble (0b1000); /**/																	  /**/
/**/	 /****************************/																	  /**/
/**/				 /**/																				  /**/
/**/				 /**/habilitar ();		/* Habilita (Enable) comandos enviados por (D4 a D7) */		  /**/
/**/				 /**/																				  /**/
/**/	 /****************************/																	  /**/
/**/	 /**/envioNibble (0b0000); /**/																	  /**/
/**/	 /**/    habilitar ();     /**/		/* Vizualizador apagado */									  /**/
/**/	 /**/envioNibble (0b0001); /**/																	  /**/
/**/	 /****************************/																	  /**/
/**/				 /**/																				  /**/
/**/				 /**/habilitar ();		/* Habilita (Enable) comandos enviados por (D4 a D7) */		  /**/
/**/				 /**/																				  /**/	
/**/	 /****************************/																	  /**/
/**/	 /**/envioNibble (0b0000); /**/																	  /**/
/**/	 /**/    habilitar ();     /**/		/* Ajusta el modo de introducir los caracteres */			  /**/
/**/	 /**/envioNibble (0b0110); /**/																	  /**/
/**/	 /****************************/																	  /**/ 
/**/				 /**/																				  /**/
/**/				 /**/habilitar ();		/* Habilita (Enable) comandos enviados por (D4 a D7) */	      /**/
/**/				 /**/																				  /**/
/**/	 /* Fin de la inicializacion */																	  /**/
/**/																									  /**/
/**/ }	/* Final de la funcion principal "inicializarLCD" */											  /**/
/*************************************************************************************************************
*************************************************************************************************************/
/***									   FUNCION "borrarLCD"					     					  ***/
/***							- Borra el contenido de la pantalla LCD									  ***/
/*************************************************************************************************************
*************************************************************************************************************/
/**/ void borrarLCD ()																					  /**/
/**/ {	/* Inicia funcion "borrarLCD" */																  /**/
/**/																									  /**/
/**/	 enviarComando (0b00000001);	/* Envia comando de 8 bits de configuracion para la LCD */		  /**/
/**/	 inicioRetorno ();				/* Pone cursor en la posicion 0,0 */						      /**/
/**/																									  /**/
/**/ }	/* Finaliza funcion "borrarLCD" */																  /**/
/*************************************************************************************************************
*************************************************************************************************************/
/***									   FUNCION "inicioRetorno"					     				  ***/
/***							- Manda el cursor en la posicion 0,0									  ***/
/*************************************************************************************************************
*************************************************************************************************************/
/**/ void inicioRetorno ()																				  /**/
/**/ {	/* Inicia funcion "inicioRetorno" */															  /**/
/**/																									  /**/
/**/	 enviarComando (0b00000010);	/* Envia comando de 8 bits de configuracion para la LCD */		  /**/
/**/																									  /**/
/**/ }	/* Finaliza funcion "inicioRetorno" */															  /**/
/*************************************************************************************************************
*************************************************************************************************************/
/***									FUNCION "enciendeDisplay"					     				  ***/
/***									   - Enciende display 											  ***/
/*************************************************************************************************************
*************************************************************************************************************/
/**/ void enciendeDisplay ()																			  /**/
/**/ {	/* Inicia funcion "enciendeDisplay" */															  /**/
/**/																								      /**/
/**/	 enviarComando (0b00001100);	/* Envia comando de 8 bits de configuracion para la LCD */		  /**/
/**/																									  /**/
/**/ }	/* Finaliza funcion "enciendeDisplay" */														  /**/	
/*************************************************************************************************************
*************************************************************************************************************/
/***									FUNCION "enciendeCursor"					     				  ***/
/***									   - Enciende cursor 											  ***/
/*************************************************************************************************************
*************************************************************************************************************/
/**/ void enciendeCursor ()																				  /**/
/**/ {	/* Inicia funcion "enciendeDisplay" */															  /**/
/**/																									  /**/
/**/	 enviarComando (0b00001110);	/* Envia comando de 8 bits de configuracion para la LCD */		  /**/
/**/																									  /**/
/**/ }	/* Finaliza funcion "enciendeDisplay" */														  /**/
/*************************************************************************************************************
*************************************************************************************************************/
/***									FUNCION "escrituraIzqDer"					     				  ***/
/***						- Despliega texto en LCD de izquierda a derecha 							  ***/
/*************************************************************************************************************
*************************************************************************************************************/
/**/ void escrituraIzqDer()																				  /**/
/**/ {	/* Inicia funcion "escrituraIzqDer" */															  /**/
/**/																									  /**/
/**/	 enviarComando (0b00000110);	/* Envia comando de 8 bits de configuracion para la LCD */		  /**/
/**/																									  /**/
/**/ }	/* Finaliza funcion "escrituraIzqDer" */														  /**/
/*************************************************************************************************************
*************************************************************************************************************/
/***										   FUNCION "posicionLCD"					     			  ***/
/***   - Resive las: variable "linea" con valor de 00xh (primer linea) o 60xh para la (segunda linea)     ***/
/***        variable "colum" con un valor de 0 a 15 decimal, al ser sumada las dos variables da la        ***/
/***								pocición a dirigirse en la pantalla LCD								  ***/
/*************************************************************************************************************
*************************************************************************************************************/
/**/ void posicionLCD(uint8_t lin, uint8_t colum)														  /**/										
/**/ {	/* Inicia funcion "posicionLCD" */															      /**/
/**/																									  /**/
/**/	 uint8_t direccion = 0;						/* Variable auxiliar para almacenar la posicion */	  /**/
/**/	 direccion = lin + colum;																		  /**/
/**/	 enviarComando ((1 << DB7) | (direccion));	/* Envia comando de 8 bits de configuracion */		  /**/
/**/												/* para la LCD */									  /**/
/**/ }	/* Finaliza funcion "posicionLCD" */															  /**/
/*************************************************************************************************************
*************************************************************************************************************/
/***									FUNCION "cadenaCaracteresLCD"					     			  ***/
/***						- Resive una cadena de caracteres para mostrar en la LCD					  ***/
/*************************************************************************************************************
*************************************************************************************************************/
/**/ void cadenaCaracteresLCD (uint8_t *cadena, uint8_t tamanio)										  /**/
/**/ {	/* Inicia funcion "cadenaCaracteresLCD" */														  /**/
/**/																									  /**/
/**/	 uint8_t contador = 0;	/* Variable auxiliar para realizar el paso por el ciclo for	*/			  /**/
/**/																									  /**/
/**/	 for (contador = 0; contador < tamanio; contador++)												  /**/
/**/	 {																								  /**/
/**/		 enviarCadena (cadena[contador]);	/* Envia letra por letra la cadena de caracteres para */  /**/  
/**/											/* ser mostrada por la pantalla LCD	*/					  /**/
/**/	 }																								  /**/
/**/																									  /**/
/**/ }	/* Finaliza funcion "cadenaCaracteresLCD" */													  /**/
/*************************************************************************************************************
*************************************************************************************************************/
/***									   FUNCION "enviarComando"					     				  ***/
/***	           - Envia comando de configuracion de 8 bits y lo procesa en 4 bits					  ***/
/*************************************************************************************************************
*************************************************************************************************************/
/**/ void enviarComando (uint8_t datoBits)																  /**/
/**/ {	/* Finaliza funcion "enviarComando" */															  /**/
/**/																									  /**/
/**/	 PORTD &= ~(1 << RS);																			  /**/
/**/																									  /**/
/**/	 uint8_t datoAuxiliar = 0;	/* Variable auxiliar para conservar los 4 bits mas bajos */			  /**/
/**/	 datoAuxiliar = datoBits;																		  /**/
/**/	 datoBits >>= 4;			/* Recorre de posición los 4 bits mas altos a la pocision de los */	  /**/
/**/								/* 4 bits mas bajos */												  /**/
/**/	 envioNibble(datoBits);		/* Manda los cuatro Bits mas "ALTOS" del comando a ejecutar */		  /**/
/**/	 habilitar();				/* Habilita (Enable) comandos mas "ALTOS" enviados por (D4 a D7) */	  /**/
/**/	 envioNibble(datoAuxiliar);	/* Manda los cuatro Bits mas "BAJOS" del comando a ejecutar */		  /**/
/**/	 habilitar();				/* Habilita (Enable) comandos mas "BAJOS" enviados por (D4 a D7) */	  /**/
/**/																									  /**/
/**/ }	/* Finaliza funcion "enviarComando" */															  /**/
/*************************************************************************************************************
*************************************************************************************************************/
/***									   FUNCION "enviarCadena"					     				  ***/
/***				 - Recive cadena de caracteres para ser mostrada en pantalla LCD					  ***/
/*************************************************************************************************************
*************************************************************************************************************/
/**/ void enviarCadena (uint8_t datoBits)																  /**/
/**/ {	/* Finaliza funcion "enviarCadena" */															  /**/
/**/																									  /**/
/**/	 PORTD |= (1 << RS);		/* RS = 0: Los bits D0 a D7 son direcciones de los caracteres a */	  /**/
/**/								/* visualizar */													  /**/
/**/	 uint8_t datoAuxiliar = 0;	/* Variable auxiliar para conservar los 4 bits mas bajos */			  /**/	
/**/	 datoAuxiliar = datoBits;																		  /**/
/**/	 datoBits >>= 4;			/* Recorre de posición los 4 bits mas altos a la pocision de los */	  /**/
/**/								/* 4 bits mas bajos */												  /**/
/**/	 envioNibble(datoBits);		/* Manda los cuatro Bits mas "ALTOS" del comando a ejecutar */		  /**/
/**/	 habilitar();				/* Habilita (Enable) comandos mas "ALTOS" enviados por (D4 a D7) */	  /**/
/**/	 envioNibble(datoAuxiliar);	/* Manda los cuatro Bits mas "BAJOS" del comando a ejecutar */		  /**/
/**/	 habilitar();				/* Habilita (Enable) comandos mas "BAJOS" enviados por (D4 a D7) */	  /**/
/**/																									  /**/
/**/ }	/* Finaliza funcion "enviarCadena" */															  /**/
/*************************************************************************************************************
*************************************************************************************************************/
/***									   FUNCION " habilitar "										  ***/
/***     - Manda a 1 el PIND2 (Enable) 1 milisegundo y en alto y un milisegundo en bajo, de esta forma	  ***/
/***                        activa el comando mandando por los pines PIND4 al PIND7						  ***/
/*************************************************************************************************************
*************************************************************************************************************/
/**/ void habilitar ()																					  /**/
/**/ {	/* Inicia funcion "habilitar" */																  /**/
/**/																									  /**/
/**/	 PORTD |= (1 << E);		/* PinD2 (Enable) en alto "activado" */									  /**/
/**/	 _delay_ms(1);			/* Espera 1 milisegundo */												  /**/
/**/	 PORTD &= ~(1 << E);	/* PinD2 (Enable) en bajo "desactivado" */								  /**/
/**/	 _delay_ms(1);			/* Espera 1 milisegundo */												  /**/	
/**/																									  /**/
/**/ }	/* Finaliza funcion "habilitar" */																  /**/
/*************************************************************************************************************
*************************************************************************************************************/
/***									   FUNCION " envioNibble "										  ***/
/***     - Resive los 4 bit mas altos  en la funcion, asignadolos a a los pines de la LCD del D4 al D7	  ***/
/*************************************************************************************************************
*************************************************************************************************************/
/**/ void envioNibble (uint8_t datosBits)															      /**/
/**/ {	/* Inicia funcion "envioNibble" */																  /**/					
/**/																									  /**/
/**-------------------------------------------------union--------------------------------------------------**/
/**/	 union lcd_16x2			/* Inicio del dato derivado union "lcd_16x2" */							  /**/
/**/	 {																								  /**/
/**++++++++++++++++++++++++++++++++++++++++++++++++struct1+++++++++++++++++++++++++++++++++++++++++++++++++**/
/**/		 struct bit_Lcd			/* Inicio de la estructura "bit_Lcd" */								  /**/
/**/		 {																							  /**/
/**/			 unsigned int lcdD4 : 1;	/* Entero sin signo negativo y campo de Bit del 0 al 1 */	  /**/
/**/			 unsigned int lcdD5 : 1;	/* Entero sin signo negativo y campo de Bit del 0 al 1 */	  /**/
/**/			 unsigned int lcdD6 : 1;	/* Entero sin signo negativo y campo de Bit del 0 al 1 */	  /**/
/**/			 unsigned int lcdD7 : 1;	/* Entero sin signo negativo y campo de Bit del 0 al 1 */	  /**/
/**/		 }						/* Fin de la estructura "bit_Lcd" */								  /**/
/**/		 bitLcd;				/* Etiqueta de la estructura "bit_Lcd" */							  /**/
/**++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++**/
/**/																									  /**/
/**°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°struct2°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°**/
/**/		 struct lcd_4Bits		/* Inicio de la estructura "lcd_4Bits" */							  /**/
/**/		 {																							  /**/
/**/			 unsigned int lcdNibble;	/* Entero sin signo negativo y campo de 32 Bits */			  /**/
/**/		 }						/* Fin de la estructura "lcd_4Bits" */								  /**/
/**/		 lcd4Bits;				/* Etiqueta de la estructura "lcd_4Bits" */							  /**/
/**°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°**/
/**/	 }																								  /**/
/**/	 lcd16x2;			/* Fin del dato derivado union "lcd_16x2" */								  /**/
/**--------------------------------------------------------------------------------------------------------**/
/**/																									  /**/
/**/	 lcd16x2.lcd4Bits.lcdNibble = datosBits;	/* El valor de la variable resivida por */     		  /**/
/**/												/* funcion "datosBits" se le asigna al miembro de */  /**/
/**/												/* "lcdNibble" de la estructura "lcd4Bits" la cual */ /**/
/**/												/* es miembro de la union lcd16x2 */				  /**/
/**/																									  /**/
/**/	 /* La union lcd16x2 guarda el espacio necesario para el valor del miembro de mayor tamaño en */  /**/
/**/     /* en este caso "lcdNibble", al tener el valor de "datosBits" y al guardarse consecutivamente */ /**/
/**/     /* los valores de una union y una estructura en memoria, el valor de datosBits se almacenara */  /**/
/**/	 /* en los miembros lcdD4, lcdD5, lcdD6 y lcdD7 al ser valores con solo un 1 campo de bits  */	  /**/
/**/	 /* cada uno solo almacenara 0 o 1 */															  /**/
/**/																									  /**/
/**/	 /* Setencias de control para asignar Nibble a los pines de puerto D (D4 al D7) */				  /**/
/**/	 if (lcd16x2.bitLcd.lcdD4 == 1)																	  /**/
/**/	 {																								  /**/
/**/		 PORTD |= (1 << D4);																		  /**/
/**/	 }																								  /**/
/**/	 if (lcd16x2.bitLcd.lcdD5 == 1)																	  /**/
/**/	 {																								  /**/
/**/		 PORTD |= (1 << D5);																		  /**/
/**/	 }																								  /**/					
/**/	 if (lcd16x2.bitLcd.lcdD6 == 1)																	  /**/
/**/	 {																							      /**/
/**/		 PORTD |= (1 << D6);																		  /**/
/**/	 }																								  /**/
/**/	 if (lcd16x2.bitLcd.lcdD7 == 1)															          /**/
/**/	 {																						          /**/
/**/		 PORTD |= (1 << D7);																		  /**/
/**/	 }																								  /**/
/**/	 if (lcd16x2.bitLcd.lcdD4 == 0)																      /**/
/**/	 {																							      /**/
/**/		 PORTD &= ~(1 << D4);																		  /**/
/**/	 }																								  /**/
/**/	 if (lcd16x2.bitLcd.lcdD5 == 0)																	  /**/
/**/	 {																								  /**/
/**/		 PORTD &= ~(1 << D5);																		  /**/
/**/	 }																								  /**/
/**/	 if (lcd16x2.bitLcd.lcdD6 == 0)																	  /**/
/**/	 {																								  /**/
/**/		 PORTD &= ~(1 << D6);																		  /**/
/**/	 }																								  /**/
/**/	 if (lcd16x2.bitLcd.lcdD7 == 0)																	  /**/
/**/	 {																								  /**/
/**/		 PORTD &= ~(1 << D7);																		  /**/
/**/ 	 }																								  /**/
/**/																									  /**/
/**/ }	/* Finaliza funcion "envioNibble" */															  /**/
/*************************************************************************************************************
*************************************************************************************************************/






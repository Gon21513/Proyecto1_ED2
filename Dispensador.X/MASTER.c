/* 
 * File:   dispensador.c
 * Author: Luis Pedro Gonzalez 21513
 *
 * Created on 06 de agosto de 2023, 04:09 PM
 */

//*****************************************************************************
// Palabra de configuraci?n
//*****************************************************************************
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (RCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

//*****************************************************************************
// Definici?n e importaci?n de librer?as
//*****************************************************************************
#include <stdint.h>
#include <stdio.h>
#include <pic16f887.h>
#include "I2C.h"
#include <xc.h>
#include "DS3231.h"
#include "LCD.h"
//*****************************************************************************
// Definici?n de variables
//*****************************************************************************
#define _XTAL_FREQ 8000000
#define LED_PIN PORTAbits.RA0 // PIN para el motor DC

char timeStr[9];//alamcena la hora como una cadena 
//variables para las fechas y tiempo

uint8_t segundo; //variable de segundos
uint8_t minuto; //variable de minutos
uint8_t hora; //variable de horas
uint8_t temporal; //variable temporal
uint8_t lastActionMinute = 0xFF; // Valor inicial inválido


//*****************************************************************************
// Definici?n de funciones para que se puedan colocar despu?s del main de lo 
// contrario hay que colocarlos todas las funciones antes del main
//*****************************************************************************
void setup(void);
void Set_sec(uint8_t sec); //Función para setear segundos
void Set_min(uint8_t min); //Función para setear minutos
void Set_hour(uint8_t hour); //Función para setear horas

//*****************************************************************************
// Main
//*****************************************************************************
void main(void) {
    setup();
    Lcd_Init();
    Set_sec(0); //Inicializa los segundos a 0
    Set_min(0); //Inicializa los minutos a 0
    Set_hour(11); //Inicializa las horas a 12
    LED_PIN = 0; // Apaga el LED inicialmente


    
    while(1){
        
        // Lee la hora actual
        Read_Time(&segundo, &minuto, &hora);
        
        //confgurar posicion del cursor
        Lcd_Set_Cursor(1,1);
        sprintf(timeStr, "%02u:%02u:%02u ", hora, minuto, segundo); //Función para pasar variables a cadena de caracteres
        Lcd_Write_String(timeStr); //Mostrar en la LCD
        
        // Comprueba si han pasado exactamente dos minutos desde la última acción
        if (lastActionMinute != 0xFF && (minuto - lastActionMinute == 2 || minuto - lastActionMinute == -58)) {
            // Enciende el LED
            LED_PIN = 1;

            // Espera 3 segundos
            __delay_ms(3000);

            // Apaga el LED
            LED_PIN = 0;

            // Almacena los minutos actuales como la última vez que se realizó la acción
            lastActionMinute = minuto;
        }
        
        // Si lastActionMinute no se ha inicializado, inicialízalo con los minutos actuales
        if (lastActionMinute == 0xFF) {
            lastActionMinute = minuto;
        }
        // Espera un poco antes de la próxima lectura (puedes ajustar este valor)
        __delay_ms(1000);
    }
}
//*****************************************************************************
// Funci?n de Inicializaci?n
//*****************************************************************************
void setup(void){
    ANSEL = 0;
    ANSELH = 0;
    TRISA = 0;
    TRISB = 0;
    TRISD = 0;
    
    PORTA = 0;
    //PORTC = 0;
    PORTB = 0;
    PORTD = 0;
    
    I2C_Master_Init(100000);        // Inicializar Comuncaci?n I2C

    //// --------------- Oscilador --------------- 
    OSCCONbits.IRCF = 0b111; // 8 MHz
    OSCCONbits.SCS = 1; // Seleccionar oscilador interno
}
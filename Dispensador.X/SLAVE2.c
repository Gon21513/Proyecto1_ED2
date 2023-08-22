/*
 * File:   SLAVE2.c - Sensor de distancia
 * Authors: Gabriel Carrera 21216 - Luis Pedro González 21513
 *
 * Created on August 16, 2023, 4:00 PM
 */

// Código de PIC Esclavo 2 - Sensor de distancia

// --------------- Palabra de Configuracion ---------------
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
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

// --------------- Librerias ---------------
#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include "I2C.h" // librería I2C

// --------------- Frecuencia ---------------
#define _XTAL_FREQ 8000000 // Frecuencia 8 MHz

// --------------- Variables ---------------
#define TRIGGER_PIN PORTDbits.RD3 //Pin de TRIGGER
#define ECHO_PIN PORTDbits.RD2 //Pin de ECHO
uint8_t z;
uint8_t dato;
uint8_t distance;

// --------------- Prototipos ---------------
void setup(void); // funcion de configuracion
float ultrasonic_measure_distance(void); // Función para obtener distancia del sensor HC-SR04

// --------------- Loop principal ---------------
void main(void) {
    setup(); // Llamada a la funcion de configuracion
    
    while(1){
        if (ultrasonic_measure_distance() > 10) {
            PORTAbits.RA0 = 1;
        } else {
            PORTAbits.RA0 = 0;
        }
    }
}

// --------------- Rutina de  interrupciones ---------------
void __interrupt() isr(void){ // interrupciones
    // I2C
    if(PIR1bits.SSPIF == 1){ 

        SSPCONbits.CKP = 0;
       
        if ((SSPCONbits.SSPOV) || (SSPCONbits.WCOL)){
            z = SSPBUF;                 // Read the previous value to clear the buffer
            SSPCONbits.SSPOV = 0;       // Clear the overflow flag
            SSPCONbits.WCOL = 0;        // Clear the collision bit
            SSPCONbits.CKP = 1;         // Enables SCL (Clock)
        }

        if(!SSPSTATbits.D_nA && !SSPSTATbits.R_nW) {
            z = SSPBUF;                 // Lectura del SSBUF para limpiar el buffer y la bandera BF
            PIR1bits.SSPIF = 0;         // Limpia bandera de interrupción recepción/transmisión SSP
            SSPCONbits.CKP = 1;         // Habilita entrada de pulsos de reloj SCL
            while(!SSPSTATbits.BF);     // Esperar a que la recepción se complete
            PORTD = SSPBUF;             // Guardar en el PORTD el valor del buffer de recepción
            __delay_us(250);
            
        }else if(!SSPSTATbits.D_nA && SSPSTATbits.R_nW){
            z = SSPBUF;
            BF = 0;
            SSPBUF = ultrasonic_measure_distance(); // Enviar distancia medida por el sensor al maestro
            SSPCONbits.CKP = 1;
            __delay_us(250);
            while(SSPSTATbits.BF);
        }
       
        PIR1bits.SSPIF = 0;    
    }
}

// --------------- Setup General ---------------
void setup(void){
    
// --------------- Definir analogicas ---------------
    ANSEL = 0; //Puertos como I/O digitales
    ANSELH = 0; //Puertos como I/O digitales
    
// --------------- Configurar puertos --------------- 
    
    TRISAbits.TRISA0 = 0;
    
    TRISDbits.TRISD3 = 0; // D3 como salida  - TRIGGER
    TRISDbits.TRISD2 = 1; // D2 como entrada - ECHO
    
    
// --------------- Limpiar puertos ---------------    
    PORTA = 0;
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;
    PORTE = 0;    
    
    // --------------- Habilitar pullups --------------- 
    //OPTION_REGbits.nRBPU = 0; 
    //WPUBbits.WPUB6 = 1;
    //WPUBbits.WPUB7 = 1; 

// --------------- Banderas e interrupciones --------------- 
    INTCONbits.GIE = 1;   // Habilitar interrupciones globales
    INTCONbits.PEIE = 1;  // Habilitar interrupciones de perifericas
    PIR1bits.SSPIF = 0; // Limpiar bandera de interrupción del SPI
    PIE1bits.SSPIE = 1; // Activar bandera del SPI
    
// --------------- Oscilador --------------- 
    OSCCONbits.IRCF = 0b111 ; // establecerlo en 8 MHz
    OSCCONbits.SCS = 1; // utilizar oscilador interno
    
// --------------- TIMER1 --------------- 
    T1CON = 0b00000000;
    
    // Inicializacion del i2C con la libreria
    I2C_Slave_Init(0xB0); // Inicializar Comuncación I2C en address de 0x50 - ESCLAVO
}


float ultrasonic_measure_distance(void){
    uint16_t pulse_duration; //Variable para guardar la duración del pulso
    float distance; //Variable para guardar distancia

    // Send a 10us pulse to trigger pin
    TRIGGER_PIN = 1; //Poner el pin en 1
    __delay_us(10); //Enviar 1 por 10 us
    TRIGGER_PIN = 0; //Apagar Pin

    while (ECHO_PIN == 0); //Mientras sea 0 no hacer nada

    TMR1L = 0x00; //Timer1 en 0
    TMR1H = 0X00;
    T1CONbits.TMR1ON = 1; // Iniciar Timer1
    while ((ECHO_PIN == 1)); // Mientras sea 1 guardar Timer 1 se incrementa
    pulse_duration = (TMR1H << 8 ) + TMR1L; //Guardar variable de Timer1
    T1CONbits.TMR1ON = 0; // Apagar timer1
    distance = (pulse_duration*0.5*1)/58; // Calcular distancia

    return distance; //Retornar distancia
}
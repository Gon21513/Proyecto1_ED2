/* 
 * File:   dispensador.c
 * Authors: Luis Pedro Gonzalez 21513 - Gabriel Carrera 21216
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
//#include <pic16f887.h>
#include "I2C.h"
#include <xc.h>
#include "DS3231.h"
#include "LCD.h"
#include "float_str.h" //Funcion para convertir cadena a texto

//*****************************************************************************
// Definici?n de variables
//*****************************************************************************
#define _XTAL_FREQ 8000000
#define MDC PORTAbits.RA0 // PIN para el motor DC
#define SERVO_PIN PORTCbits.RC2 //para el motor DC

#define pwm0 125 // valor para 0 grados (1.5 ms)
#define pwm45 300 // valor para 45 grados  (2.25 ms)

char timeStr[9];//alamcena la hora como una cadena 
char buffer[30]; //Arreglo para guardar valores de distancia
char buffer2[48]; //Arreglo para guardar valores de temperatura

//variables para las fechas y tiempo

uint8_t segundo; //variable de segundos
uint8_t minuto; //variable de minutos
uint8_t hora; //variable de horas
uint8_t temporal; //variable temporal
uint8_t lastActionMinute = 0xFF; // Valor inicial inválido
uint8_t infrarrojo; // variale para guardar  el valor del infrerrojo en ese moemto 
uint8_t counter = 0;
uint8_t distance; //Variable para lectura de la distancia del slave 2 
float temperatura; //Variable para guardar valor de temperatura


//*****************************************************************************
// Definici?n de funciones para que se puedan colocar despu?s del main de lo 
// contrario hay que colocarlos todas las funciones antes del main
//*****************************************************************************
void setup(void);
void Set_sec(uint8_t sec); //Función para setear segundos
void Set_min(uint8_t min); //Función para setear minutos
void Set_hour(uint8_t hour); //Función para setear horas
void moveServo(unsigned short pulso);//funcion para recibir el pulso y mover servo
void AHT10_Init(void); // Inicializar el sensor AHT10
void AHT10_Soft_Reset(void); // Función para resetear el sensor de temperatura
void AHT10_Read(void); // Función para leer y desplegar temperatura


//*****************************************************************************
// Main
//*****************************************************************************
void main(void) {
    setup();
    Lcd_Init();
    Set_sec(0); //Inicializa los segundos a 0
    Set_min(0); //Inicializa los minutos a 0
    Set_hour(11); //Inicializa las horas a 12
    //setupPWM(); //incia el pwm para el servo 
    //tmr0_setup(); //configuraciones de tmr0


    MDC = 0; // Apaga el LED inicialmente

    AHT10_Soft_Reset(); //Resetear el sensor de temperatura
    AHT10_Init(); //Inicar el sensor de temperatura
    
    while(1){
       
////////////////////////////////////////////////////////////////
        //----------- RTC, Lee la hora actual-----------------------
///////////////////////////////////////////////////////////////
        Read_Time(&segundo, &minuto, &hora);
	AHT10_Read(); //Tomar la temperatura
        
        //confgurar posicion del cursor
        Lcd_Set_Cursor(1,1);
        sprintf(timeStr, "%02u:%02u:%02u ", hora, minuto, segundo); //Función para pasar variables a cadena de caracteres
        Lcd_Write_String(timeStr); //Mostrar en la LCD
        
        // Comprueba si han pasado exactamente dos minutos desde la última acción
        if (lastActionMinute != 0xFF && (minuto - lastActionMinute == 2 || minuto - lastActionMinute == -58)) {
            // Enciende el LED
            MDC = 1;
            // Espera 3 segundos
            __delay_ms(2000);
            // Apaga el LED
            MDC = 0;
            // Almacena los minutos actuales como la última vez que se realizó la acción
            lastActionMinute = minuto;
        }
        
        // Si lastActionMinute no se ha inicializado, inicialízalo con los minutos actuales
        if (lastActionMinute == 0xFF) {
            lastActionMinute = minuto;
        }
        // Espera un poco antes de la próxima lectura 
        __delay_ms(700);
        
///////////////////////////////////////////////////////////     
        //---------------datos de infrarrojo-----------------
///////////////////////////////////////////////////////////////
        
        I2C_Master_Start();
        I2C_Master_Write(0x51);
        infrarrojo = I2C_Master_Read(0);
        I2C_Master_Stop();
        __delay_ms(50);
        
        // Verifica el estado del sensor y muestra la palabra "Sirviendo" si es 1
        if(infrarrojo == 1) {

            Lcd_Set_Cursor(2,1); // Ajusta el cursor a la primera fila
            Lcd_Write_String("WAT");
            moveServo(pwm45); //mover a 45 grados para abrir agua

               
        } else {

            Lcd_Set_Cursor(2,1); // Ajusta el cursor a la primera fila
            Lcd_Write_String("   "); // Borra la palabra "Sirviendo" con espacios
            moveServo(pwm0); //mover a 0 grados para abrir agua

        }

	// --------------- Lectura de distancia del Slave 2 ---------------
        
        I2C_Master_Start();
        I2C_Master_Write(0xB1);
        distance = I2C_Master_Read(0);
        I2C_Master_Stop();
        __delay_ms(50);
        
        floattostr(distance, buffer, 2); //Convertir distancia a cadena
        Lcd_Set_Cursor(2,5); //Setear cursor en 1,1
        Lcd_Write_String(buffer); //Mostrar en Lcd
        
        // Verifica el estado del sensor y muestra la palabra "Sirviendo" si es 1
        if (distance > 10 & distance < 50) {
            Lcd_Set_Cursor(2,12); // Ajusta el cursor a la primera fila
            Lcd_Write_String("ALARM");
        } else {
            Lcd_Set_Cursor(2,9); // Ajusta el cursor a la primera fila
            Lcd_Write_String("          "); // Borra la palabra "Sirviendo" con espacios
        }
        
    }
}
//*****************************************************************************
// Funci?n de Inicializaci?n
//*****************************************************************************
void setup(void){
    ANSEL = 0;
    ANSELH = 0;
    
    //salida de lcd en portb 
    //salida del motor dc en porta0
    //salida del servo en portc2
    TRISA = 0;
    TRISB = 0;
    TRISCbits.TRISC2 = 0; // salida de servo 
    TRISD = 0;
    
    PORTA = 0;
    PORTC = 0;//  no estaba
    PORTB = 0;
    PORTD = 0;
    
    I2C_Master_Init(100000);        // Inicializar Comuncaci?n I2C

    //// --------------- Oscilador --------------- 
    OSCCONbits.IRCF = 0b111; // 8 MHz
    OSCCONbits.SCS = 1; // Seleccionar oscilador interno
    

}


//funcion e delay para mover el servo

void moveServo(unsigned short pulso) {
    // Repetir 10 veces en lugar de 50 para reducir el retardo
    for (int i = 0; i < 10; i++) {
        SERVO_PIN = 1; // Encender el pin de control del servo (inicio del pulso)
        
        // Retardo durante la duración del pulso 
        // Esto determina el ángulo del servo
        for (int j = 0; j < pulso; j++) {
            __delay_us(1);
        }
        
        SERVO_PIN = 0; // Apagar el pin de control del servo, fin del pulso
        
        // Retardo durante el tiempo restante para completar el período de 20ms
        // (2000 - pulso) microsegundos
        for (int j = 0; j < (2000 - pulso); j++) {
            __delay_us(1);
        }
    }
}

// --------------- Funciones para el sensor de temperatura ---------------

void AHT10_Init(void){ //Función para inicializar sensor de temperatura
    __delay_ms(40); //delay de 40ms
    uint8_t status; //variable para status del sensor
    I2C_Master_Start(); //Inicializar comunicación I2C
    I2C_Master_Write(0x70); //Direccion de sensor de temperatura
    I2C_Master_Write(0x71); //Enviar para obtener el status del sensor
    I2C_Master_RepeatedStart(); //Repeated Start
    I2C_Master_Write(0x71); //Leer del sensor de temperatura
    status = I2C_Master_Read(0); //Guardar status
    I2C_Master_Stop(); //detener comunicacion
    
    I2C_Master_Start(); //Inicializar comunicación I2C
    I2C_Master_Write(0x70); //Direccion de sensor de temperatura
    I2C_Master_Write(0xE1); //Enviar secuencia de inicialización
    I2C_Master_Write(0x08);
    I2C_Master_Write(0x00);
    I2C_Master_Stop(); //detener comunicacion

    __delay_ms(10); //delay de 10ms

}

void AHT10_Read(void){ //Función para leer
    uint8_t data[7]; //arreglo para guardar los datos recibidos del sensor de temperatura
    uint8_t r; //Variable para determinar si el sensor está listo para volver a realizar una medición
    
    I2C_Master_Start(); //Inicializar comunicación I2C
    I2C_Master_Write(0x70); //Direccion de sensor de temperatura
    I2C_Master_Write(0xAC); //Enviar secuencia de medición
    I2C_Master_Write(0x33);
    I2C_Master_Write(0x00);
    I2C_Master_Stop(); //detener comunicacion
    __delay_ms(80); //delay de 80ms
    
    I2C_Master_Start(); //Inicializar comunicación I2C
    I2C_Master_Write(0x70); //Direccion de sensor de temperatura
    I2C_Master_Write(0x71); //Secuencia para obtener status
    I2C_Master_RepeatedStart(); //Repeated start
    I2C_Master_Write(0x71); //Dirección mas bit de escritura
    r = I2C_Master_Read(0); //Guardar status
    I2C_Master_Stop(); //detener comunicacion
    
    r = r & 0b00000000; //convertir todas las variables en 0
    while (r != 0b00000000); //Mientras sean 0 no hacer nada
    
    I2C_Master_Start(); //Inicializar comunicación I2C
    I2C_Master_Write(0x71); //Enviar dirección mas bit de escritura
    data[0] = I2C_Master_Read(1); //Guardar
    data[1] = I2C_Master_Read(1); //Guardar status
    data[2] = I2C_Master_Read(1); //Guardar valor de humedad 1
    data[3] = I2C_Master_Read(1); //Guardar valor de humedad 2
    data[4] = I2C_Master_Read(1); //Guardar valor de humedad 3 y temperatura 1
    data[5] = I2C_Master_Read(1); //Guardar valor de temperatura 2
    data[6] = I2C_Master_Read(0); //Guardar valor de temperatura 3
    I2C_Master_Stop(); //detener comunicacion
    
	temperatura = (((uint32_t)data[3] & 0x0F) << 16) + ((uint16_t)data[4] << 8) + data[5]; //Unir datos de temperatura en uno solo
    temperatura = ((temperatura/1048576)*200-50); //Realizar conversión indicada por el fabricante
    floattostr(temperatura, buffer2, 2); //Convertir el dato a cadena
    Lcd_Set_Cursor(1,12); //Setear cursor en 1,12
    Lcd_Write_String(buffer2); //Mostrar en LCD
}

void AHT10_Soft_Reset(void){ //Función de reset 
    __delay_ms(40); //delay de 40ms
    I2C_Master_Start(); //Inicializar comunicación I2C
    I2C_Master_Write(0x70); //Direccion de sensor de temperatura
    I2C_Master_Write(0xBA);//Enviar secuencia de reset
    I2C_Master_Stop(); //detener comunicacion
    __delay_ms(25); //delay de 25ms
}
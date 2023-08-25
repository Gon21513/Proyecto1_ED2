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
#include "IOCB.h" // Libreria para antirrebotes

//*****************************************************************************
// Definici?n de variables
//*****************************************************************************
#define _XTAL_FREQ 8000000
#define MDC PORTAbits.RA0 // PIN para el motor DC
#define SERVO_PIN PORTCbits.RC2 //para el motor DC

#define pwm0 45 // valor para 0 grados (1.5 ms)
#define pwm45 190 // valor para 45 grados  (2.25 ms)

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
float temperature; //Variable para guardar valor de temperatura
uint8_t bandera = 0; //Variable para antirrebotes
uint8_t screen = 0; //variable para selector de modos de pantalla
uint8_t temp_int = 0; // Variable para comparar temperatura

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
void ESP32_Write(void); // Función para enviar datos al ESP32


//*****************************************************************************
// Main
//*****************************************************************************
void main(void) {
    setup();
    Lcd_Init();
    Set_sec(0); //Inicializa los segundos a 0
    Set_min(0); //Inicializa los minutos a 0
    Set_hour(12); //Inicializa las horas a 12

    MDC = 0; // Apaga el LED inicialmente
    
    // --------------- Sensor de temperatura ---------------
    AHT10_Soft_Reset(); //Resetear el sensor de temperatura
    AHT10_Init(); //Inicar el sensor de temperatura
    
    screen = 0;
    distance = 5;
    
    while(1){
        
        ESP32_Write(); // Enviar datos al ESP32 por i2c para adafruit
        
        if (screen == 0){ //Chequear el modo de LCD
            //confgurar posicion del cursor
            Lcd_Set_Cursor(1,1);
            sprintf(timeStr, "%02u:%02u:%02u ", hora, minuto, segundo); //Función para pasar variables a cadena de caracteres
            Lcd_Write_String(timeStr); //Mostrar en la LCD
            floattostr(temperature, buffer2, 2); //Convertir el dato a cadena
            Lcd_Set_Cursor(1,11); //Setear cursor en 1,12
            Lcd_Write_String("T"); //Mostrar en LCD
            Lcd_Set_Cursor(1,12); //Setear cursor en 1,12
            Lcd_Write_String(buffer2); //Mostrar en LCD
        }
        else if (screen == 1){ //Chequear el modo de LCD
            //confgurar posicion del cursor
            Lcd_Set_Cursor(1,1);
            Lcd_Write_String("Alerta! Favor de"); //Mostrar en la LCD
            Lcd_Set_Cursor(2,1);
            Lcd_Write_String("renovar agua.   "); //Mostrar en la LCD
        }
        else if (screen == 2){ //Chequear el modo de LCD
            //confgurar posicion del cursor
            Lcd_Set_Cursor(1,1);
            Lcd_Write_String("Alerta! Favor de"); //Mostrar en la LCD
            Lcd_Set_Cursor(2,1);
            Lcd_Write_String("rellenar comida."); //Mostrar en la LCD
        }
        
        // --------------- Lectura de temperatura ---------------
        AHT10_Read(); //Tomar la temperatura
        
        // Verifica el estado del sensor y muestra la palabra "Sirviendo" si es 1
        if (temperature > 32.6) {
            //screen = 1;
            Lcd_Set_Cursor(1,1);
            Lcd_Write_String("Alerta! Favor de"); //Mostrar en la LCD
            Lcd_Set_Cursor(2,1);
            Lcd_Write_String("renovar agua.   "); //Mostrar en la LCD
        } else if (temperature < 32.6) {
            screen = 0;
        }

////////////////////////////////////////////////////////////////
        //----------- RTC, Lee la hora actual-----------------------
///////////////////////////////////////////////////////////////
        Read_Time(&segundo, &minuto, &hora);
        
        // Comprueba si han pasado exactamente dos minutos desde la última acción
        if (lastActionMinute != 0xFF && (minuto - lastActionMinute == 2 || minuto - lastActionMinute == -58)) {
            // Enciende el LED
            MDC = 1; // Dispensar comida
            Lcd_Set_Cursor(2,1);
            Lcd_Write_String("SIRVIENDO COMIDA"); //Mostrar "COMIDA!" la LCD 
            ESP32_Write(); // Enviar datos al ESP32 por i2c para adafruit
            // Espera 3 segundos
            __delay_ms(2000);
            // Apaga el LED
            MDC = 0;
            Lcd_Set_Cursor(2,1);
            Lcd_Write_String("                "); //borrar "AGUA!" la LCD 
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
            Lcd_Set_Cursor(2,1);
            Lcd_Write_String("SIRVIENDO AGUA!"); //Mostrar "AGUA!" la LCD 
            moveServo(pwm45); //mover a 45 grados para abrir agua
        } else {
            moveServo(pwm0); //mover a 0 grados para abrir agua
            Lcd_Set_Cursor(2,1);
            Lcd_Write_String("                "); //Mostrar "Rellenar comida" la LCD 
        }

	// --------------- Lectura de distancia del Slave 2 ---------------
        
        I2C_Master_Start();
        I2C_Master_Write(0xB1);
        distance = I2C_Master_Read(0);
        I2C_Master_Stop();
        __delay_ms(50);
        
        
        // Verifica el estado del sensor y muestra la palabra "Sirviendo" si es 1
        if (distance > 10 && distance < 50) {
            screen = 2;
        } 
        else {
            screen = 0;
            Lcd_Set_Cursor(1,10);
            Lcd_Write_String("  "); //Borrar en la LCD 
        }
         
        
        
    }
}

// --------------- Rutina de  interrupciones ---------------
void __interrupt() isr(void){ // interrupciones
    if (INTCONbits.RBIF == 1){ //Chequear interrupciones del puerto B
        INTCONbits.RBIF = 0; //Limpiar bandera
        if (PORTBbits.RB7 == 0){ //Ver si se presiono botón RB7
            __delay_ms(2); //delay de 1 ms
            bandera = 1; //Bandera en 1 si se presiono
        }
        if (PORTBbits.RB7 == 1 && bandera == 1){ // Esperar a que se suelte botón RB7
            screen = 0; // establecer la LCD como default
        }
        if (PORTBbits.RB6 == 0){ //Ver si se presiono botón RB6
            __delay_ms(2); //delay de 1 ms
            bandera = 2; //Bandera en 2 si se presiono
        }
        if (PORTBbits.RB6 == 1 && bandera == 2){ // Esperar a que se suelte botón RB6
            Lcd_Set_Cursor(1,1);
            Lcd_Write_String("Nivel de comida:"); //Mostrar "Nivel de comida:" la LCD
            if (distance > 50 || distance < 6){
                Lcd_Set_Cursor(2,1);
                Lcd_Write_String("Alto"); //Mostrar "Alto" la LCD
            }
            else if (distance < 11 || distance > 5){
                Lcd_Set_Cursor(2,1);
                Lcd_Write_String("Bajo"); //Mostrar "Bajo" la LCD
            }
            // Espera 3 segundos
            __delay_ms(2000);
        }
    }
}

// --------------- Setup General ---------------
void setup(void){
    
// --------------- Definir analogicas ---------------
    ANSEL = 0;
    ANSELH = 0;
 
// --------------- Configurar puertos ---------------     
    TRISA = 0; // Para PIN A0 como salida - Motor DC
    TRISBbits.TRISB0 = 0; // PIN B0 como salida - LCD
    TRISBbits.TRISB1 = 0; // PIN B1 como salida - LCD
    TRISBbits.TRISB5 = 1; // PIN B5 como entrada - botones
    TRISBbits.TRISB6 = 1; // PIN B6 como entrada - botones
    TRISBbits.TRISB7 = 1; // PIN B7 como entrada - botones
    TRISCbits.TRISC2 = 0; // PIN C2 como salida - Servomotor 
    TRISD = 0;

// --------------- Limpiar puertos ---------------        
    PORTA = 0;
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;
    
// --------------- Banderas e interrupciones --------------- 
    INTCONbits.GIE = 1;   // Habilitar interrupciones globales
    INTCONbits.PEIE = 1;  // Habilitar interrupciones de perifericas
    INTCONbits.RBIE = 1;  // Habilitar interrupciones en PORTB
    
    // Utilizar la libreria para habilitar pullup e IOCB de cada boton deseado
    ioc_init(7);
    ioc_init(6);
    ioc_init(5);    
    
// --------------- Oscilador --------------- 
    OSCCONbits.IRCF = 0b111 ; // establecerlo en 8 MHz
    OSCCONbits.SCS = 1; // utilizar oscilador interno
    
    // Inicializacion del i2C con la libreria
    I2C_Master_Init(100000); // Inicializar Comuncación I2C en 100 kHz - MAESTRO
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
    
	temperature = (((uint32_t)data[3] & 0x0F) << 16) + ((uint16_t)data[4] << 8) + data[5]; //Unir datos de temperatura en uno solo
    temperature = ((temperature/1048576)*200-50); //Realizar conversión indicada por el fabricante
    
    temp_int = (((uint32_t)data[3] & 0x0F) << 16) + ((uint16_t)data[4] << 8) + data[5]; //Unir datos de temperatura en uno solo
    temp_int = ((temperature/1048576)*200-50); //Realizar conversión indicada por el fabricante
}

void AHT10_Soft_Reset(void){ //Función de reset 
    __delay_ms(40); //delay de 40ms
    I2C_Master_Start(); //Inicializar comunicación I2C
    I2C_Master_Write(0x70); //Direccion de sensor de temperatura
    I2C_Master_Write(0xBA);//Enviar secuencia de reset
    I2C_Master_Stop(); //detener comunicacion
    __delay_ms(25); //delay de 25ms
}

void ESP32_Write(void){ //Función para enviar datos al microcontrolador ESP32
    I2C_Master_Start(); //Inicializar comunicación I2C
    I2C_Master_Write(0x60); //Dirección del esclavo
    I2C_Master_Write(MDC); //Enviar pin de motor DC para saber si está sirviendo comida
    I2C_Master_Write(distance); //Enviar distancia
    I2C_Master_Write(infrarrojo); //Enviar si se activa el infrarrojo
    I2C_Master_Write(temperature); //Enviar temperatura
    I2C_Master_Stop(); //detener comunicacion
    __delay_ms(10); //delay de 10ms
}
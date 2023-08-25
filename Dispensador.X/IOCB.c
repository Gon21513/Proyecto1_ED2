/* 
 * File:   IOCB.c
 * Author: Gabriel Carrera 21216
 * Comments: a
 * Revision history:  a
 */

#include "IOCB.h"

// --------------- Setup del IntPORTB --------------- 
void ioc_init(char pin){
    INTCONbits.RBIE = 1;  // Habilitar interrupciones en PORTB
    OPTION_REGbits.nRBPU = 0; // Habilitar pullups
    
    if(pin == 7){
        WPUBbits.WPUB7 = 1;   // Habilitar pullup en RB7
        IOCBbits.IOCB7 = 1;   // Habilitar interrupciones en RB7
    }
    if(pin == 6){
        WPUBbits.WPUB6 = 1;   // Habilitar pullup en RB6
        IOCBbits.IOCB6 = 1;   // Habilitar interrupciones en RB6
    }
    if(pin == 5){
        WPUBbits.WPUB5 = 1;   // Habilitar pullup en RB5
        IOCBbits.IOCB5 = 1;   // Habilitar interrupciones en RB5
    }
    if(pin == 4){
        WPUBbits.WPUB4 = 1;   // Habilitar pullup en RB4
        IOCBbits.IOCB4 = 1;   // Habilitar interrupciones en RB4
    }
    if(pin == 3){
        WPUBbits.WPUB3 = 1;   // Habilitar pullup en RB3
        IOCBbits.IOCB3 = 1;   // Habilitar interrupciones en RB3
    }
    if(pin == 2){
        WPUBbits.WPUB2 = 1;   // Habilitar pullup en RB2
        IOCBbits.IOCB2 = 1;   // Habilitar interrupciones en RB2
    }
    if(pin == 1){
        WPUBbits.WPUB1 = 1;   // Habilitar pullup en RB1
        IOCBbits.IOCB1 = 1;   // Habilitar interrupciones en RB1
    }
    if(pin == 0){
        WPUBbits.WPUB0 = 1;   // Habilitar pullup en RB0
        IOCBbits.IOCB0 = 1;   // Habilitar interrupciones en RB0
    }
    INTCONbits.RBIF = 0;  // Limpiar bandera de interrupcion de PORTB
}

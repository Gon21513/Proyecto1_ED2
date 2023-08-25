/* 
 * File: IOCB.h Libreria de interrupcion PORTB
 * Author: Gabriel Carrera 21216
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef IOCB_H
#define	IOCB_H

#include <xc.h> // include processor files - each processor file is guarded.   
#include <stdint.h>

// --------------- Setup del Oscilador --------------- 
void ioc_init(char pin);

#endif	/* INT_PORTB_H */


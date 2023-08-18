//Luis Pedro Gonzalez 

#include "DS3231.h"


uint8_t Dec_to_Bcd(uint8_t dec_number); //Funci�n para pasar de decimal a bcd
uint8_t Bcd_to_Dec(uint8_t bcd); //Funci�n para pasar de bcd a decimal
// Funci�n para convertir BCD a binario
uint8_t BCDtoBinary(uint8_t val) {
    return (val & 0x0F) + ((val >> 4) * 10);
}

// Inicializa DS3231
void DS3231_Init(void) {
    // Inicializar el I2C en modo maestro
    I2C_Master_Init(100000);  // 100 kHz es un valor com�n para I2C
    // Si requieres alguna configuraci�n espec�fica para el DS3231, realiza aqu�
}



uint8_t DS3231_ReadSeconds(void) {
    uint8_t secondsBCD;
    
    I2C_Master_Start(); // Comienza la comunicaci�n I2C
    I2C_Master_Write(DS3231_WRITE_ADDR); // Direcci�n del modo escritura
    I2C_Master_Write(0x00); // Direcci�n del registro de segundos- direccion de los minutos en datsheet
    I2C_Master_RepeatedStart(); // Reinicia la comunicaci�n para cambiar a modo de lectura
    I2C_Master_Write(DS3231_READ_ADDR);  // Escribe la direcci�n del DS3231 en modo lectura
    secondsBCD = I2C_Master_Read(0); // Leer segundos
    I2C_Master_Stop();
    
    return BCDtoBinary(secondsBCD); // Convertir a binario y retornar
}



//funciones para conversiones 

uint8_t Read(uint8_t address){ //Funci�n para obtener datos
    uint8_t dato = 0; //Variable temporal
    I2C_Master_Start(); //Iniciar i2c
    I2C_Master_Write(0xD0); //Introducir direcci�n del esclavo
    I2C_Master_Write(address); //Introducir direcci�n 
    I2C_Master_RepeatedStart(); //Restart i2c
    I2C_Master_Write(0xD1); //Introducir direcci�n del esclavo m�s bit de escritura
    dato = I2C_Master_Read(0); //Almacenar dato en variable temporal
    //I2C_Nack(); //Encender bit de not aknowledge e iniciar secuencia de reconocimiento  y transimitir el bit de reconocimiento
    I2C_Master_Stop(); //Stop i2c
    __delay_us(10); //delay de 10 us
    return dato; //Retornar dato
}

void Read_Time(uint8_t *s, uint8_t *m, uint8_t *h){ //Funci�n de obtener valores del tiempo
    *s = Bcd_to_Dec(Read(0x00)); //Obtener segundos
    *m = Bcd_to_Dec(Read(0x01)); //Obtener minutos
    *h = Bcd_to_Dec(Read(0x02)); //Obtener horas
}

void Read_Fecha(uint8_t *d, uint8_t *mo, uint8_t *y){
    *d = Bcd_to_Dec(Read(0x04)); //Obtener d�as
    *mo = Bcd_to_Dec(Read(0x05)); //Obtener mes
    *y = Bcd_to_Dec(Read(0x06)); //Obtener a�o
}

void Set_sec(uint8_t sec){ //Funci�n para setear segundo
    I2C_Master_Start(); //Iniciar I2C
    I2C_Master_Write(0xD0); //Direcci�n del esclavo y bit de escritura
    I2C_Master_Write(0x00); //Direcci�n del registro a modificar
    I2C_Master_Write(Dec_to_Bcd(sec)); //Mandar dato en BCD
    I2C_Master_Stop(); //Terminar i2c
}

void Set_min(uint8_t min){ //Funci�n para setear minutos
    I2C_Master_Start(); //Iniciar I2C
    I2C_Master_Write(0xD0); //Direcci�n del esclavo y bit de escritura
    I2C_Master_Write(0x01); //Direcci�n del registro a modificar
    I2C_Master_Write(Dec_to_Bcd(min)); //Mandar dato en BCD
    I2C_Master_Stop(); //Terminar i2c
}

void Set_hour(uint8_t hour){ //Funci�n para setear horas
    I2C_Master_Start(); //Iniciar I2C
    I2C_Master_Write(0xD0); //Direcci�n del esclavo y bit de escritura
    I2C_Master_Write(0x02); //Direcci�n del registro a modificar
    I2C_Master_Write(Dec_to_Bcd(hour)); //Mandar dato en BCD
    I2C_Master_Stop(); //Terminar i2c
}

void Set_day_week(uint8_t day_week){ //Funci�n para setear d�a de semana
    I2C_Master_Start(); //Iniciar I2C
    I2C_Master_Write(0xD0); //Direcci�n del esclavo y bit de escritura
    I2C_Master_Write(0x03); //Direcci�n del registro a modificar
    I2C_Master_Write(Dec_to_Bcd(day_week)); //Mandar dato en BCD
    I2C_Master_Stop(); //Terminar i2c
}

void Set_day(uint8_t day){ //Funci�n para setear numero de d�a
    I2C_Master_Start(); //Iniciar I2C
    I2C_Master_Write(0xD0); //Direcci�n del esclavo y bit de escritura
    I2C_Master_Write(0x04); //Direcci�n del registro a modificar
    I2C_Master_Write(Dec_to_Bcd(day)); //Mandar dato en BCD
    I2C_Master_Stop(); //Terminar i2c
}

void Set_month(uint8_t month){ //Funci�n para setear mes
    I2C_Master_Start(); //Iniciar I2C
    I2C_Master_Write(0xD0); //Direcci�n del esclavo y bit de escritura
    I2C_Master_Write(0x05); //Direcci�n del registro a modificar
    I2C_Master_Write(Dec_to_Bcd(month)); //Mandar dato en BCD
    I2C_Master_Stop(); //Terminar i2c
}

void Set_year(uint8_t year){ //Funci�n para setear a�o
    I2C_Master_Start(); //Iniciar I2C
    I2C_Master_Write(0xD0); //Direcci�n del esclavo y bit de escritura
    I2C_Master_Write(0x06);
    I2C_Master_Write(Dec_to_Bcd(year)); //Mandar dato en BCD
    I2C_Master_Stop(); //Terminar i2c
}

uint8_t Dec_to_Bcd(uint8_t dec_number){ //Funci�n para pasar de numero decimal a bcd
    uint8_t bcd_number; //Variable para almacenar dato bcd
    bcd_number = 0; //Limpiar numero
    while(1){ //Loop
        if (dec_number >= 10){ //Convertir numero y repetir ciclo hasta que el numero sea menor que 10
            dec_number = dec_number - 10; //Restar 10
            bcd_number = bcd_number + 0b00010000; //Ir sumando diez en bcd
        }
        else { //Suma de n�meros
            bcd_number = bcd_number + dec_number; //Suma
            break; //Salirse del loop
        }
    }
    return bcd_number; //Retornar valor BCD
}

uint8_t Bcd_to_Dec(uint8_t bcd){ //Funci�n para pasar n�meros de bcd a decimal
    uint8_t dec; //Variable para guardar valor
    dec = ((bcd>>4)*10)+(bcd & 0b00001111); // Hacer un corrimiento de bits y sumar con la unidad
    return dec; //Retornar valor
}

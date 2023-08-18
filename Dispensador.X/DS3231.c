//Luis Pedro Gonzalez 

#include "DS3231.h"


uint8_t Dec_to_Bcd(uint8_t dec_number); //Función para pasar de decimal a bcd
uint8_t Bcd_to_Dec(uint8_t bcd); //Función para pasar de bcd a decimal
// Función para convertir BCD a binario
uint8_t BCDtoBinary(uint8_t val) {
    return (val & 0x0F) + ((val >> 4) * 10);
}

// Inicializa DS3231
void DS3231_Init(void) {
    // Inicializar el I2C en modo maestro
    I2C_Master_Init(100000);  // 100 kHz es un valor común para I2C
    // Si requieres alguna configuración específica para el DS3231, realiza aquí
}



uint8_t DS3231_ReadSeconds(void) {
    uint8_t secondsBCD;
    
    I2C_Master_Start(); // Comienza la comunicación I2C
    I2C_Master_Write(DS3231_WRITE_ADDR); // Dirección del modo escritura
    I2C_Master_Write(0x00); // Dirección del registro de segundos- direccion de los minutos en datsheet
    I2C_Master_RepeatedStart(); // Reinicia la comunicación para cambiar a modo de lectura
    I2C_Master_Write(DS3231_READ_ADDR);  // Escribe la dirección del DS3231 en modo lectura
    secondsBCD = I2C_Master_Read(0); // Leer segundos
    I2C_Master_Stop();
    
    return BCDtoBinary(secondsBCD); // Convertir a binario y retornar
}



//funciones para conversiones 

uint8_t Read(uint8_t address){ //Función para obtener datos
    uint8_t dato = 0; //Variable temporal
    I2C_Master_Start(); //Iniciar i2c
    I2C_Master_Write(0xD0); //Introducir dirección del esclavo
    I2C_Master_Write(address); //Introducir dirección 
    I2C_Master_RepeatedStart(); //Restart i2c
    I2C_Master_Write(0xD1); //Introducir dirección del esclavo más bit de escritura
    dato = I2C_Master_Read(0); //Almacenar dato en variable temporal
    //I2C_Nack(); //Encender bit de not aknowledge e iniciar secuencia de reconocimiento  y transimitir el bit de reconocimiento
    I2C_Master_Stop(); //Stop i2c
    __delay_us(10); //delay de 10 us
    return dato; //Retornar dato
}

void Read_Time(uint8_t *s, uint8_t *m, uint8_t *h){ //Función de obtener valores del tiempo
    *s = Bcd_to_Dec(Read(0x00)); //Obtener segundos
    *m = Bcd_to_Dec(Read(0x01)); //Obtener minutos
    *h = Bcd_to_Dec(Read(0x02)); //Obtener horas
}

void Read_Fecha(uint8_t *d, uint8_t *mo, uint8_t *y){
    *d = Bcd_to_Dec(Read(0x04)); //Obtener días
    *mo = Bcd_to_Dec(Read(0x05)); //Obtener mes
    *y = Bcd_to_Dec(Read(0x06)); //Obtener año
}

void Set_sec(uint8_t sec){ //Función para setear segundo
    I2C_Master_Start(); //Iniciar I2C
    I2C_Master_Write(0xD0); //Dirección del esclavo y bit de escritura
    I2C_Master_Write(0x00); //Dirección del registro a modificar
    I2C_Master_Write(Dec_to_Bcd(sec)); //Mandar dato en BCD
    I2C_Master_Stop(); //Terminar i2c
}

void Set_min(uint8_t min){ //Función para setear minutos
    I2C_Master_Start(); //Iniciar I2C
    I2C_Master_Write(0xD0); //Dirección del esclavo y bit de escritura
    I2C_Master_Write(0x01); //Dirección del registro a modificar
    I2C_Master_Write(Dec_to_Bcd(min)); //Mandar dato en BCD
    I2C_Master_Stop(); //Terminar i2c
}

void Set_hour(uint8_t hour){ //Función para setear horas
    I2C_Master_Start(); //Iniciar I2C
    I2C_Master_Write(0xD0); //Dirección del esclavo y bit de escritura
    I2C_Master_Write(0x02); //Dirección del registro a modificar
    I2C_Master_Write(Dec_to_Bcd(hour)); //Mandar dato en BCD
    I2C_Master_Stop(); //Terminar i2c
}

void Set_day_week(uint8_t day_week){ //Función para setear día de semana
    I2C_Master_Start(); //Iniciar I2C
    I2C_Master_Write(0xD0); //Dirección del esclavo y bit de escritura
    I2C_Master_Write(0x03); //Dirección del registro a modificar
    I2C_Master_Write(Dec_to_Bcd(day_week)); //Mandar dato en BCD
    I2C_Master_Stop(); //Terminar i2c
}

void Set_day(uint8_t day){ //Función para setear numero de día
    I2C_Master_Start(); //Iniciar I2C
    I2C_Master_Write(0xD0); //Dirección del esclavo y bit de escritura
    I2C_Master_Write(0x04); //Dirección del registro a modificar
    I2C_Master_Write(Dec_to_Bcd(day)); //Mandar dato en BCD
    I2C_Master_Stop(); //Terminar i2c
}

void Set_month(uint8_t month){ //Función para setear mes
    I2C_Master_Start(); //Iniciar I2C
    I2C_Master_Write(0xD0); //Dirección del esclavo y bit de escritura
    I2C_Master_Write(0x05); //Dirección del registro a modificar
    I2C_Master_Write(Dec_to_Bcd(month)); //Mandar dato en BCD
    I2C_Master_Stop(); //Terminar i2c
}

void Set_year(uint8_t year){ //Función para setear año
    I2C_Master_Start(); //Iniciar I2C
    I2C_Master_Write(0xD0); //Dirección del esclavo y bit de escritura
    I2C_Master_Write(0x06);
    I2C_Master_Write(Dec_to_Bcd(year)); //Mandar dato en BCD
    I2C_Master_Stop(); //Terminar i2c
}

uint8_t Dec_to_Bcd(uint8_t dec_number){ //Función para pasar de numero decimal a bcd
    uint8_t bcd_number; //Variable para almacenar dato bcd
    bcd_number = 0; //Limpiar numero
    while(1){ //Loop
        if (dec_number >= 10){ //Convertir numero y repetir ciclo hasta que el numero sea menor que 10
            dec_number = dec_number - 10; //Restar 10
            bcd_number = bcd_number + 0b00010000; //Ir sumando diez en bcd
        }
        else { //Suma de números
            bcd_number = bcd_number + dec_number; //Suma
            break; //Salirse del loop
        }
    }
    return bcd_number; //Retornar valor BCD
}

uint8_t Bcd_to_Dec(uint8_t bcd){ //Función para pasar números de bcd a decimal
    uint8_t dec; //Variable para guardar valor
    dec = ((bcd>>4)*10)+(bcd & 0b00001111); // Hacer un corrimiento de bits y sumar con la unidad
    return dec; //Retornar valor
}

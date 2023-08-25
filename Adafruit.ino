// Archivo de comunicación para ESP32 de esclavo con PIC maestro y adafruit
// Created by: Gabriel Carrera 21216

#include "Wire.h" // Incluir libreria para i2c, del ejemplo de arduino WireSlave
#include "config.h" // Incluir configuración del WiFi
#define I2C_DEV_ADDR 0x30 // Establecer dirección del ESP32 como esclavo del pic maestro

uint32_t i = 0;
uint8_t DCmotor; // PIN para el motor DC
uint8_t infrared; // variable para guardar  el valor del infrarrojo en ese momento 
uint8_t distance; // Variable para lectura de la distancia del slave 2 
float temperature; // Variable para guardar valor de temperatura

void onRequest(){ // Función para enviar los datos de la comunicación con el maestro
  Wire.print(i++); // Número de bytes
  Wire.print(" Packets.");
  Serial.println("onRequest");
}

void onReceive(int len){ // Función para recibir datos del maestro
  while(Wire.available()){ // Ejecutar cuando esté disponible la comunicación
    DCmotor = Wire.read(); // Leer estado del motor dc
    distance = Wire.read(); // Leer distancia
    infrared = Wire.read(); // Leer estado del infrarrojo y servomotor
    temperature = Wire.read(); // Leer temperatura 
  }
}

// set up the feeds
AdafruitIO_Feed *mdc = io.feed("dcmotor"); //Indicar las feeds de Adafruit
AdafruitIO_Feed *dis = io.feed("distance"); // Indicar las feeds de Adafruit
AdafruitIO_Feed *ir = io.feed("infrared"); // Indicar las feeds de Adafruit
AdafruitIO_Feed *temp = io.feed("temperature"); // Indicar las feeds de Adafruit

void setup() {
  // put your setup code here, to run once:
  Wire.onReceive(onReceive); // Definir función para recibir del maestro
  Wire.onRequest(onRequest); // Definir función para enviar al maestro
  Wire.begin((uint8_t)I2C_DEV_ADDR); // Definir en modo esclavo y su dirección  
  Serial.begin(115200); //Serial a 115200
  while(! Serial); 

  Serial.print("Connecting to Adafruit IO");

  // connect to io.adafruit.com
  io.connect();

  // wait for a connection
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  // we are connected
  Serial.println();
  Serial.println(io.statusText());

}

void loop() {
  // put your main code here, to run repeatedly:
  io.run(); // Mantener Adafruit IO actualizando datos, necesario al inicio del loop para mantener conectado
  // MOSTRAR TODAS LAS VARIABLES ENVIADAS EN EL MONITOR SERIAL PARA COMPROBAR
  Serial.println(DCmotor);
  Serial.println(distance);
  Serial.println(infrared);
  Serial.println(temperature);
  
  mdc->save(DCmotor); //Indicar las feeds de Adafruit
  dis->save(distance); // Indicar las feeds de Adafruit
  ir->save(infrared); // Enviar la variable de infrarrojo a Adafruit en la feed de dispensador de agua
  temp->save(temperature); // Enviar la variable de temperatura a Adafruit en la feed de temperatura

  delay(3000); //Enviar cada 3s
}

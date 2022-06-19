#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
//#include <Servo.h>
//#include <EEPROM.h>


// Código hecho por Andrés


// Modo depuración (Dar información por el puerto serie)
#define DEBUG 0


// Lista de dispositivos del Cohete 2 7/ABR/22
/*
   Módulo GPS NEO 6M
   Sensor BMP280
   Tarjeta SD
   EEPROM 24FC512
   Sensor Hall
   Sensor de temperatura

   Apertura paracaidas electroimanes
   Zumbador alarma
*/



//-------------------------------------------------
//              Parámetros Cohete
//-------------------------------------------------
#define TIEMPO_LANZAMIENTO    600000       // ms (Tiempo maximo de espera desde que hay señal GPS)
#define ACC_START             2.0          // g
#define T_MIN_PARACAIDAS      4000         // ms
#define T_MAX_PARACAIDAS      13000        // ms
#define DIF_ALTURA_APERTURA   20.0         // m
#define DIF_ALTURA_ALARMA     200.0        // m
#define T_MAX_ALARMA          30000        // ms


//Control apertura
float alt_max = 0.0;
uint16_t t_inicio = 0;
#define FLIGHT_TIME millis() - t_inicio



//-------------------------------------------------
//             Declaración de pines
//-------------------------------------------------
#define PIN_MICRO_SD_CS   10
#define PIN_LED_ERROR     9
#define PIN_LED_READY     2
#define PIN_ZUMBADOR      16  // (A2)
#define PIN_GPS_TX        3
#define PIN_GPS_RX        4
#define PIN_ELECTROIMAN   15  // (A1)
#define PIN_PULSADOR      6
#define PIN_HALL          7
#define PIN_LM35          A0



//-------------------------------------------------
//              MODULOS Y SENSORES
//-------------------------------------------------

// Cosas acelerómetro
#define KX134    0x1F // 0x1E
#define OFFSET_X 0.00
#define OFFSET_Y 0.08
#define OFFSET_Z 0.00


// Cosas SD
#define SSpin 10
File *archivo;


// GPS
#if _SS_MAX_RX_BUFF != 128
#error "LEEM ERROR: No esta definido el buffer del SofwareSerial.h en 128 bytes. Hablar con Carlos/Andrés para solucionar problema"
#endif
SoftwareSerial ss(PIN_GPS_TX, PIN_GPS_RX);
float GPS_ALT;
float GPS_VEL;
float GPS_LON = TinyGPS::GPS_INVALID_ANGLE;
float GPS_LAT = TinyGPS::GPS_INVALID_ANGLE;
uint8_t GPS_SEC;
uint8_t GPS_MIN;
uint8_t GPS_HOU;
uint8_t GPS_SAT;
//static void smartdelay(unsigned long ms);


// Presion
Adafruit_BMP280 bmp;
float presion_referencia;


// EEPROM I2C
#define EEPROM_I2C_ADDRESS 0x50
uint16_t eeprom_mem = 0;


// Guardar datos en EEPROM INTERNA
/*
  #define T_ALMACENAMIENTO 200
  uint16_t mem = 0;
  unsigned long t_guardar = 0;
*/

// Variables
float X_out;
float Y_out;
float Z_out;
float T_BMP;
float Altitud_BMP;
float Presion_BMP;
float T_EXT;
float HALL;



void setup() {


  // 0. DECLARACIONES

#if DEBUG == 1
  Serial.begin(115200);
#endif
  pinMode(PIN_LED_ERROR, OUTPUT);
  pinMode(PIN_LED_READY, OUTPUT);
  pinMode(PIN_ZUMBADOR, OUTPUT);
  pinMode(PIN_ELECTROIMAN, OUTPUT);
  pinMode(PIN_PULSADOR, INPUT_PULLUP);
  pinMode(PIN_HALL, INPUT);


  //SPISettings(1000000, MSBFIRST, SPI_MODE0);



  // 1. INICIALIZACION Y TEST DE FUNCIONAMIENTO

  digitalWrite(PIN_LED_ERROR, 1);
  paracaidas_close();
  zumbador_off();

  // Wire.begin para todos los modulos
  Wire.begin();
  delay(50);

/*
  // Lectura de datos I2C
  EEPROM_I2C_Lectura_datos();
  while (true) {
    delay(1);
  }
*/


  if (!KX134_64g_init()) {
#if DEBUG == 1
    Serial.println("Error KX134");
#endif
    error_inicio();
  }


  if (!bmp.begin()) {
#if DEBUG == 1
    Serial.println("Error BMP280");
#endif
    error_inicio();
  }


  pinMode(SSpin, OUTPUT);
  if (!SD.begin(SSpin)) {
#if DEBUG == 1
    Serial.println("Error SD");
#endif
    error_inicio();
  }


  archivo = &(SD.open("datos.txt", FILE_WRITE));
  if (archivo == NULL) {
#if DEBUG == 1
    Serial.println("Error Archivo SD");
#endif
    error_inicio();
  }


  if (!init_EEPROMI2C() == NULL) {
#if DEBUG == 1
    Serial.println("Error EEPROM I2C");
#endif
    error_inicio();
  }

  if (!gps_init_NEO6M()) {
#if DEBUG == 1
    Serial.println("Error GPS");
#endif
    error_inicio();
  }




  // 2. ESPERA A RECIBIR SEÑAL GPS VALIDA
#if DEBUG == 1
  Serial.println("Start OK, waiting for GPS valid signal");
#endif
  //gps_wait_signal();
  digitalWrite(PIN_LED_ERROR, 0);




  // 3. PREVIO AL LANZAMIENTO
  for (uint8_t i = 0; i < 50 ; i++) {
    presion_referencia += bmp.readPressure();
  }
  presion_referencia = presion_referencia / 5000.0;




  // 4. ESPERAR TIEMPO DE LANZAMIENTO
  uint16_t var = millis();
  t_inicio = millis();
  Z_out = 0.0;
  while ((millis() - t_inicio) < TIEMPO_LANZAMIENTO) {

    // Avisador acustico 1:
    if ((millis() - var) >= 500) {
      zumbador_on();
      digitalWrite(PIN_LED_READY, 0);
    }
    if ((millis() - var) >= 1000) {
      zumbador_off();
      digitalWrite(PIN_LED_READY, 1);
      var = millis();
    }

    // Por si accidentalmente se lanza antes, salir del bucle
    // Lectura de aceleración
    KX134_64g_read_acc();
    if (abs(Z_out) > ACC_START * 2) {
      zumbador_off();
      archivo->write(0x41);
      archivo->write(0x41);     // 'DD'
      archivo->flush();
      break;
    }
  }



  /*
  // 5. DETECTAR ACELRERACIÓN MÁXIMA E INICIAR TOMA DE DATOS
  digitalWrite(PIN_LED_READY, 1);
  var = millis();
  t_inicio = millis();
  while (true) {

    // Toma de datos
    Toma_de_datos();

    // Avisador acustico 2:
    if ((millis() - var) >= 800) {
      zumbador_on();
    }
    if ((millis() - var) >= 1000) {
      zumbador_off();
      var = millis();
    }

    // Aceleración despegue detectada
    if (abs(Z_out) > ACC_START) {
      zumbador_off();
      archivo->write(0x41);
      archivo->write(0x41);     // 'DD'
      archivo->flush();
      break;
    }
  }
  */

  digitalWrite(PIN_LED_READY, 1);
  digitalWrite(PIN_LED_ERROR, 1);
  t_inicio = millis();


}


void loop()
{

  uint16_t tiempo;

  tiempo = millis();


  Toma_de_datos();

  
  // CONTROL DEL PARACAIDAS
  if (Altitud_BMP > alt_max && (FLIGHT_TIME < T_MIN_PARACAIDAS)) {
    alt_max = Altitud_BMP;
  }

  if ( (alt_max > (Altitud_BMP + DIF_ALTURA_APERTURA) && FLIGHT_TIME < T_MIN_PARACAIDAS)  ||  FLIGHT_TIME > T_MAX_PARACAIDAS)  {
    paracaidas_open();
    digitalWrite(PIN_LED_ERROR, 0);
    archivo->write(0x41);
    archivo->write(0x41);     // 'AA'
    archivo->flush();
    alt_max = 8000.0;
  }



  // CONTROL DE ACTIVACION DE ALARMA
  if ( (alt_max > (Altitud_BMP + DIF_ALTURA_ALARMA)) || FLIGHT_TIME > T_MAX_ALARMA)  {
    zumbador_on();
  }

  if (millis() - tiempo < 10)
  {
    delay(10-millis()+tiempo); //delay de lo que falta para llegar a 10
  }

}




/********************************************************
                     Toma de datos
*********************************************************/

void Toma_de_datos() {


  // Lectura de datos:
  T_BMP = bmp.readTemperature();
  Presion_BMP = bmp.readPressure();
  Altitud_BMP = bmp.readAltitude(); //bmp.readAltitude(presion_referencia);
  KX134_64g_read_acc();
  gps_read();

  // Hall
  HALL = !digitalRead(PIN_HALL);

  // Temperatura LM35
  T_EXT = (float)analogRead(PIN_LM35);
  T_EXT = T_EXT * 0.488759;



  // Datos serie (solo DEBUG)
#if DEBUG == 1
  Serial.print(X_out);
  Serial.write('\t');
  Serial.print(Y_out);
  Serial.write('\t');
  Serial.print(Z_out);
  Serial.write('\t');
  Serial.print(T_EXT);
  Serial.write('\t');
  Serial.print(Presion_BMP);
  Serial.write('\t');
  Serial.print(Altitud_BMP);
  Serial.write('\t');
  Serial.print(T_BMP);
  Serial.write('\t');
  Serial.print(GPS_ALT);
  Serial.write('\t');
  Serial.print(GPS_LAT);
  Serial.write('\t');
  Serial.print(GPS_LON);
  Serial.write('\t');
  Serial.print(GPS_HOU);
  Serial.write(':');
  Serial.print(GPS_MIN);
  Serial.write(':');
  Serial.print(GPS_SEC);
  Serial.write('\t');
  Serial.print(GPS_SAT);
  Serial.write('\n');
  Serial.print(HALL);
  Serial.write('\n');
#endif



  // EEPROM I2C
  EEPROM_I2C_Almacena_datos();

  // Tarjeta SD
  SD_Almacena_datos();


  // EEPROM INTERNA (si procede cada T_ALMACENAMIENTO)
  // EEPROM_Almacena_datos();



}




/********************************************************
                       Escritura SD
*********************************************************/


void SD_Almacena_datos() {
  // Escritura SD
  archivo->write(0x45);
  archivo->write(0x45);     // 'EE'
  archivo->write((byte)(FLIGHT_TIME & 0x00FF));
  archivo->write((byte)((FLIGHT_TIME & 0xFF00) >> 8));
  archivo->write((byte*)(&X_out), 4);
  archivo->write((byte*)(&Y_out), 4);
  archivo->write((byte*)(&Z_out), 4);
  archivo->write((byte*)(&T_EXT), 4);
  archivo->write((byte*)(&Presion_BMP), 4);
  archivo->write((byte*)(&Altitud_BMP), 4);
  archivo->write((byte*)(&T_BMP), 4);
  archivo->write((byte*)(&HALL), 4);
  archivo->write((byte*)(&GPS_ALT), 4);
  archivo->write((byte*)(&GPS_LAT), 4);
  archivo->write((byte*)(&GPS_LON), 4);
  archivo->write((byte*)(&GPS_VEL), 4); 
  archivo->write(GPS_HOU);
  archivo->write(GPS_MIN);
  archivo->write(GPS_SEC);
  archivo->flush();
}




/********************************************************
                  EEPROM EXTERNA I2C
*********************************************************/

boolean init_EEPROMI2C() {

  Wire.beginTransmission(EEPROM_I2C_ADDRESS);
  if (Wire.endTransmission() != 0) {
    return 1;
  }
  return 0;
}


// Almacenar los siguientes datos:
void EEPROM_I2C_Almacena_datos() {

  byte paquete[30];  // No se pueden guardar paquetes superiores a 30 bytes
  uint16_t aux = FLIGHT_TIME;
  uint16_to_2byte(aux, &(paquete[0]));
  float_to_4byte(&Z_out, &(paquete[2]));
  float_to_4byte(&X_out, &(paquete[6]));
  float_to_4byte(&Y_out, &(paquete[10]));
  float_to_4byte(&Presion_BMP, &(paquete[14]));
  float_to_4byte(&Altitud_BMP, &(paquete[18]));
  float_to_4byte(&GPS_LAT, &(paquete[22]));
  float_to_4byte(&GPS_LON, &(paquete[26]));

  writeEEPROM_Page(eeprom_mem, paquete, 30);
  eeprom_mem += 30;
}



// Almacenar máximo 30 bytes seguidos
void writeEEPROM_Page(uint16_t address, byte *val, byte tam) {
  Wire.beginTransmission(EEPROM_I2C_ADDRESS);
  Wire.write((uint8_t)(address >> 8));   // MSB
  Wire.write((uint8_t)(address & 0xFF)); // LSB
  Wire.write(val,tam);
  Wire.endTransmission();
  delay(10);
}


// Guardar el float en &aux, &aux+1, &aux+2, &aux+3
void float_to_4byte(float* var, byte* aux) {
  byte* p = (byte*)var;
  for (char i = 3; i >= 0; i--) {
    *(aux + i) = *p;
    p++;
  }
}


// Conversión de los bytes a float
void _4byte_to_float(byte* aux, float *out) {
  uint32_t mem_aux = 0;
  mem_aux |= aux[3];
  mem_aux |= (uint32_t)(aux[2]) << 8;
  mem_aux |= (uint32_t)(aux[1]) << 16;
  mem_aux |= (uint32_t)(aux[0]) << 24;
  *(out) = *((float*)&mem_aux);
}


// Guardar el uint16_t MSB, LSB
void uint16_to_2byte(uint16_t dato_in, byte* dir_dato_out) {
  *(dir_dato_out) = (byte)(dato_in >> 8);
  *(dir_dato_out + 1) = (byte)dato_in;
}


void EEPROM_I2C_Lectura_datos() {
  eeprom_mem = 0;
  byte paquete[30];
  float aux[7];
  uint16_t tim;
  for (int i = 0; i < 10000; i++) {

    // Leer paquetes individuales
    for (int j = 0; j < 30; j++) {
      paquete[j] = readEEPROM(j + eeprom_mem);
      //Serial.print(paquete[j], HEX);
    }
    eeprom_mem += 30;

    tim = paquete[1];
    tim += ((uint16_t)paquete[0]) << 8;
    Serial.print(tim);
    Serial.write('\t');
    
    for (int k = 0; k < 7; k++) {
      _4byte_to_float(&(paquete[k * 4 + 2]), &(aux[k]));
      Serial.print(aux[k], 5);
      Serial.write('\t');
    }

    Serial.write('\n');
  }

  eeprom_mem = 0;
}


// Función para leer de la EEPROM
byte readEEPROM(uint32_t address) {
  byte rcvData = 0xFF;
  Wire.beginTransmission(EEPROM_I2C_ADDRESS);
  Wire.write((uint32_t)(address >> 8));   // MSB
  Wire.write((uint32_t)(address & 0x00FF)); // LSB
  Wire.endTransmission();
  Wire.requestFrom(EEPROM_I2C_ADDRESS, 1);
  rcvData =  Wire.read();
  return rcvData;
  delay(5);
}


// Función para escribir en la EEPROM
void writeEEPROM(uint16_t address, byte val) {
  Wire.beginTransmission(EEPROM_I2C_ADDRESS);
  Wire.write((uint8_t)(address >> 8));   // MSB
  Wire.write((uint8_t)(address & 0xFF)); // LSB
  Wire.write(val);
  Wire.endTransmission();
  delay(5);
}



/********************************************************
                     EEPROM  INTERNA
*********************************************************/

int mem = 0;
/*
  void EEPROM_Almacena_datos() {
  uint32_t tiempo;
  if (mem <= 1007) {
    tiempo = (millis() - t_inicio);
    if (tiempo > t_guardar) {
      t_guardar += T_ALMACENAMIENTO;
      EEPROM_tiempo(tiempo);
      EEPROM_float(&Presion_BMP);
      EEPROM_float(&X_out);
      EEPROM_float(&altitud);
      //EEPROM_string_a_float();
    }
  }

  }

  void EEPROM_Almacena_Apertura() {
  EEPROM_tiempo((millis() - t_inicio));
  float aux = 0.0;
  EEPROM_float(&aux);
  EEPROM_float(&aux);
  EEPROM_float(&aux);
  mem += 14;
  }


  void EEPROM_tiempo(uint32_t tiempo) {
  uint8_t aux = (uint8_t)((tiempo & 0x0000FF00) >> 8);
  EEPROM.write(mem, aux);
  mem++;
  aux = (tiempo & 0x000000FF);
  EEPROM.write(mem, aux);
  mem++;
  }


  void EEPROM_float(float* dir_dato) {
  byte* puntero_dato = (byte*)dir_dato;
  byte aux;
  for (byte i = 4; i > 0; i--) {
    aux = *(puntero_dato);
    EEPROM.write(mem, aux);
    mem++;
  }
  }

  void EEPROM_string_a_float(String str) {
  float flo = str.toFloat();
  EEPROM_float(&(flo));
  }

*/

/********************************************************
                          GPS
*********************************************************/

boolean gps_init_NEO6M() {

  // Configuración GPS NEO6M
  ss.begin(9600);
  ss.println("$PUBX,40,GLL,0,0,0,0*5C");
  ss.println("$PUBX,40,ZDA,0,0,0,0*44");
  ss.println("$PUBX,40,VTG,0,0,0,0*5E");
  ss.println("$PUBX,40,GSV,0,0,0,0*59");
  ss.println("$PUBX,40,GSA,0,0,0,0*4E");
  ss.println("$PUBX,40,RMC,0,0,0,0*47");

  /*
  const byte Hz5[] = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A
  };

  for (int i = 0; i < sizeof(Hz5); i++) {
    ss.write(Hz5[i]);
  }
  */

  // Verificar inicializacion
  delay(1000);
  if (!ss.available()) {
    return 0;
  }
  return 1;
}



void gps_wait_signal() {
  while (abs(GPS_LAT) > 90.0 || abs(GPS_LON) > 90.0) {
    gps_read();
    //delay(100);
  }
}



void gps_read() {
  TinyGPS gps;
  char var = -1;

  while (ss.available()) {
    var = ss.read();
    gps.encode(var);
  }

  if (var != -1) {
    int auxi = 0;
    byte auxb = 0;
    gps.f_get_position(&GPS_LAT, &GPS_LON);
    gps.crack_datetime(&auxi, &auxb, &auxb, &GPS_HOU, &GPS_MIN, &GPS_SEC);
    GPS_ALT = gps.f_altitude();
    GPS_VEL = gps.f_speed_kmph();
    GPS_SAT = gps.satellites();
  }
}



/*
  static void smartdelay(unsigned long ms)
  {
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps->encode(ss.read());
  } while (millis() - start < ms);
  }
*/



/********************************************************
                 ACELEROMETRO KX134
*********************************************************/


void KX134_64g_read_acc() {

  Wire.beginTransmission(KX134);
  Wire.write(0x08);
  Wire.endTransmission(false);
  Wire.requestFrom(KX134, 6, true);

  //const float convRange2G =  0.00006103518784142582;
  //const float convRange4G =  0.0001220703756828516;
  //const float convRange8G =  0.0002441407513657033;
  const float convRange64G = .001953125095370342112;
  X_out = ((float)( Wire.read() | Wire.read() << 8)) * convRange64G - OFFSET_X;
  Y_out = ((float)( Wire.read() | Wire.read() << 8)) * convRange64G - OFFSET_Y;
  Z_out = ((float)( Wire.read() | Wire.read() << 8)) *  convRange64G - OFFSET_Z;
}


boolean KX134_64g_init() {

  Wire.beginTransmission(KX134);
  if (Wire.endTransmission() == 2) {
    return 0;
  }

  // OPCIONES PARA PONER EL RANGO DE LECTURA:
  /*
    #define KX134_RANGE8G  0x00
    #define KX134_RANGE16G 0x01
    #define KX134_RANGE32G 0x02
    #define KX134_RANGE64G 0x03
  */
  byte tempRegVal;
  tempRegVal = B11000000;       // Inicialización acelerómetro
  tempRegVal |= (0x03 << 3);    // Cambiar aquí el rango de lectura
  Wire.beginTransmission(KX134);
  Wire.write(0x1B);
  Wire.write(tempRegVal);
  Wire.endTransmission();
  // Salida por defecto a 50Hz

  // Desactivación del filtro paso bajo
  Wire.beginTransmission(KX134);
  Wire.write(0x21);
  Wire.write(B10000110);
  Wire.endTransmission();
  return 1;
}







/********************************************************
                 PARACAIDAS Y ZUMBADOR
*********************************************************/



void paracaidas_open() {
  // Puente de Mosfet
  digitalWrite(PIN_ELECTROIMAN, 1);
}

void paracaidas_close() {
  // Puente de Mosfet
  // Estado en reposo aun con arduino apagado
  digitalWrite(PIN_ELECTROIMAN, 0);
}

void zumbador_on() {
  // Configuracion BJT + MOSFET
  digitalWrite(PIN_ZUMBADOR, 0);
}

void zumbador_off() {
  // Configuracion BJT + MOSFET
  digitalWrite(PIN_ZUMBADOR, 1);
}




/********************************************************
                         FLAGS
*********************************************************/

void error_inicio() {
  zumbador_on();
  while (true) {
    digitalWrite(PIN_LED_ERROR, HIGH);
    delay(200);
    digitalWrite(PIN_LED_ERROR, LOW);
    delay(200);
  }
}

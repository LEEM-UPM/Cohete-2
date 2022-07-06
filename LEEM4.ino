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
// LEEMUR 4 JUL/22
/*
   Módulo GPS LOCOSYS-1612-G / ¿Módulo GPS NEO 6M?
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
#define ACC_START             2.0          // g
#define T_MIN_PARACAIDAS      4000         // ms
#define T_MAX_PARACAIDAS      13000        // ms
#define DIF_ALTURA_APERTURA   20.0         // m
#define DIF_ALTURA_ALARMA     200.0        // m
#define T_MAX_ALARMA          30000        // ms
#define T_ESPERA_EM           60000       // ms  (Tiempo de espera de electroimanes)
// Apogeo estimado:
// Tiempo estimado:


//Control apertura
#define FLIGHT_TIME (millis() - t_inicio)
float alt_max = 0.0;
uint32_t t_inicio = 0;
boolean start = false;


//-------------------------------------------------
//             Declaración de pines
//-------------------------------------------------
#define PIN_MICRO_SD_CS   10
#define PIN_LED_ERROR     9
#define PIN_LED_READY     2
#define PIN_ZUMBADOR      16  // (A2)
#define PIN_GPS_TX        3
#define PIN_GPS_RX        4
#define PIN_ELECTROIMAN_A 15  // (A1)
#define PIN_ELECTROIMAN_B 8
#define PIN_PULSADOR      6
#define PIN_HALL          7
#define PIN_LM35          A0



//-------------------------------------------------
//              MODULOS Y SENSORES
//-------------------------------------------------

// Cosas acelerómetro
#define KX134    0x1F // 0x1E
#define OFFSET_X 0.00
#define OFFSET_Y 0.00
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


// Presion
Adafruit_BMP280 bmp;


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
//float HALL;





/********************************************************
                 PARACAIDAS Y ZUMBADOR
*********************************************************/

void paracaidas_off() {
  // Puente de mosfet
  digitalWrite(PIN_ELECTROIMAN_A, 0);
  digitalWrite(PIN_ELECTROIMAN_B, 0);
}

void paracaidas_open() {
  // Puente de mosfet
  digitalWrite(PIN_ELECTROIMAN_A, 1);
  digitalWrite(PIN_ELECTROIMAN_B, 0);
}

void paracaidas_close() {
  // Puente de mosfet
  digitalWrite(PIN_ELECTROIMAN_A, 0);
  digitalWrite(PIN_ELECTROIMAN_B, 1);
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

class Avisos {
  public:
    static uint32_t var;

    // 1. Inicializacion modulos y GPS
    static void espera_modulos() {
      zumbador_off();
      digitalWrite(PIN_LED_ERROR, HIGH);
      digitalWrite(PIN_LED_READY, LOW);
      //var = millis();
    }

    // 2. Esperar al armado de EM
    static void espera_em() {
      digitalWrite(PIN_LED_READY, 0);
      if ((millis() - Avisos::var) >= 100) {
        zumbador_off();
        digitalWrite(PIN_LED_ERROR, LOW);
      }
      if ((millis() - Avisos::var) >= 750) {
        zumbador_on();
        digitalWrite(PIN_LED_ERROR, HIGH);
        Avisos::var = millis();
      }
    }

    // Boton presionado:
    static void boton_presionado() {
      digitalWrite(PIN_LED_ERROR, LOW);
      for (int i = 0; i < 3; i++) {
        zumbador_on();
        delay(100);
        zumbador_off();
        delay(200);
      }
    }

    // 3. Tiempo de espera de
    static void espera_tiempo() {
      digitalWrite(PIN_LED_ERROR, 0);
      if ((millis() - Avisos::var) >= 100) {
        zumbador_off();
        digitalWrite(PIN_LED_READY, LOW);
      }
      if ((millis() - Avisos::var) >= 1000) {
        zumbador_on();
        digitalWrite(PIN_LED_READY, HIGH);
        Avisos::var = millis();
      }

    }

    // 4. Listos para el lanzamiento
    static void espera_lanzamiento() {
      digitalWrite(PIN_LED_READY, 1);
      if ((millis() - Avisos::var) >= 500) {
        zumbador_on();
        digitalWrite(PIN_LED_ERROR, HIGH);
      }
      if ((millis() - Avisos::var) >= 1000) {
        zumbador_off();
        Avisos::var = millis();
        digitalWrite(PIN_LED_ERROR, LOW);
      }
    }

    // :-( Error en algún modulo
    static void error_inicio() {
      zumbador_on();
      while (true) {
        digitalWrite(PIN_LED_ERROR, HIGH);
        delay(200);
        digitalWrite(PIN_LED_ERROR, LOW);
        delay(200);
        if (digitalRead(PIN_PULSADOR) == LOW) {
          zumbador_off();
        }
      }
    }
};
uint32_t Avisos::var = 0;




//-------------------------------------------------
//                     SETUP
//-------------------------------------------------

void setup() {


  // 0. DECLARACIONES
#if DEBUG == 1
  Serial.begin(115200);
#endif
  pinMode(PIN_LED_ERROR, OUTPUT);
  pinMode(PIN_LED_READY, OUTPUT);
  pinMode(PIN_ZUMBADOR, OUTPUT);
  pinMode(PIN_ELECTROIMAN_A, OUTPUT);
  pinMode(PIN_ELECTROIMAN_B, OUTPUT);
  pinMode(PIN_PULSADOR, INPUT_PULLUP);
  pinMode(PIN_HALL, INPUT);



  // 1. INICIALIZACION Y TEST DE FUNCIONAMIENTO
  paracaidas_off();
  Avisos::espera_modulos();

  // Wire.begin para todos los modulos
  Wire.begin();
  delay(50);

  /*
    // Lectura de datos I2C
    digitalWrite(PIN_LED_READY, 1);
    EEPROM_I2C_Lectura_datos();
    while (true) {
      delay(1);
    }
  */

  if (!KX134_64g_init()) {
#if DEBUG == 1
    Serial.println("Error KX134");
#endif
    Avisos::error_inicio();
  }


  if (!bmp.begin()) {
#if DEBUG == 1
    Serial.println("Error BMP280");
#endif
    Avisos::error_inicio();
  }


  pinMode(SSpin, OUTPUT);
  if (!SD.begin(SSpin)) {
#if DEBUG == 1
    Serial.println("Error SD");
#endif
    Avisos::error_inicio();
  }


  archivo = &(SD.open("datos.txt", FILE_WRITE));
  if (archivo == NULL) {
#if DEBUG == 1
    Serial.println("Error Archivo SD");
#endif
    Avisos::error_inicio();
  }


  if (!init_EEPROMI2C() == NULL) {
#if DEBUG == 1
    Serial.println("Error EEPROM I2C");
#endif
    Avisos::error_inicio();
  }


/*
  if (!gps_init_NEO6M()) {
#if DEBUG == 1
    Serial.println("Error GPS");
#endif
    Avisos::error_inicio();
  }
*/


  // 2.1 ESPERA A RECIBIR SEÑAL GPS VALIDA
#if DEBUG == 1
  Serial.println("Start OK, waiting for GPS valid signal");
#endif
  //gps_wait_signal(1000); // Experimental


  // 2.2 ESPERA AL ARMADO DE ELECROIMANES:
#if DEBUG == 1
  Serial.println("GPS correcto, pulsa el boton para la cuenta atras");
#endif
  while (digitalRead(PIN_PULSADOR) == HIGH) {
    Avisos::espera_em();
  }


  // 2.3 TIEMPO DE ARMADO DE ELECTROIMANES
  Avisos::boton_presionado();
  uint32_t t_o;
  t_o = millis();
  while ( millis() < (t_o + T_ESPERA_EM) ) {
    Avisos::espera_tiempo();
  }
  paracaidas_open();



  // 5. DETECTAR ACELRERACIÓN MÁXIMA E INICIAR TOMA DE DATOS
  digitalWrite(PIN_LED_READY, 1);
  t_inicio = millis();
  while (true) {

    // Toma de datos
    Toma_de_datos();   //(EEPROM_I2C no)

    // Avisador acustico
    Avisos::espera_lanzamiento();

    // Aceleración despegue detectada
    KX134_64g_read_acc();
    if (abs(Z_out) > ACC_START) {
      zumbador_off();
      break;
    }
  }

  zumbador_off();
  digitalWrite(PIN_LED_READY, 1);
  digitalWrite(PIN_LED_ERROR, 1);
  t_inicio = millis();
  start = true;
}


void loop()
{

  // Toma de datos SD e EEPROM_I2C
  Toma_de_datos();


  // CONTROL DEL PARACAIDAS
  if (Altitud_BMP > alt_max && (FLIGHT_TIME > T_MIN_PARACAIDAS)) {
    alt_max = Altitud_BMP;
  }

  if ( ((Altitud_BMP < (alt_max - DIF_ALTURA_APERTURA)) && (FLIGHT_TIME > T_MIN_PARACAIDAS))  ||  FLIGHT_TIME > T_MAX_PARACAIDAS)  {
    paracaidas_open();
    digitalWrite(PIN_LED_ERROR, 0);
    SD_paracaidas();
  }


  // CONTROL DE ACTIVACION DE ALARMA
  if ( (alt_max > (Altitud_BMP + DIF_ALTURA_ALARMA)) || FLIGHT_TIME > T_MAX_ALARMA)  {
    zumbador_on();
  }

}




/********************************************************
                     Toma de datos
*********************************************************/

void Toma_de_datos() {

  // Lectura de datos:
  T_BMP = bmp.readTemperature();
  Presion_BMP = bmp.readPressure();
  Altitud_BMP = bmp.readAltitude();
  KX134_64g_read_acc();
  gps_read();

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
  Serial.print(!digitalRead(PIN_HALL));
  Serial.write('\n');
#endif


  // EEPROM I2C
  if (start) {
    EEPROM_I2C_Almacena_datos();
  }

  // Tarjeta SD
  SD_Almacena_datos();


  // EEPROM INTERNA (si procede cada T_ALMACENAMIENTO)
  // EEPROM_Almacena_datos();

}




/********************************************************
                       Escritura SD
*********************************************************/


void SD_Almacena_datos() {
  float HALL;
  HALL = !digitalRead(PIN_HALL);

  // Escritura SD
  archivo->write(0x45);
  archivo->write(0x45);     // 'EE'
  if (start) {
    archivo->write((byte)(FLIGHT_TIME & 0x000000FF));
    archivo->write((byte)((FLIGHT_TIME & 0x0000FF00) >> 8));
    archivo->write((byte)((FLIGHT_TIME & 0x00FF0000) >> 16));
    archivo->write((byte)((FLIGHT_TIME & 0xFF000000) >> 24));
  }
  else {
    uint32_t t;
    t = millis();
    archivo->write((byte*)(&t), 4);
  }
  archivo->write((byte*)(&X_out), 4);
  archivo->write((byte*)(&Y_out), 4);
  archivo->write((byte*)(&Z_out), 4);
  archivo->write((byte*)(&T_EXT), 4);
  archivo->write((byte*)(&Presion_BMP), 4);
  archivo->write((byte*)(&Altitud_BMP), 4);
  archivo->write((byte*)(&T_BMP), 4);
  archivo->write((byte*)(&(HALL)), 4);
  archivo->write((byte*)(&GPS_ALT), 4);
  archivo->write((byte*)(&GPS_LAT), 4);
  archivo->write((byte*)(&GPS_LON), 4);
  archivo->write((byte*)(&GPS_VEL), 4);
  archivo->write(GPS_HOU);
  archivo->write(GPS_MIN);
  archivo->write(GPS_SEC);
  archivo->flush();
}


void SD_paracaidas() {
  archivo->write(0x41);
  archivo->write(0x41);     // 'AA'
  archivo->write((byte)(FLIGHT_TIME & 0x000000FF));
  archivo->write((byte)((FLIGHT_TIME & 0x0000FF00) >> 8));
  archivo->write((byte)((FLIGHT_TIME & 0x00FF0000) >> 16));
  archivo->write((byte)((FLIGHT_TIME & 0xFF000000) >> 24));
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
  if (eeprom_mem < (65536 - 35)) {
    byte paquete[30];  // No se pueden guardar paquetes superiores a 30 bytes, se llena el buffer I2C
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
}



// Almacenar máximo 30 bytes seguidos
void writeEEPROM_Page(uint16_t address, byte *val, byte tam) {
  Wire.beginTransmission(EEPROM_I2C_ADDRESS);
  Wire.write((uint8_t)(address >> 8));   // MSB
  Wire.write((uint8_t)(address & 0xFF)); // LSB
  Wire.write(val, tam);
  Wire.endTransmission();
  delay(10); // Cuestionable !!!
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

    if (eeprom_mem >= (65536 - 30)) {
      Serial.println("Fin de la lectura de datos EEPROM_I2C");
      while (true);
    }

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
/*

  int mem = 0;
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
  start_com();
  ss.println("GLL,0,0,0,0*5C");
  start_com();
  ss.println("ZDA,0,0,0,0*44");
  start_com();
  ss.println("VTG,0,0,0,0*5E");
  start_com();
  ss.println("GSV,0,0,0,0*59");
  start_com();
  ss.println("GSA,0,0,0,0*4E");
  start_com();
  ss.println("RMC,0,0,0,0*47");
  ss.println("\xB5\x62\x06\x08\x06\x00\xC8\x00\x01\x00\x01\x00\xDE\x6A");
  return 1;
}

void start_com(){
  ss.print("$PUBX,40,");
}

/*
boolean gps_init_LOCOSYS_1612G() {

  // Configuración GPS LOCOSYS_1612_G
  ss.begin(9600);
  ss.println("$PMTK251,38400*27");   // 38400bps
  ss.end();
  ss.begin(38400);
  ss.println("$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"); // Solo GGA
  ss.println("$PMTK220,100*2F");     // 10Hz de lectura
  ss.println("$PMTK251,38400*27");   // 38400bps

  return 1;
}
*/

void gps_wait_signal() {
  while (abs(GPS_LAT) > 90.0 || abs(GPS_LON) > 90.0) {
    gps_read();
    //delay(100);
  }

#if DEBUG == 1
  Serial.println("GPS signal OK");
#endif
}


// Funcion sobrecargada para asegurarse que la senal GPS se mantiene por "tiempo"
void gps_wait_signal(int tiempo) {
  boolean sign_ok = false;
  uint32_t start;
  start = millis();
  while(true){
    sign_ok = (abs(GPS_LAT) < 90.0 && abs(GPS_LON) < 90.0);
    if(sign_ok && (millis() > (tiempo + start)) ){
      break;
    }
    if(!sign_ok){
      start = millis();
    }
  }

#if DEBUG == 1
  Serial.println("GPS signal OK");
#endif
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

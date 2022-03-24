#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <SD.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
//#include <Servo.h>
//#include <EEPROM.h>


// Código hecho por Andrés (código en proceso)


// Lista de dispositivos del Cohete 2 7/ABR/22
/*
   Módulo GPS NEO 6M
   Sensor BMP280
   Tarjeta SD
   EEPROM 24FC512
   Sensor Hall

   Apertura paracaidas electroimanes
   Zumbador alarma
*/



//-------------------------------------------------
//              Parámetros Cobete
//-------------------------------------------------
#define ACC_START             5.0     // g
#define T_MIN_PARACAIDAS      4000    // ms
#define T_MAX_PARACAIDAS      13000   // ms





//-------------------------------------------------
//             Declaración de pines
//-------------------------------------------------
#define PIN_MICRO_SD_CS   10
#define PIN_LED_ERROR     9
#define PIN_LED_READY     2
#define PIN_ZUMBADOR      15  // (A1)
#define PIN_GPS_TX        3
#define PIN_GPS_RX        4
#define PIN_ELECTROIMAN   14  // (A0)
#define PIN_PULSADOR      6
#define PIN_HALL          7





// Cosas acelerómetro



// Cosas SD
#define SSpin 10
File *archivo;


// GPS
TinyGPS *gps;
SoftwareSerial ss(PIN_GPS_TX, PIN_GPS_RX);
static void smartdelay(unsigned long ms);
#define GPS_SAT gps->satellites.value()
#define GPS_LAT gps->location.lat()
#define GPS_LON gps->location.lng()
#define GPS_ALT gps->altitude.meters()
#define GPS_SEC gps->time.second()
#define GPS_MIN gps->time.minute()
#define GPS_HOU gps->time.hour() + 1


// Presion
Adafruit_BMP280 bmp;


// Variables
float T_K;
float altitud;
float velocidad;
float T_C;
float Altitud_BMP;
float Presion;


//Control apertura
bool con_acc = false;
bool con_pres = false;
int flight_time;
float pres_min = 99999999999;
unsigned long t_inicio = 0;



// Guardar datos en EEPROM INTERNA
/*
#define T_ALMACENAMIENTO 200
uint16_t mem = 0;
unsigned long t_guardar = 0;
*/


// Prototipos de funciones:
void paracaidas_open();
void paracaidas_close();
void zumbador_on();
void zumbador_off();



void error_inicio() {
  while (true) {
    digitalWrite(PIN_LED_ERROR, HIGH);
    delay(200);
    digitalWrite(PIN_LED_ERROR, LOW);
    delay(200);
  }
}


void setup() {


  // 0. DECLARACIONES

  Serial.begin(115200);
  delay(2000);
  pinMode(PIN_LED_ERROR, OUTPUT);
  pinMode(PIN_LED_READY, OUTPUT);
  pinMode(PIN_ZUMBADOR, OUTPUT);
  pinMode(PIN_ELECTROIMAN, OUTPUT);
  pinMode(PIN_PULSADOR, INPUT_PULLUP);
  // PIN_HALL  ??
  

  
  // 1. INICIALIZACION Y TEST DE FUNCIONAMIENTO
  
  paracaidas_close();
  zumbador_off();
  
  
  Wire.begin();
  if (!bmp.begin()) {
    Serial.println(F("No se puede encontral el sensor BMP280, revisar el cableado "));
    error_inicio();
  }
    
  //Serial.println("Inicializando tarjeta ...");
  if (!SD.begin(SSpin)) {
    error_inicio();
  }
  archivo = &(SD.open("datos.txt", FILE_WRITE));
  
  
  // GPS
  ss.begin(9600);

  while (!archivo) {
    delay(50);
  }

  
  
  // 2. ESPERA A RECIBIR SEÑAL GPS VALIDA
  
  
  
 
  /*
    Serial.println("inicializacion correcta");
    Serial.println("--------- Datos del testeo del paracaídas ---------");
    Serial.println("  ");
    Serial.println("----------------------------------------------------");
    Serial.println("  ");
    Serial.println("P_MPRLS P_MPRLS  A_X A_Y A_Z   Alt    Speed ");
    Serial.println("   hPa     Pa         g         m      km/h  ");
    Serial.println("-------------------------------------------------------------------------------------------------------------------------------------");
  */

  //archivo->println("--------- Datos del testeo del paracaídas ---------\n");
  //archivo->println("P_MPRLS P_MPRLS  A_X A_Y A_Z   Alt  Speed Card ");
  //archivo->println("   hPa     Pa         g         m      km/h\n");



  // 3. ESPERA A ACELERACIÓN LANZAMIENTO
  X_out = 0.0;
  while (true) {
    lectura_MPU();
    if (abs(X_out) > ACC_START) {
      break;
    }
  }
  t_inicio = millis();



}


void loop()
{
  // Datos del BMP280

  //Serial.print(bmp.readTemperature());
  //Serial.print(",");
  //Serial.print(T_K);
  //Serial.print(",");
  //Serial.print(bmp.readPressure());
  //Serial.print(",");
  //Serial.print(bmp.readAltitude(1013.25)); /* HAY QUE AJUSTAR LA PRESIÓN DE REFERENCIA */
  //Serial.print(",");

  // Datos del acelerómetro KX13X
  /*
    Acelerometro = kxAccel.getAccelData();
    Serial.print(Acelerometro.xData, 4);
    Serial.print(",");
    Serial.print(Acelerometro.yData, 4);
    Serial.print(",");
    Serial.print(Acelerometro.zData, 4);
    Serial.print(",");
  */

  // MPU LOOP
  lectura_MPU();

  altitud = gps->f_altitude();
  velocidad = gps->f_speed_kmph();

  T_C = bmp.readTemperature();
  Presion = bmp.readPressure();
  Altitud_BMP = bmp.readAltitude(1013.25);

  //Serial.print(altitud);
  //Serial.print(",");
  //Serial.println(velocidad);



  // Escritura SD
  archivo->write((byte*)(&T_C), 4);
  archivo->write(44);
  archivo->write((byte*)(&T_K), 4);
  archivo->write(44);
  archivo->write((byte*)(&Presion), 4);
  archivo->write(44);
  archivo->write((byte*)(&Altitud_BMP), 4);
  archivo->write(44);
  archivo->write((byte*)(&X_out), 4);
  archivo->write(44);
  archivo->write((byte*)(&Y_out), 4);
  archivo->write(44);
  archivo->write((byte*)(&Z_out), 4);
  archivo->write(44);
  archivo->write((byte*)(&altitud), 4);
  archivo->write(44);
  archivo->write((byte*)(&velocidad), 4);
  archivo->write(59); // 'EE'
  archivo->write(10);
  archivo->flush();



  // EEPROM INTERNA (si procede cada T_ALMACENAMIENTO)
  //EEPROM_Almacena_datos();



  // CONTROL DEL PARACAIDAS
  if (Presion < pres_min)
  {
    pres_min = Presion;
  }

  if (!con_acc && abs(X_out) > 5)
  {
    con_acc = true;
    flight_time = millis();
  }
  if (!con_pres && con_acc && (pres_min < 0.9 * Presion || (millis - flight_time) > 10000))
  {
    con_pres = true;
    servoMotor.write(110);
  }

  delay(10);

}



void lectura_MPU() {
  Wire.beginTransmission(ADXL345);
  Wire.write(0x32); // Start with register 0x32 (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  X_out = ( Wire.read() | Wire.read() << 8); // X-axis value
  X_out = X_out / 30.72; //For a range of +-2g, we need to divide the raw values by 256, according to the datasheet
  Y_out = ( Wire.read() | Wire.read() << 8); // Y-axis value
  Y_out = Y_out / 30.72;
  Z_out = ( Wire.read() | Wire.read() << 8); // Z-axis value
  Z_out = Z_out / 30.72;
  Serial.print("\t\t");
  Serial.println(X_out);
}



void writeTo(byte address, byte val) {
  writeToI2C(address, val);
}

void writeToI2C(byte _address, byte _val) {
  Wire.beginTransmission(ADXL345_DEVICE);
  Wire.write(_address);
  Wire.write(_val);
  Wire.endTransmission();
}


void readFrom(byte address, int num, byte _buff[]) {
  readFromI2C(address, num, _buff); // If I2C Communication
}

void readFromI2C(byte address, int num, byte _buff[]) {
  Wire.beginTransmission(ADXL345_DEVICE);
  Wire.write(address);
  Wire.endTransmission();

  //  Wire.beginTransmission(ADXL345_DEVICE);
  // Wire.reqeustFrom contains the beginTransmission and endTransmission in it.
  Wire.requestFrom(ADXL345_DEVICE, num);  // Request 6 Bytes

  int i = 0;
  while (Wire.available())
  {
    _buff[i] = Wire.read();       // Receive Byte
    i++;
  }
  if (i != num) {
    status = ADXL345_ERROR;
    error_code = ADXL345_READ_ERROR;
  }
  //  Wire.endTransmission();
}





/********************************************************
                       Lectura SD
*********************************************************/






/********************************************************
                     EEPROM  INTERNA
*********************************************************/

void EEPROM_Almacena_datos() {
  uint32_t tiempo;
  if (mem <= 1007) {
    tiempo = (millis() - t_inicio);
    if (tiempo > t_guardar) {
      t_guardar += T_ALMACENAMIENTO;
      EEPROM_tiempo(tiempo);
      EEPROM_float(&Presion);
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



/********************************************************
                          GPS
*********************************************************/


void gps_init() {

  // Configuración GPS NEO-6M
  ss.print("$PUBX,40,GLL,0,0,0,0*5C\r\n");
  ss.print("$PUBX,40,ZDA,0,0,0,0*44\r\n");
  ss.print("$PUBX,40,VTG,0,0,0,0*5E\r\n");
  ss.print("$PUBX,40,GSV,0,0,0,0*59\r\n");
  ss.print("$PUBX,40,GSA,0,0,0,0*4E\r\n");
  ss.print("$PUBX,40,RMC,0,0,0,0*47\r\n");
}

static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps->encode(ss.read());
  } while (millis() - start < ms);
}







/********************************************************
                 PARACAIDAS Y ZUMBADOR
*********************************************************/


void paracaidas_open(){
  // Puente de Mosfet
  digitalWrite(PIN_ELECTROIMAN, 1);
}

void paracaidas_close(){
  // Puente de Mosfet
  // Estado en reposo aun con arduino apagado
  digitalWrite(PIN_ELECTROIMAN, 0);
}

void zumbador_on(){
  // Configuracion BJT + MOSFET
  digitalWrite(PIN_ZUMBADOR, 0);
}

void zumbador_off(){
  // Configuracion BJT + MOSFET
  digitalWrite(PIN_ZUMBADOR, 1);
}



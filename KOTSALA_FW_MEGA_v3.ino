#include <ArduinoJson.h>
// vercion 2

String estados = "";
String par1 = "";
String par2 = "";
String par3 = "";
String par4 = "";
String par5 = "";
String par6 = "";
String par7 = "";
String par8 = "";

#define ENA 2
#define ENB 7
#define IN1 3
#define IN2 4
#define IN3 5
#define IN4 6

#define carSpeed 120

void M1adelante() {
  analogWrite(ENA, carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

void M1atras() {
  analogWrite(ENA, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void M2adelante() {  // abrir
  analogWrite(ENB, carSpeed);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}
void M2atras() {  // cerrar
  analogWrite(ENB, carSpeed);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void M1paro() {
  digitalWrite(ENA, LOW);
}
void M2paro() {
  digitalWrite(ENB, LOW);
}
/*int Pulsador1 = 0;
int Pulsador2 = 0;*/
int pulsador3 = 0;
int pulsador4 = 0;
//int final1 = 27;
//int final2 = 29;
int final3 = 31;
int final4 = 33;
bool bFuncionamiento = false;


// Definimos los pines a los que están conectados los relés
const int rele1Pin = 13;
const int rele2Pin = 12;
const int rele3Pin = 11;
const int rele4Pin = 10;

const int pinSensorTierra1 = A0;
const int pinSensorTierra2 = A1;

int Estado_0, Estado_1, Estado_2, Estado_3, Estado_4, Estado_5, Estado_6, Estado_7;
int ResEstado0, ResEstado1, ResEstado2, ResEstado3, ResEstado4, ResEstado5, ResEstado6, ResEstado7;

int Par1 = 0;
int Par2 = 0;
int Par3 = 0;
int Par4 = 0;
int Par5 = 0;
int Par6 = 0;
int Par7 = 0;
int Par8 = 0;
int humedadTierraprom = 0;
int riegoActivo = 0;  // Variable para rastrear si el riego está activo
int estadocortinas = 0;

//Paso 1
#include "DHT.h"

//Paso 2
#define DHTPIN 23
#define DHTTYPE DHT21  // DHT21 (AM2301)

//Paso 3
DHT dht(DHTPIN, DHTTYPE);

String Estados_Fecha_1 = "";
String Estados_Fecha_a1 = "";

#include "RTClib.h"
RTC_Millis rtc;
RTC_Millis tm;

void setup() {
  // Inicializamos comunicación serie
  Serial.begin(115200);

    // Inicializamos los pines de los relés como salidas
  pinMode(rele1Pin, OUTPUT);
  pinMode(rele2Pin, OUTPUT);
  pinMode(rele3Pin, OUTPUT);
  pinMode(rele4Pin, OUTPUT);
  
  // Apagamos todos los relés al inicio (si son relés activos en bajo, cambiar HIGH por LOW)
  digitalWrite(rele1Pin, HIGH);
  digitalWrite(rele2Pin, HIGH);
  digitalWrite(rele3Pin, HIGH);
  digitalWrite(rele4Pin, HIGH);

  //pinMode(final1, INPUT);
  //pinMode(final2, INPUT);
  attachInterrupt(digitalPinToInterrupt(21), Pulsador1, RISING);  //Pulsador 1 (27)
  attachInterrupt(digitalPinToInterrupt(20), Pulsador2, RISING);  //Pulsador 2 (29)


  pinMode(final3, INPUT);
  pinMode(final4, INPUT);

  Serial.println("DHTxx test!");
  dht.begin();


  rtc.begin(DateTime(F(__DATE__), F(__TIME__)));
}
float h;
float t;
float f;
int humedadTierra1;
int humedadTierra2;
void loop() {

  //pulsador1 = digitalRead(final1);
  //pulsador2 = digitalRead(final2);
  pulsador3 = digitalRead(final3);
  pulsador4 = digitalRead(final4);

  // Leer el valor analógico del pin del sensor
  int valorSensorTierra1 = analogRead(pinSensorTierra1);
  int valorSensorTierra2 = analogRead(pinSensorTierra2);

  // Convertir el valor analógico a un valor de humedad (entre 0 y 100)
  humedadTierra1 = map(valorSensorTierra1, 0, 1023, 0, 100);
  humedadTierra2 = map(valorSensorTierra2, 0, 1023, 0, 100);
  humedadTierraprom = humedadTierra1 + humedadTierra2 / 2;
  Par1 = par1.toInt();
  Par2 = par2.toInt();
  Par3 = par3.toInt();
  Par4 = par4.toInt();
  Par5 = par5.toInt();
  Par6 = par6.toInt();
  Par7 = par7.toInt();
  Par8 = par8.toInt();

  //Paso 5
  h = dht.readHumidity();

  //Paso 6
  t = dht.readTemperature();

  //Paso 7
  f = dht.readTemperature(true);

  //Paso 8
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  //Paso 9
  float hif = dht.computeHeatIndex(f, h);
  float hic = dht.computeHeatIndex(t, h, false);

  //Paso 10
  /* Serial.print("Humidity: ");
    Serial.print(h);
    Serial.print(" %\t");
    Serial.print("Temperature: ");
    Serial.print(t);
    Serial.print(" *C ");
    Serial.print(f);
    Serial.print(" *F\t");
    Serial.print("Heat index: ");
    Serial.print(hic);
    Serial.print(" *C ");
    Serial.print(hif);
    Serial.println(" *F");
*/
  //Evaluar si hace falta humedad y si se ejecutando el ciclo
  /*if (Par1 > humedadTierraprom ) {
    if (Estado_0 == 1) {
      if (!bFuncionamiento) {
        M1adelante();
        bFuncionamiento = true;
      }
      riegoActivo = true;
      ResEstado0 = 1;
    } else {
      riegoActivo = false;
    }
  }*/
  if (Par5 > humedadTierraprom ) {
    //if (Estado_0 == 1) {
      if (!bFuncionamiento) {
        M1adelante();
        bFuncionamiento = true;
      }
      riegoActivo = true;
      ResEstado0 = 1;
     } else {
     riegoActivo = false;
    //}
  }

   /* if (Par3 > humedadTierraprom ) {
        M1adelante();
      riegoActivo = 1;
    } else {
      riegoActivo = 0;
    }*/

  if(Par3 > h){
    CerrarVentanas();
  }
  if(Par3 < h){
     AbrirVentanas();
  }
  if(Par1 > t){
    // apagar ventiladores 
  apagarRelay(rele4Pin);
  }
  if(Par1 < t){
    // enceder ventiladores 
  encenderRelay(rele4Pin);
  }

  smartdelay(2000);
}
String Estados_s1 = "";
void serialEvent() {
  String receivedData = Serial.readStringUntil('\n');  // Lee la cadena hasta '\n'
                                                       //  Serial.println(receivedData);
  // Deserializa el JSON recibido
  StaticJsonDocument<1000> doc;
  DeserializationError error = deserializeJson(doc, receivedData);
  if (error) {
    return;
  }

  // Accede al valor del campo "Nombre"
  String nombre = doc["Nombre"];

  if (nombre == "Res_Cor_1") {
    // Accede a los campos específicos de Res_Cor_1
    estados = doc["Estados"].as<String>();
    par1 = doc["Par_1"].as<String>();
    par2 = doc["Par_2"].as<String>();
    par3 = doc["Par_3"].as<String>();
    par4 = doc["Par_4"].as<String>();
    par5 = doc["Par_5"].as<String>();
    par6 = doc["Par_6"].as<String>();
    par7 = doc["Par_7"].as<String>();
    par8 = doc["Par_8"].as<String>();
    // Realiza acciones con los datos de Res_Cor_1

    Estado_0 = estados.charAt(0) - '0';
    Estado_1 = estados.charAt(1) - '0';
    Estado_2 = estados.charAt(2) - '0';
    Estado_3 = estados.charAt(3) - '0';
    Estado_4 = estados.charAt(4) - '0';
    Estado_5 = estados.charAt(5) - '0';
    Estado_6 = estados.charAt(6) - '0';
    Estado_7 = estados.charAt(7) - '0';

    String Estados_1 = doc["Estados"];

    String Estados_Fecha_1 = doc["Estados_Fecha_1"];
    if (Estados_Fecha_1 != Estados_Fecha_a1) {
      Estados_Fecha_a1 = Estados_Fecha_1;

      if (Estados_s1 != Estados_1) {

        Estados_s1 = Estados_1;
        //Actualizar_estados();
      }
    }

    // ... y así sucesivamente para los otros campos de Res_Cor_1

    //////////////////////////////////////////////////////////////////////////ENVIO DE DATOS////////////////////////////////////////////////////////////////


    DateTime now = rtc.now();

    char Fecha_format[24];
    sprintf(Fecha_format, "%04d/%02d/%02d %02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());


    String Estados_m1 = (String)Estado_0 + (String)Estado_1 + (String)Estado_2 + (String)Estado_3 + (String)Estado_4 + (String)Estado_5 + (String)Estado_6 + (String)Estado_7;
    String Cliente_nombre = "Kotsala";
    String json_string_2;
    StaticJsonDocument<300> jsonDoc2;
    jsonDoc2["Nombre"] = "Cor_1";
    jsonDoc2["Sensor1"] = t;
    jsonDoc2["Sensor2"] = h;
    jsonDoc2["Sensor3"] = h;
    jsonDoc2["Sensor4"] = t;
    jsonDoc2["Sensor5"] = humedadTierra1;
    jsonDoc2["Sensor6"] = humedadTierra2;
    jsonDoc2["Sensor7"] = humedadTierraprom;
    jsonDoc2["Sensor8"] = riegoActivo;
    jsonDoc2["Estados_1"] = Estados_m1;

    jsonDoc2["Fecha"] = Fecha_format;
    serializeJson(jsonDoc2, json_string_2);
    Serial.println(json_string_2);
  }
}
static void smartdelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    serialEvent();
  pulsador3 = digitalRead(final3);
  pulsador4 = digitalRead(final4);
  } while (millis() - start < ms);
}

void AbrirVentanas() {
  M2adelante();
  if (pulsador3 == 1) {
    M2paro();
  }
}
void CerrarVentanas() {
  M2atras();
  if (pulsador4 == 1) {
    M2paro();
  }
}

void Pulsador1() {
  //&& Estado_0 == 1
  if (Par5 > humedadTierraprom ) {
    M1adelante();
  } else {
    bFuncionamiento = false;
    M1paro();
  }
}
void Pulsador2() {
  if(bFuncionamiento == 1){
  M1atras();
  }
}

void encenderRelay(int pin) {
  digitalWrite(pin, LOW); // Si es un relé activo en alto, cambiar LOW por HIGH
}

// Función para apagar un relé
void apagarRelay(int pin) {
  digitalWrite(pin, HIGH); // Si es un relé activo en alto, cambiar HIGH por LOW
}

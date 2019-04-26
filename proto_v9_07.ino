#include <OneWire.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DallasTemperature.h>

#include<dht.h>
dht DHT;

//SENSORES ANALOGICOS
const int CTRL_MUX = D0;    //CONTROLADOR DE MULTIPLEXOR
const int ANALOG = A0;      //ENTRADA ANALOGICA
const int OUT_MUX_S0 = D6;
const int OUT_MUX_S1 = D7;
const int OUT_MUX_S2 = D8;
int Mux1_State[8] = {0};          //ARREGLO DE LECTURAS DEL MULTIPLEXOR
float Mux1_State_Pctje[8] = {0};    //ARREGLO DE LECTURAS DEL MULTIPLEXOR EN PRCENTAJES O UNIDADES

//DS18B20 - SENSOR DE TEMPERATURA DIGITAL
#define ONE_WIRE_BUS_1 D1
#define ONE_WIRE_BUS_2 D4

//DHT11 - TEMPERATURA Y HUMEDAD - 1SEG DE MUESTREO
#define DHT11_PIN_1 D3
#define DHT11_PIN_2 D2
#define DHT11_DELAY 1000

//DATOS DE RED
const char* ssid = "API";
const char* password = "starcraft";
const char* mqtt_server = "192.168.100.1";

//MOSQUITTO
int puerto = 1883;
String cliente = "linea2";
char topic[] = "/ibero/02/";
int tamTopic = 12;

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire_1(ONE_WIRE_BUS_1);
OneWire oneWire_2(ONE_WIRE_BUS_2);
DallasTemperature ds18b20_1(&oneWire_1);
DallasTemperature ds18b20_2(&oneWire_2);

//INICIALIZACION DEL CLIENTE WIFI Y ASOCIACION A MQTT
WiFiClient espClient;
PubSubClient client(espClient);

//GENERALES
int muestreo = 25000;   //TIEMPO DE MUESTREO
int sendData = 500;    //RETRASO PARA ENVIO DE DATOS
//String msjMuestreo = String("Muestreo cada ") + (muestreo/1000) + String("s");
long lastMsg = 0;
//char msg[50];
int value = 0;

void setup() {
  Serial.begin(9600);
  
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  
  ds18b20_1.begin();
  ds18b20_2.begin();
  
  pinMode(CTRL_MUX, OUTPUT);
  pinMode(OUT_MUX_S0, OUTPUT);
  pinMode(OUT_MUX_S1, OUTPUT);
  pinMode(OUT_MUX_S2, OUTPUT);
}

void setup_wifi() {
   delay(100);
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) 
    {
      delay(500);
      Serial.print(".");
    }
  randomSeed(micros());
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) 
{
  char msj[length + sizeof(topic) + tamTopic ];
  sprintf(msj,"%s%s%s","Topic : [",topic,"] : ");
  Serial.print(msj);
  int i = 0;
  for( i=0 ; i < length ; i++){
    Serial.print(char(payload[i]));
  }
  Serial.println();
}

void reconnect() {
  while (!client.connected()) 
  {
    Serial.print("Conectando MQTT ...");
    //if you MQTT broker has clientID,username and password
    //please change following line to    if (client.connect(clientId,userName,passWord))
    if (client.connect(cliente.c_str()))
    {
      Serial.println("connected");
      client.subscribe( topic ); //once connected to MQTT broker, subscribe command if any
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      // Wait 6 seconds before retrying
      delay(2000);
    }
  }
} //end reconnect()

void getTempDS18B20(int pin){ // read with raw sample data.
  char lectura[6] = {};
  char sens_topic[15] = {};
  float f = 0;
  int n_disp = 0;
  switch(pin){
    case D1:
      ds18b20_1.requestTemperatures();
      n_disp = ds18b20_1.getDeviceCount();
      for(int i = 0; i < n_disp; i++){
        f = ds18b20_1.getTempCByIndex(i);
        sprintf(sens_topic,"%s%d%s%d","TE_18B20_", (int)pin,"_", (int)i );
        sprintf(lectura,"%d.%02d",(int)f, (int)(f*100)%100);
        Serial.print(sens_topic);Serial.print(":");Serial.println(lectura);
        sendDataMQTT(sens_topic,lectura);
        delay(sendData);
      }
      break;
    case D4:
      ds18b20_2.requestTemperatures();
      f = ds18b20_2.getTempCByIndex(0);
      sprintf(sens_topic,"%s%d","TE_18B20_", (int)pin);
      sprintf(lectura,"%d.%02d",(int)f, (int)(f*100)%100);
      Serial.print(sens_topic);Serial.print(":");Serial.println(lectura);
      sendDataMQTT(sens_topic,lectura);
      delay(sendData);    
      break;
  }
}

void sendDataMQTT(char topicM[], char lectura[] ){
  char tema[30] = {};
  strcat(tema,topic);
  //strcat(tema,"/");
  strcat(tema,topicM);
  client.publish(tema,lectura);
}

void getTempHumDHT11(int pin){
     int chk = DHT.read11(pin);
     float f_TEMP = DHT.temperature;
     float f_HUM  = DHT.humidity;
     
     //Serial.println(f_TEMP);
     //Serial.println(f_HUM);
     
     char lecturaTE[6] = {};
     char sensTE[11] = {};

     sprintf(sensTE,"%s%d","TE_DHT11_", (int)pin);
     sprintf(lecturaTE,"%d.%02d",(int)f_TEMP, (int)(f_TEMP*100)%100);
     
     char lecturaHU[6] = {};
     char sensHU[11] = {};

     sprintf(sensHU,"%s%d","HU_DHT11_", (int)pin);
     sprintf(lecturaHU,"%d.%02d",(int)f_HUM, (int)(f_HUM*100)%100);

     delay(DHT11_DELAY);
     Serial.print(sensTE);Serial.print(":");Serial.println(lecturaTE);
     sendDataMQTT(sensTE,lecturaTE);
     delay(sendData);
     Serial.print(sensHU);Serial.print(":");Serial.println(lecturaHU);
     sendDataMQTT(sensHU,lecturaHU);
}

void getMultiplexacion8Bits() {
  digitalWrite(CTRL_MUX, HIGH);
  delay(3000);
  for (int i = 0; i < 8; i++){
    digitalWrite(OUT_MUX_S0, HIGH && (i & B00000001));
    digitalWrite(OUT_MUX_S1, HIGH && (i & B00000010));
    digitalWrite(OUT_MUX_S2, HIGH && (i & B00000100));
    Mux1_State[i] = analogRead(ANALOG);
  }
  digitalWrite(OUT_MUX_S0, LOW);
  digitalWrite(OUT_MUX_S1, LOW);
  digitalWrite(OUT_MUX_S2, LOW);

  digitalWrite(CTRL_MUX, LOW);
}

void interpretaAnalog8Bits(){
  //8 posiciones por ser un multiplexor de 8 canales, (MUX - HCF4051) elementos:
  //[elemento]:<pin>
  //[0]:13  [1]:14  [2]:15  [3]:12  [4]:1 [5]:5 [6]:2 [7]:4
  //U: Ultravioleta    S: Suelo     L: Luminosidad       F: Fotoresistencia
  char sensores[8] = {'L','L','S','S','U','F','F','F'};
  String modelos[8] = {"TEMT6_99_0","TEMT6_99_1","SOILH_99_2","SOILH_99_3","GYML8_99_4","FOTOR_99_5","FOTOR_99_6","FOTOR_99_7"};
  char msj[14] = {};
  char lectura_sa[6] = {};
  String msjS;
  float coeficiente_lineal = 100.0/1024.0;
  String tipo_sensor;
  String modelo_sensor;
  for (int i = 0; i < 8; i++){    //CALIBRACION POR SENSOR
    
    switch(sensores[i]){
      case 'L':
        Mux1_State_Pctje[i] = Mux1_State[i] * coeficiente_lineal;
        msjS = "LX_";
        Serial.print("LUX:");
        break;
      case 'S':
        Mux1_State_Pctje[i] = getPctje(Mux1_State[i], 1023, 950);
        msjS = "SL_";
        Serial.print("SOIL:");
        break;
      case 'U':
        Mux1_State_Pctje[i] = mapfloat( (3.3 * Mux1_State[i]/1023) , 0.99, 2.9, 0.0, 15.0);
        msjS = "UV_";
        Serial.print("UV:");
        break;
      case 'F':
        Mux1_State_Pctje[i] = 100.0 - (Mux1_State[i] * coeficiente_lineal);
        msjS = "LX_";
        Serial.print("FOTO_LX:");
        break;
    }
    
    Serial.print(Mux1_State[i]); Serial.print(" PCTJE:"); Serial.println(Mux1_State_Pctje[i]);
    msjS = msjS + modelos[i];
    msjS.toCharArray(msj,14);   
    sprintf(lectura_sa,"%0.2f",Mux1_State_Pctje[i]);
    //Serial.print(msj);Serial.print(">>");Serial.println(lectura_sa);
    sendDataMQTT(msj,lectura_sa);

  /*char cad[50] = {};
  strcat(cad,topic);
  strcat(cad,msj);
  //Serial.println(cad);
  client.publish(cad,lectura_sa);
*/
    //delay(sendData);
    //Serial.print("MQTT:");Serial.print(msj);Serial.print(">");Serial.println(lectura_sa);
  }
  Serial.println("-------------------------------------------");
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int getPctje(int medicion, int min, int max){
  medicion = constrain(medicion,min,max); //Establecer los rangos (CALIBRAR)
  medicion = map(medicion,min,max,100,0); //min - 100% , max - 0%
  return medicion;
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  long now = millis();
  if (now - lastMsg > muestreo) {
     lastMsg = now;
     getTempDS18B20(ONE_WIRE_BUS_1); //1s
     getTempDS18B20(ONE_WIRE_BUS_2); //1s
     
     getTempHumDHT11(DHT11_PIN_1); //1s
     getTempHumDHT11(DHT11_PIN_2); //1s
    
     getMultiplexacion8Bits();   //2s
     interpretaAnalog8Bits();   //Conversion de valores Analogos a 8bits (0-1023) a Porcentajes
     //Es quien provoca el desbordamiento de pila + posiblemente el cable tambien
     //recordar que este fue el que se modifico y dejo de tronar con la version de la placa anterior
     //Se actualizo la placa y DallasTemperature, y volvio a tronar, revisar los chars, cadenas y sprinf
     
     /*char analogos[40];
     String msg="A: ";
     for(int i = 0; i < 8; i ++) {
      if(i == 7) {
        Serial.print(Mux1_State[i]);
        Serial.print(":");
        Serial.println(Mux1_State_Pctje[i]);
        msg = msg + Mux1_State[i];
      } else {
        Serial.print(Mux1_State[i]);
        Serial.print(":");
        Serial.print(Mux1_State_Pctje[i]);
        Serial.print(",");
        msg = msg + Mux1_State[i] + "," ;
      }
    }
    char message[58];
    msg.toCharArray(message,40);
    client.publish(topic, message);*/
  }
}

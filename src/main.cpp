#define TINY_GSM_MODEM_SIM800 //Tipo de modem que estamos usando
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
//#include <TinyGPS++.h>


#define TOPIC "emergencia"

// PINOUT UTILIZADOS //
#define ONBOARD_LED  2 // PINE DEL LED (no se usa)
  // PINES DEL GPS
#define GPS_RXPin 16
#define GPS_TXPin 17
  // PINES DEL GSM
#define GSM_RXPin 4
#define GSM_TXPin 2

//CONFIGURAMOS MQTT LIBRERIA//
const char *mqtt_server = "52.23.115.184";
const int mqtt_port = 1883;
const char *mqtt_user = "web_client"; // Se utiloz?
const char *mqtt_pass = "121212"; // Se utilizo?


// OBJETOS //
//Canal serial que vamos usar para comunicarmos GSM
HardwareSerial SerialGSM(1);
TinyGsm modemGSM(SerialGSM);
TinyGsmClient gsmClient(modemGSM);

//Canal serial que vamos usar para comunicarmos Gps
//HardwareSerial  neogps(2); // The serial connection to the GPS device
//TinyGPSPlus gps; // The TinyGPS++ object
//CONFIGURAMOS MQTT
PubSubClient client(gsmClient);
//TIEMPO DE ULTIMO ENVIO
uint32_t lastTime = 0;
unsigned long tiempo;


char msg[40];
long lastMsg = 0;



// DECLARAR FUNCIONES A USAR
void GET_THE_GPS(float *longitud, float *latitud);
void setupGSM();
void data_to_publish_MQTT();
void reconnect();
void Check_GSM_Network();
String readRSSI();

void setup()
{
	pinMode(ONBOARD_LED, OUTPUT);
	Serial.begin(115200);  // SeriaL ESP32 -> PC
  // neogps.begin(9600,SERIAL_8N1, GPS_RXPin,GPS_TXPin); // SeriaL GPS -> ESP32
  SerialGSM.begin(9600, SERIAL_8N1, GSM_RXPin, GSM_TXPin, false); // SeriaL GSM -> ESP32
  Serial.println("SIM800L serial initialize");
	setupGSM(); //Inicializa e configura o modem GSM
	randomSeed(micros());
  // CONFIG MQTT
	client.setServer(mqtt_server, mqtt_port);
  // Config Cripto

  

}

void loop()
{

  if(!client.connected())
    {
    reconnect();
    }
  client.loop();

  long now = millis();
  if(now - lastTime > 500){
    lastMsg = now;
    /// GET THE RSSI ///
    String RSmyRSSI = readRSSI();
    int RSSI = RSmyRSSI.toInt();
    /// GET THE GPS ///
		float lon = 0;
		float lat =0;
		GET_THE_GPS(&lon,&lat);
    // GET THE ALERT TAG //
    String msg_alert = "1";
    String to_send = "RSSI:" + String(RSSI) + "," +  "lon:" + String(lon) + "," +  "lat:" + String(lat) + "," + "Alert:"+ msg_alert ;
		to_send.toCharArray(msg, 40);
		Serial.print("Publicamos Mensaje No seguro -> "); 
		Serial.println(msg);
		client.publish("msj_no_seguro", msg);


    delay(5000);
    }
}

void setupGSM()
{
  Serial.println("Initializing modem...");
  modemGSM.init();
  Serial.println(modemGSM.getModemInfo());
  Serial.println("Setup GSM...");//display.println("Setup GSM...");
  delay(100);
  //Mostra informação sobre o modem
  Serial.println(modemGSM.getModemInfo());
  //Inicializa o modem
  if (!modemGSM.restart())
  {
    Serial.println("Restarting GSM Modem failed");//display.println("Restarting GSM Modem failed");
    delay(5000);
    ESP.restart();
    return;
  }

  //Espera pela rede
  if (!modemGSM.waitForNetwork())
  {
    Serial.println("Failed to connect to network");//display.println("Failed to connect to network");
    delay(5000);
    ESP.restart();
    return;
  }

  //Conecta à rede gprs (APN, usuário, senha)
  if (!modemGSM.gprsConnect("", "", "")) {
    Serial.println("GPRS Connection Failed");//display.println("GPRS Connection Failed");
    delay(10000);
    ESP.restart();
    return;
  }

  Serial.println("Setup GSM Success");//display.println("Setup GSM Success");
}

void reconnect() {

    while (!client.connected()) {
        Serial.println("Intentando conexión Mqtt...");
        Serial.print("Credenciales:");
        Serial.print(mqtt_server);
        Serial.print("+");
        Serial.print(mqtt_user);
        Serial.print("+");
        Serial.print(mqtt_pass);

        // Creamos un cliente ID
        String clientId = "esp32_";
        clientId += String(random(0xffff), HEX);
        // Intentamos conectar
        if (client.connect(clientId.c_str(),mqtt_user,mqtt_pass)) {
            Serial.println("Conectado!");
            // Nos suscribimos
            client.subscribe("led1");
            client.subscribe("led2");
        } else {
            Serial.print("falló :( con error -> ");
            Serial.print(client.state());
            Serial.println(" Intentamos de nuevo en 5 segundos");
            Check_GSM_Network();
            delay(2000);
        }
    }
}


String readRSSI(){

SerialGSM.println("AT+CSQ");
  bool encontrada = false;
  String cadena_entrante = "";
  tiempo = millis();
  while(!encontrada) {
    if(SerialGSM.available() > 0) {
      char c = SerialGSM.read();
      if(c == '\n') {
        cadena_entrante += '\0';
        //Serial.println(cadena_entrante);
        if(cadena_entrante.indexOf("+CSQ:") >= 0) {
          encontrada = true;
        }
        else cadena_entrante = "";
      }
      else cadena_entrante += c;
    }
    if(millis() - tiempo >= 30000UL) { // si en 30 segundos no hay respuesta valida, sale del while
      break;
    }
  }
  cadena_entrante.remove(0, 5);
  return cadena_entrante;
}


void Check_GSM_Network(){
  Serial.println("Chekando Conexion a red GSM");
  if (!modemGSM.waitForNetwork())
  {
    Serial.println("Falla Conexion de GSM a red");
    delay(4000);
    setupGSM();

  }
  else {
    Serial.println("Hay Red!!");
    return;
  }
}


void GET_THE_GPS(float *longitud, float *latitud){
	

  *longitud = random(1, 500) / 100.0;
	*latitud = random(1, 500) / 100.0;

}
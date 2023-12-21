// Station LoRa connectée

// A commenter selon qu'on souhaite programmer l'emetteur ou le recepteur

 #define emitter // pour programmer le module comme émetteur
//#define receiver // pour programmer le module avec l'écran comme récepteur#include <Arduino.h>

//****************************************************************************************************************
// Liste des bibliothèques
#include <WiFi.h>
#include <LittleFS.h> // gestion des 4Mo de stockage de l'ESP
#include <LoRa_E32.h>
#include <DNSServer.h> // page web lora
// #include <TFT_eSPI.h>          // gestion de l'ecran du recepteur
#include <ESPAsyncWebServer.h> // page web lora

#include <LiquidCrystal_I2C.h>

#include <Adafruit_Sensor.h> // Bibliothèques pour gérer la sonde de température DHT22
#include <DHT.h>
#include <DHT_U.h>

//****************************************************************************************************************
// Parametre pour le DHT22 température et humidité de l'air
#define DHTPIN 25     // Pin de connection du DHT22
#define DHTTYPE DHT22 // DHT 22 (AM2302)
DHT_Unified dht(DHTPIN, DHTTYPE);

//****************************************************************************************************************
// Id de connexion WiFi inutile ici
const char *ssid = "dfvdef"; //"devolo-30d32d310fc9"; // id de la connexion WiFi
const char *password = "HRAMNGBUZOKAFWLG";

//****************************************************************************************************************
// intances pour le AJAX
AsyncWebServer server(80);
AsyncEventSource events("/events");

//****************************************************************************************************************
// Module Lora
/*
 * LoRa E32-TTL-100
 * Start device or reset to send a message
 * https://mischianti.org
 *
 * E32-TTL-100----- Wemos D1 mini
 * M0         ----- GND
 * M1         ----- GND
 * TX         ----- PIN D2 (PullUP)
 * RX         ----- PIN D3 (PullUP)
 * AUX        ----- Not connected
 * VCC        ----- 3.3v/5v
 * GND        ----- GND
 *
 */

const String INDENTIFIANT_LORA = "ABC"; // Identifiant unique pour associer chaque emeteurs/recepteur

# define DUREE_SOMMEIL 10  // Durée en secondes du sommeil entre deux envois des infos

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */

// pin du port série pour communiquer avec le module LoRa
#define PIN_RX 21 // a connecter sur la pin TX du LoRa
#define PIN_TX 22 // a connecter sur la pin RX du LoRa
#define PIN_MOSFET 15

// autre pins du module LoRa
#define AUX 27
#define M0 32
#define M1 33
LoRa_E32 e32ttl(&Serial2, AUX, M0, M1); //  RX AUX M0 M1

//****************************************************************************************************************
// Variable stockée dans le RTC de l'ESP et qui est donc conservée durant la phase de sommeil
RTC_DATA_ATTR int bootCount = 0;

//****************************************************************************************************************
// Definition des parametres de la sonde d'humidité 2 broches
#define PIN_ADC1 34

//****************************************************************************************************************
// Definition des parametres de l'écran LCD
#define PIN_SDA 15
#define PIN_SDC 13
LiquidCrystal_I2C LCD(0x27, 20, 4);
// Matrice du caractère "°" pour l'écran LCD
byte customChar0[8] = {
    0b00000,
    0b00100,
    0b01010,
    0b00100,
    0b00000,
    0b00000,
    0b00000,
    0b00000};

//****************************************************************************************************************
// Constantes
#define ACT_WEB 5000 // intervalle de temps en ms qu'il faut attendre avant d'actualiser la page web
#define ACT_LORA 200 // intervalle de temps en ms qu'il faut attendre avant d'interroger le module LoRa

//****************************************************************************************************************
// Variables globales

String actualInfo = "";           // infos diverses...
String actualTemperature = "NC";  // température reçue de la sonde DHT22
String actualAirHumidity = "NC";  // humidité reçue de la sonde DHT22
String actualSoilHumidity = "NC"; // humidité reçue de la sonde avec 2 broches

String actualId = "NC"; // n° d'identification du dernier message reçu

long momentActualisationWeb; // dernier instant ou la page web a été actualisée
long momentActualisationLCD; // dernier instant ou l'écran TFT a été actualisé
long momentEcouteLoRa;       // dernier instant ou le module LoRa a été interrogé

bool needRefreshLCD = true; // flag pour le rafraichissement de l'écran

IPAddress HTTPS_ServerIP;
DNSServer dnsServer;

int cpt = 0;

//****************************************************************************************************************
// Class surchargée pour créer une reponse au requete DNS (Pas vraiment utile...)
class CaptiveRequestHandler : public AsyncWebHandler
{
public:
  CaptiveRequestHandler() {}
  virtual ~CaptiveRequestHandler() {}

  bool canHandle(AsyncWebServerRequest *request)
  {
    // request->addInterestingHeader("ANY");
    return true;
  }

  void handleRequest(AsyncWebServerRequest *request)
  {
    request->send(LittleFS, "/index.html", "text/html", false);
    AsyncResponseStream *response = request->beginResponseStream("text/html");

    response->print("<!DOCTYPE html><html><head><title>Captive Portal</title></head><body>");
    response->print("<p>This is out captive portal front page.</p>");
    response->printf("<p>You were trying to reach: http://%s%s</p>", request->host().c_str(), request->url().c_str());
    response->printf("<p>Try opening <a href='http://%s'>this link</a> instead</p>", WiFi.softAPIP().toString().c_str());
    response->print("</body></html>");
    // response->print(accueil);
    request->send(response);

    // request->send(LittleFS, "/index.html", "text/html", false);
    // Serial.println("DNS handler ");
  }
};

//****************************************************************************************************************
// Procédure pour envoyer les infos de l'ESP32
void infoESP()
{

  /* Print chip information */
  esp_chip_info_t chip_info;
  uint32_t flash_size;
  esp_chip_info(&chip_info);
  printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
         CONFIG_IDF_TARGET,
         chip_info.cores,
         (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
         (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
         (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
         (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

  unsigned major_rev = chip_info.revision / 100;
  unsigned minor_rev = chip_info.revision % 100;
  printf("silicon revision v%d.%d, ", major_rev, minor_rev);
  if (esp_flash_get_size(NULL, &flash_size) != ESP_OK)
  {
    printf("Get flash size failed");
    return;
  }

  printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
         (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

  printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());
};

//****************************************************************************************************************
// Liste des handlers utiliser par le serveur  pour repondre aux requêtes

// poignée de la page d'accueil sur serveur web
void handleRoot(AsyncWebServerRequest *request)
{
  // Serial.println("handle root");
  request->send(LittleFS, "/index.html", "text/html", false);
}

// poingnée pour servir les feuilles de styles
void handleStyleCSS(AsyncWebServerRequest *request)
{
  // Serial.println("handle css");
  request->send(LittleFS, "/monstyle.css", "text/css", false);
}

// poignée pour servir l'image LoRa de la page d'accueil
void handleImageLoRa(AsyncWebServerRequest *request)
{
  // Serial.println("handle image LoRa");
  request->send(LittleFS, "/lora.png", "image/png", false);
}

// poignée pour traiter la commande envoyée par la page web
void handleCommande(AsyncWebServerRequest *request)
{
  // Serial.println("handleCommande");
  String s = request->getParam("text_commande", true)->value();
  // Serial.println("commande " + s + " reçue de la station");
  request->redirect("/");
}

// poignée pour fichier inexistant du serveur web
void notFound(AsyncWebServerRequest *request)
{
  request->send(404, "text/plain", "Not found");
}

//****************************************************************************************************************
// procédure qui imprime les paramètres actuels du module LoRa
void printParameters(struct Configuration configuration)
{
  Serial.println("----------------------------------------");

  Serial.print(F("HEAD BIN: "));
  Serial.print(configuration.HEAD, BIN);
  Serial.print(" ");
  Serial.print(configuration.HEAD, DEC);
  Serial.print(" ");
  Serial.println(configuration.HEAD, HEX);
  Serial.println(F(" "));
  Serial.print(F("AddH BIN: "));
  Serial.println(configuration.ADDH, BIN);
  Serial.print(F("AddL BIN: "));
  Serial.println(configuration.ADDL, BIN);
  Serial.print(F("Chan BIN: "));
  Serial.print(configuration.CHAN, DEC);
  Serial.print(" -> ");
  Serial.println(configuration.getChannelDescription());
  Serial.println(F(" "));
  Serial.print(F("SpeedParityBit BIN    : "));
  Serial.print(configuration.SPED.uartParity, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.SPED.getUARTParityDescription());
  Serial.print(F("SpeedUARTDataRate BIN : "));
  Serial.print(configuration.SPED.uartBaudRate, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.SPED.getUARTBaudRate());
  Serial.print(F("SpeedAirDataRate BIN  : "));
  Serial.print(configuration.SPED.airDataRate, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.SPED.getAirDataRate());

  Serial.print(F("OptionTrans BIN       : "));
  Serial.print(configuration.OPTION.fixedTransmission, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getFixedTransmissionDescription());
  Serial.print(F("OptionPullup BIN      : "));
  Serial.print(configuration.OPTION.ioDriveMode, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getIODroveModeDescription());
  Serial.print(F("OptionWakeup BIN      : "));
  Serial.print(configuration.OPTION.wirelessWakeupTime, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getWirelessWakeUPTimeDescription());
  Serial.print(F("OptionFEC BIN         : "));
  Serial.print(configuration.OPTION.fec, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getFECDescription());
  Serial.print(F("OptionPower BIN       : "));
  Serial.print(configuration.OPTION.transmissionPower, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getTransmissionPowerDescription());

  Serial.println("----------------------------------------");
}

// procédure qui imprime les infos du module LoRa
void printModuleInformation(struct ModuleInformation moduleInformation)
{
  Serial.println("----------------------------------------");
  Serial.print(F("HEAD BIN: "));
  Serial.print(moduleInformation.HEAD, BIN);
  Serial.print(" ");
  Serial.print(moduleInformation.HEAD, DEC);
  Serial.print(" ");
  Serial.println(moduleInformation.HEAD, HEX);

  Serial.print(F("Freq.: "));
  Serial.println(moduleInformation.frequency, HEX);
  Serial.print(F("Version  : "));
  Serial.println(moduleInformation.version, HEX);
  Serial.print(F("Features : "));
  Serial.println(moduleInformation.features, HEX);
  Serial.println("----------------------------------------");
}

bool wifiOk = false; // true si l'ESP a pu se connecter à internet

//*************************************************************************************************************************
//         Procedure qui met l'esp en someil profond pendant temps secondes
//*************************************************************************************************************************
void sommeil(int temps)
{

  Serial.flush();
  int rep = esp_sleep_enable_timer_wakeup((temps)*uS_TO_S_FACTOR);
  if (rep == ESP_OK)
  {
    Serial.println("Sommeil pendant " + String(temps) + " secondes");
    esp_deep_sleep_start();
  }
  else
  {
    Serial.println("Erreur : impossible de basculer en sommeil avec cette durée");
    Serial.println("l'ESP va rebooter dans 5s");
    delay(5000);
    ESP.restart();
  }
}

//****************************************************************************************************************
// Procedure qui affiche les données sur l'écran LCD
void refreshLCD()
{

  LCD.clear();
  LCD.setCursor(0, 0);
  LCD.print("Info: " + actualInfo);

  LCD.setCursor(0, 1);
  LCD.print("id: " + actualId);

  LCD.setCursor(0, 2);
  LCD.print("Air:" + actualTemperature);
  LCD.write((byte)0);
  LCD.print('C');

  LCD.print("|" + actualAirHumidity + "%");

  LCD.setCursor(0, 3);
  LCD.print("Hum. sol: " + actualSoilHumidity + "%");

  needRefreshLCD = false;
}

//*************************************************************************************************************************
//         Procedure qui scanne le bus I2C
//*************************************************************************************************************************
void scanI2C()
{
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
      {
        Serial.print("0");
      }
      Serial.println(address, HEX);
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknow error at address 0x");
      if (address < 16)
      {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
}

//******************************************************************************************************
//   SETUP    ******************************************************************************************
void setup()
{

  /***************************************************************************************************/
  // Début partie commune emitter et receiver

  // demarrage du port série n°1 utilisé pour le débugage
  Serial.begin(115200);
  Serial.println("Début!!");

  infoESP();

  // demarrage du port série n°2 utilisé par le module loRa
  Serial.println("initialisation port série #2 ");
  Serial2.begin(9600, SERIAL_8N1, PIN_RX, PIN_TX);

  Wire.begin(PIN_SDA, PIN_SDC);

  scanI2C();

  // Fin partie commune emitter receiver
  /***************************************************************************************************/

#ifdef emitter
  Serial.println("Démarrage n°" + String(bootCount));
  // initialisation du module LoRa
  pinMode(PIN_MOSFET, OUTPUT);
  pinMode(PIN_ADC1, INPUT_PULLDOWN);
  // Allumage du regulateur de tension LM via le mosfet
  Serial.println("Allumage du régulateur par le mosfet (et attente 2s)");
  digitalWrite(PIN_MOSFET, HIGH);
  delay(2000);
  Serial.println("initialisation du module LoRa");
  e32ttl.begin();
  String message = INDENTIFIANT_LORA + ";";
  message += "id:" + String(bootCount);

  delay(1000);
  Serial.println("Demarrage capteur DHT22");
  dht.begin();

  sensors_event_t event;
  dht.temperature().getEvent(&event);

  if (isnan(event.temperature))
  {
    Serial.println(F("Error reading temperature!"));
    message += String(";") + "temp:" + "NC";
  }
  else
  {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
    message += String(";") + "temp:" + String(event.temperature);
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity))
  {
    Serial.println(F("Error reading humidity!"));
    message += String(";") + "airHum:" + "NC";
  }
  else
  {
    Serial.print(F("Air humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
    message += String(";") + "airHum:" + String(event.relative_humidity);
  }

  // Lecture de la tension de la sonde 2 broche
  int adc = analogRead(PIN_ADC1);
  float sh = 100.0 - float(adc) / float(4096) * 100.0;
  actualSoilHumidity = String(sh);
  Serial.println("Soil humidity: " + actualSoilHumidity + "%");
  message += String(";") + "soilHum:" + actualSoilHumidity;

  Serial.println("Envoi des infos ");
  ResponseStatus rs = e32ttl.sendMessage(message);
  // Check If there is some problem of succesfully send
  Serial.println(rs.getResponseDescription());
  delay(1000);

  bootCount++;
  Serial.println("Arrêt du régulateur par le mosfet dans 2s");
  delay(2000);
  digitalWrite(PIN_MOSFET, LOW);

  pinMode(PIN_MOSFET, INPUT_PULLDOWN); // idealement il aurait fallu une resistance pulldown sur PIN_MOSFET...oubliée

  Serial.println("mise en sommeil");

  sommeil(DUREE_SOMMEIL);
#endif

#ifdef receiver

  Serial.println("initialisation de l'ecran ");
  LCD.begin(20, 4); // initialisation du LCD
  LCD.backlight();  // Allumage du retroeclairage
  LCD.print("ALBILAB");
  LCD.createChar(0, customChar0); // create a new custom character (index 0)

  // initialisation de variables pour le serveur web
  momentActualisationWeb = millis();
  momentActualisationLCD = millis();
  refreshLCD();

  // initialisation du système de fichier
  Serial.println("initialisation littleFS");
  LittleFS.begin();
  delay(500);
  Serial.println("initialisation module LoRa");
  e32ttl.begin();

  // Tentative de connexion au reseau WiFi
  // Serial.println("Tentative de connexion au reseau WiFi :" + String(ssid));
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {

    // Serial.println("Erreur : Connexion impossible !");
    // Serial.println("L'ESP bascule en mode point d'accès");
    //  on remet l'esp en mode point d'accès
    WiFi.mode(WIFI_AP);
    // IPAddress local_IP(192, 168, 4, 22);
    // IPAddress gateway(192, 168, 4, 9);
    // IPAddress subnet(255, 255, 255, 0);
    //  WiFi.mode(WIFI_AP); // on met l'esp en mode point d'accès
    WiFi.softAP("ESP LoRa", "", 10, 0, 4, false);
    delay(500);
    // Serial.println("point d'accès activé");
    // Serial.println(WiFi.softAPIP());
    // Serial.println("Setting soft-AP configuration ... ");
    //  Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ? "Ready" : "Failed!");
    //  Serial.print("Setting soft-AP ... ");
    //  Serial.println(WiFi.softAP(ssid,password) ? "Ready" : "Failed!");
    //   on active le P.A.
    //  Server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER); //only when requested from AP
    //  dnsServer.start(53, "*", WiFi.softAPIP());

    // dnsServer.start(53, "*",local_IP);
  }
  else
  {
    // Serial.println("Connection WiFi réussie");
    wifiOk = true;
    HTTPS_ServerIP = WiFi.localIP(); // IP actuelle
    String IPlocale = String(HTTPS_ServerIP[0]) + "." + String(HTTPS_ServerIP[1]) + "." + String(HTTPS_ServerIP[2]) + "." + String(HTTPS_ServerIP[3]);
    // Serial.println("IP : " + IPlocale);
  }
  // initialisation des requêtes gérées par le serveur pour la page web
  Serial.println("Initialisation du serveur");
  server.on("/", HTTP_GET, handleRoot);
  server.on("/monstyle.css", handleStyleCSS);
  server.on("/lora.png", handleImageLoRa);
  server.onNotFound(notFound);
  server.on("/COMMANDE", HTTP_POST, handleCommande);

  events.onConnect([](AsyncEventSourceClient *client)
                   {
                     if (client->lastId())
                     {
                       Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
                     }
                     // send event with message "hello!", id current millis
                     // and set reconnect delay to 1 second
                     // client->send("hello!", NULL, millis(), 10000);
                   });

  // initialisation Ajax
  Serial.println("Initialisation des events Ajax");
  server.addHandler(&events);
  Serial.println("Démarrage du serveur");
  server.begin();
#endif
}


//******************************************************************************************************
// Procedure qui place les données reçues dans les variables globales du programme
void traiteMessage(String message)
{
  if (message.indexOf(':') != -1)
  {
    String partie1 = message.substring(0, message.indexOf(':'));
    String partie2 = message.substring(message.indexOf(':') + 1, message.length());
    if (partie1 == "id")
    {
      Serial.println("Actualisation numéro identifiant");
      actualId = partie2;
    }
    else if (partie1 == "temp")
    {
      Serial.println("Actualisation température");
      actualTemperature = partie2;
    }
    else if (partie1 == "airHum")
    {
      Serial.println("Actualisation air humidité");
      actualAirHumidity = partie2;
    }
    else if (partie1 == "info")
    {
      Serial.println("Actualisation info");
      actualInfo = partie2;
    }
    else if (partie1 == "soilHum")
    {
      Serial.println("Actualisation sol humidité");
      actualSoilHumidity = partie2;
    }

    else
    {
      Serial.println("Erreur : entête message inconnue !");
      actualInfo = "Err. entete!";
    }
  }
  else
  {
    Serial.println("Erreur : format de message inconnue !");
    actualInfo = "format!";
  }
}
//******************************************************************************************************
//                                           LOOP   
//******************************************************************************************************

void loop()
{
#ifdef receiver

  // Serial.println("Début loop");

  //  Actualisation de la page WEB si c'est le moment
  if (millis() > (momentActualisationWeb + ACT_WEB))
  {
    momentActualisationWeb = millis();
    Serial.println("Actualisation page web si nécessaire");
    cpt++;
    int connexion = WiFi.softAPgetStationNum();
    Serial.println("Nombre de connexion(s) :" + String(connexion));
    if (connexion >= 1)
    {
      Serial.println("connexion >=1 -> envoi des nouveaux paramètres");
      momentActualisationWeb = millis();
      events.send(actualId.c_str(), "identifiant", millis());
      events.send(actualTemperature.c_str(), "temperature", millis());
      events.send(actualAirHumidity.c_str(), "airHumidity", millis());
      events.send(actualSoilHumidity.c_str(), "soilHumidity", millis());
      
      events.send(actualInfo.c_str(), "info", millis());
    }
    else
    {
      Serial.println("Aucun clients connectés -> pas d'envoi");
    }
  }

  // Actualisation de l'écran si necessaire
  if (needRefreshLCD)
  {
    Serial.println("Actualisation LCD");
    refreshLCD();
    needRefreshLCD = false;
  };

  // Ecoute du module LoRa si c'est le moment
  if (millis() > (momentEcouteLoRa + ACT_LORA))
  {
    Serial.println("Ecoute du LoRa");
    momentEcouteLoRa = millis();

    if (e32ttl.available() > 1)
    {
      ResponseContainer rs = e32ttl.receiveMessage();
      if (rs.status.code != E32_SUCCESS)
      {
        Serial.print("Erreur lors de la reception d'un message :");
        Serial.println(rs.status.getResponseDescription());
        actualInfo = "reception!";
        needRefreshLCD = true;
      }
      else
      {
        String message = rs.data; // First ever get the data
        Serial.print("Reception du message :" + message);
        Serial.println();
        Serial.println(rs.status.getResponseDescription());
        actualInfo = "";

        // on regarde si le premier parametre du message correspond à notre identifiant LoRa
        if (message.substring(0, message.indexOf(';')) == INDENTIFIANT_LORA)
        { // si OUI on traite le message
          needRefreshLCD = true;
          message = message.substring(message.indexOf(';') + 1, message.length());
          // découpe  en sous messages et traitement
          while (message.indexOf(';') != -1)
          {
            String subMessage = message.substring(0, message.indexOf(';'));
            Serial.println("sous message:" + subMessage);
            traiteMessage(subMessage);
            message = message.substring(message.indexOf(';') + 1, message.length());
          }
          Serial.println("sous message:" + message);
          traiteMessage(message);
        }
        else
        { // sinon on ne fait rien
          Serial.println("Ce message ne nous concerne pas");
        }

        rs.data.clear();
      }
    }
  };
#endif
}

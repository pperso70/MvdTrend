// pour acceder a la page: https://pperso70.github.io/MvdCh/
#include <Arduino.h> //pour platformio
#include <WiFi.h>
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "FastAccelStepper.h"
// #include <ArduinoOTA.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_BMP290.h>  // remplacement de BMP280
#include <WebSocketsClient.h> //links2004/WebSockets@2.4.1
#include <ArduinoJson.h>

#define NUM_VALUES 50
// Déclaration du tableau pour stocker les valeurs
String values[NUM_VALUES] = {""};

// RTC_DATA_ATTR int bootCount = 0;
int bootCount = 0;

Adafruit_BMP290 bmp; // Adafruit_BMP280 bmp; I2C

#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

#define stepPinStepper_azi 2
#define dirPinStepper_azi 14
#define enablePinStepper_azi 15
// Définir l'adresse à laquelle vous souhaitez sauvegarder la valeur
#define EEPROM_ADDRESS 0
#define EEPROM_ADDR_BOOT_COUNT 4

const int SDA_PIN = 13;     // Ajustez selon votre configuration
const int SCL_PIN = 16;     // Ajustez selon votre configuration
const int I2C_FREQ = 10000; // Fréquence I2C standard 100000(100 kHz)

const int MAX_RETRIES = 5;
const unsigned long RETRY_DELAY = 1000;     // 1 seconde entre les tentatives
const unsigned long CHECK_INTERVAL = 10000; // Vérifier toutes les 10 secondes

unsigned long lastCheckTime = 0;

bool raz_compteur = false;

// const char* ssid = "freeboxwifi";
// const char *ssid = "SFR_2CE0";
const char *ssid = "iPhone de Edith";
// const char *ssid = "Solene-G10";
//const char *ssid = "PC";
// const char *password = "1234567890";
// const char *password = "aaaaaaaa";
// const char *password = "nrup8250";
const char *password = "aaaaaaaa";
// const char *websockets_server = "testchauf.glitch.me";  //sans "https://"
// const char *websockets_server = "flash-wild-ketchup.glitch.me";  //sans "https://"
const char *websockets_server = "confirmed-careful-objective.glitch.me"; // sans "https://"
const int websockets_port = 443;

const int maxAttempts = 5;
const int delayBetweenAttempts = 5000; // 5 secondes

WebSocketsClient webSocket;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *Stepper_azi = NULL;

uint8_t cam_num;
bool connected = false;
int lampVal;
String JSONtxt;
int Ref_Position = 500;
int Init_Position = 500;
bool excursion = false;
bool exmem = false;
unsigned long memo_temps2 = 0, memo_temps3 = 0, memo_temps4 = 0, memo_temps5 = 0, memo_temps6 = 0, temps_memo_debut = 0, temps_de_cycle = 0;

int LED_ROUGE_PIN = 33; // led_rouge
int led_rouge;

const int LED_BUILTIN = 4; // led blanche
int Ref_Position_old = 0;
bool dmde_sauvegarde = false;
float temperature = 1.234;
unsigned long lastConnectionCheck = 0;
unsigned long DebutVideo = 0;
unsigned long DebutLed = 0;
const unsigned long connectionTimeout = 30000; // 1 minute en millisecond

void configCamera()
{
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  // config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  // config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // config.frame_size = FRAMESIZE_SVGA;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 9;
  config.fb_count = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
}

void webSocketEvent(WStype_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
  case WStype_DISCONNECTED:
    Serial.println("Déconnecté du WebSocket");
    break;
  case WStype_CONNECTED:
    Serial.println("Connecté au serveur WebSocket Glitch");
    break;
  case WStype_TEXT:
    Serial.printf("Reçu de Glitch: %s\n", payload);
    handleCommand(payload, length);
    break;
  default:
    Serial.printf("Type d'événement WebSocket non géré: %d\n", type);
    break;
  }
}

void sendSensorData()
{
  // Envoi des données de capteurs
  // DynamicJsonDocument doc(1024);
  StaticJsonDocument<512> doc;
  doc["temps_de_cycle"] = temps_de_cycle;
  float position = Stepper_azi->getCurrentPosition() / 175.0;
  doc["position"] = String(position, 2);       // Affichage avec 2 décimales
  doc["temperature"] = String(temperature, 2); // Affichage avec 2 décimales
  doc["rssi"] = WiFi.RSSI();                   // Simuler une humidité
  doc["restarts"] = bootCount;                 // Add the restart count
  for (int i = 0; i < NUM_VALUES; i++)
  {
    doc["point" + String(i)] = values[i];
  }

  String output;
  serializeJson(doc, output);
  // Serial.println("Envoi de données capteurs : " + output);
  webSocket.sendTXT(output);

  // Capture et envoi de l'image
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb)
  {
    Serial.println("Erreur: Impossible d'acquérir le frame buffer");
    return;
  }
  // Envoi de l'image en tant que données binaires
  // Serial.println("Envoi de l'image : " + String(fb->len) + " bytes");
  webSocket.sendBIN(fb->buf, fb->len);
  // Libération du frame buffer
  esp_camera_fb_return(fb);
}

void handleCommand(uint8_t *payload, size_t length)
{
  StaticJsonDocument<512> doc;
  // DynamicJsonDocument doc(1024);
  deserializeJson(doc, payload, length);

  if (doc.containsKey("command"))
  {
    String command = doc["command"].as<String>();
    Serial.println("Commande reçue : " + command);
    if (command == "LED_ON")
    {
      // Allumer la LED
      digitalWrite(LED_BUILTIN, HIGH); // turn the LED on
      Serial.println("Allumer la LED");
      DebutVideo = millis();
      DebutLed = millis();

      if (raz_compteur)
      {
        raz_compteur = false;
        bootCount = 0;
        sauvegarderVariable();
      }
      if (!raz_compteur)
      {
        raz_compteur = true;
      }
    }
    else if (command == "LED_OFF")
    {
      // Éteindre la LED
      digitalWrite(LED_BUILTIN, LOW); // turn the LED off
      Serial.println("Éteindre la LED");
      DebutVideo = millis();
      raz_compteur = false;
    }
    else if (command == "Up")
    {
      excursion = false;
      if (Ref_Position < 1950)
      {
        Ref_Position += 50;
      }
      Serial.println("Up_pc");
    }

    else if (command == "Down")
    {
      excursion = false;
      if (Ref_Position > 50)
      {
        Ref_Position -= 50;
      }
      Serial.println("Down_pc");
      DebutVideo = millis();
    }

    else if (command == "Excur")
    {
      excursion = true;
      Serial.println("Excursion");
      DebutVideo = millis();
    }

    else if (command == "Stop")
    {
      excursion = false;
      Serial.println("Stop");
      DebutVideo = millis();
    }

    else if (command == "Raz1000")
    {
      Ref_Position = 875;
      excursion = false;
      Stepper_azi->setCurrentPosition(875);
      Serial.println("Raz1000");
      DebutVideo = millis();
    }

    else if (command == "démarre")
    {
      Serial.println("démarre");
      DebutVideo = millis();
    }

    Serial.println(Ref_Position);
  }
}

void excur()
{
  if (excursion)
  {
    exmem = true;
    if (Ref_Position != 0 && Ref_Position != 1750)
    {
      if (Ref_Position > 875)
      {
        Ref_Position = 1750;
      }
      else
      {
        Ref_Position = 0;
      }
    }

    if (Ref_Position == 0 && Stepper_azi->getCurrentPosition() < 20)
      Ref_Position = 1750;
    if (Ref_Position == 1750 && Stepper_azi->getCurrentPosition() > 1730)
      Ref_Position = 0;
  }
  else
  {
    if (exmem)
    {
      exmem = false;
      Ref_Position = Stepper_azi->getCurrentPosition();
    }
  }
}

// Fonction pour sauvegarder la variable dans la mémoire EEPROM
void sauvegarderVariable()
{
  EEPROM.begin(10);                         // Initialise la mémoire EEPROM
  EEPROM.put(EEPROM_ADDRESS, Ref_Position); // Écrit la valeur de la variable à l'adresse spécifiée
  EEPROM.put(EEPROM_ADDR_BOOT_COUNT, bootCount);
  EEPROM.commit(); // Commence l'écriture des données
  EEPROM.end();    // Termine l'utilisation de la mémoire EEPROM
}

// Fonction pour restaurer la variable depuis la mémoire EEPROM
void restaurerVariable()
{
  EEPROM.begin(10);                         // Initialise la mémoire EEPROM
  EEPROM.get(EEPROM_ADDRESS, Ref_Position); // Lit la valeur sauvegardée à l'adresse spécifiée
  EEPROM.get(EEPROM_ADDR_BOOT_COUNT, bootCount);
  Stepper_azi->setCurrentPosition(Ref_Position);
  EEPROM.end(); // Termine l'utilisation de la mémoire EEPROM
}

// Fonction pour décaler les valeurs et insérer la nouvelle lecture
void shiftRegister(String newValue)
{
  // Décale toutes les valeurs à droite
  for (int i = NUM_VALUES - 1; i > 0; i--)
  {
    values[i] = values[i - 1];
  }
  // Place la nouvelle valeur à l'emplacement 0
  values[0] = newValue;
}

String interpretStatus(uint8_t status)
{
  String interpretation = "Statut: ";
  interpretation += ", Mesure en cours: " + String((status & 0b100) ? "Oui" : "Non");
  interpretation += ", Résultat disponible: " + String((status & 0b1000) ? "Oui" : "Non");
  return interpretation;
}

bool isSensorReadingValid()
{
  uint8_t status = bmp.getStatus();
  Serial.print("Statut du capteur: 0b");
  Serial.println(status, BIN);
  Serial.println(interpretStatus(status));

  // Vérifier si le capteur est en mode normal et qu'un résultat est disponible
  if ((status & 0b1100) != 0b1100)
  {
    Serial.println("Le capteur n'est pas dans un état de fonctionnement normal.");
    return false;
  }
}

bool initBMP290()
{
  Wire.begin(SDA_PIN, SCL_PIN, I2C_FREQ); // Initialisation I2C une seule fois
  // Wire.begin();
  if (!bmp.begin(0x76))
  {
    return false;
  }
  return true;
}

void restartBMP290()
{
  Serial.println("Tentative de réinitialisation de la liaison BMP280...");

  int retries = 0;
  while (!initBMP290() && retries < MAX_RETRIES)
  {
    Serial.println("Échec de l'initialisation. Nouvelle tentative...");
    retries++;
    delay(RETRY_DELAY);
  }

  if (retries < MAX_RETRIES)
  {
    Serial.println("BMP290 réinitialisé avec succès !");
  }
  else
  {
    Serial.println("Impossible de réinitialiser le BMP290 après plusieurs tentatives.");
  }
}

void setup()
{
  Serial.begin(115200);

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // disable  Brownout detector

  engine.init(); // init moteur pas à pas
  Stepper_azi = engine.stepperConnectToPin(stepPinStepper_azi);
  if (Stepper_azi)
  {
    Stepper_azi->setDirectionPin(dirPinStepper_azi);
    Stepper_azi->setEnablePin(enablePinStepper_azi);
    Stepper_azi->setAutoEnable(true);
    Stepper_azi->setDelayToDisable(500);
    Stepper_azi->setSpeedInHz(100);    // 400
    Stepper_azi->setAcceleration(200); // 800
  }

  Stepper_azi->setCurrentPosition(Init_Position);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < maxAttempts)
  {
    delay(1000);
    Serial.println("Tentative de connexion au WiFi...");
    attempts++;

    if (attempts == maxAttempts)
    {
      Serial.println("Échec de la connexion. Redémarrage du WiFi...");
      WiFi.disconnect();
      delay(1000);
      WiFi.begin(ssid, password);
      attempts = 0;
    }
  }
  
  Serial.println("Connecté au WiFi");

  Serial.println("Tentative de connexion à Glitch...");
  webSocket.beginSSL(websockets_server, websockets_port, "/");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_ROUGE_PIN, OUTPUT);
  configCamera();

  // ArduinoOTA.setHostname("Chauffage");  // on donne une petit nom a notre module
  // ArduinoOTA.begin();

  restaurerVariable();
  bootCount++;
  sauvegarderVariable();
  Serial.println("Boot number: " + String(bootCount));

  Serial.println("Serveur HTTP démarré");
  Serial.println(WiFi.localIP());

  Serial.print("Ref_Position:  ");
  Serial.println(Ref_Position);
  Serial.print("getCurrentPosition:  ");
  Serial.println(Stepper_azi->getCurrentPosition());
  
  if (!initBMP290())
  {
    Serial.println("Impossible de trouver le capteur BMP290. Vérifiez le câblage !");
    restartBMP290();
  }

  digitalWrite(LED_ROUGE_PIN, HIGH);

} // fin du setup

void loop()
{
  temps_memo_debut = micros();
  // webSocket.loop();
  if (millis() > 10000)
  {
    Stepper_azi->moveTo(Ref_Position);
  }
  webSocket.loop();

  excur();
  // ArduinoOTA.handle();

  // Envoyer des données toutes les 250 millis secondes au debut apres 2 minute toute les 10S
  static unsigned long lastTime = 0;
  if (millis() - DebutVideo < 120000)
  {
    if (millis() - lastTime > 250)
    {
      sendSensorData();
      lastTime = millis();
    }
  }
  else if (millis() - lastTime > 10000)
  {
    sendSensorData();
    lastTime = millis();
  }
  if (millis() - DebutLed > 40000 && digitalRead(LED_BUILTIN))
  {
    // Éteindre la LED
    digitalWrite(LED_BUILTIN, LOW); // turn the LED off
  }

  if (millis() > memo_temps2 + 100)
  { // tous les 100ms
    memo_temps2 = millis();
    if (Ref_Position != Ref_Position_old)
    {
      memo_temps5 = millis();
      dmde_sauvegarde = true;
      Ref_Position_old = Ref_Position;
    }
    if ((millis() > memo_temps5 + 5000) && dmde_sauvegarde)
    {
      dmde_sauvegarde = false;
      sauvegarderVariable(); // Sauvegarder la nouvelle valeur si elle est différente de l'ancienne
      Serial.print("Variable mise à jour : ");
      Serial.println(Ref_Position);
    }
    // Serial.println(temps_de_cycle);
  } // fin cycle 100ms

  if (millis() > memo_temps4 + 500)
  { // tous les 1000ms
    memo_temps4 = millis();
    temperature = bmp.readTemperature();

    // Appel de la fonction pour stocker la valeur dans le registre à décalage
    shiftRegister(String(temperature, 2));
    
    if (WiFi.status() != WL_CONNECTED)
    {
      if (millis() - lastConnectionCheck > connectionTimeout)
      {
        Serial.println("Connexion Wi-Fi perdue. Redémarrage...");
        ESP.restart();
      }
    }
    else
    {
      lastConnectionCheck = millis(); // Reset du timer si la connexion est rétablie
    }
  } // fin memo_temps4 + 500

  if (millis() > memo_temps6 + 5000)
  { // tous les 5s
    memo_temps6 = millis();

    byte error = Wire.endTransmission();
    Serial.print(" byte error:");
    Serial.println(error);

    byte errorbmp = bmp.getStatus();
    Serial.print("bmp.getStatus:");
    Serial.println(errorbmp);

    if (!isSensorReadingValid())
    {
      Serial.println("Lecture invalide du capteur BMP290. Tentative de redémarrage...");
      restartBMP290();
    }
  } // fin memo_temps6 + 5s

  if (millis() - memo_temps3 > 200)
  {
    memo_temps3 = millis();

    if (led_rouge == LOW)
    {
      led_rouge = HIGH; // Allumez la sortie
    }
    else
    {
      led_rouge = LOW; // Éteindre la sortie
    }
  }
  digitalWrite(LED_ROUGE_PIN, led_rouge);

  temps_de_cycle = micros() - temps_memo_debut;
} // fin du loop

//#define VNH5019
#define LCD1306
#define IBT2
#define LAME_IBT2
#define TIMER
//#define TEMPE_DS
//#define WIFIOTA
#define BT_ENABLE
#define US_SOL

#include <Arduino.h>




#if defined(WIFIOTA)

#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

WiFiMulti WiFiMulti;
const char *OTAName = "Mower_1";         // A name and a password for the OTA service
const char *OTAPassword = "PWD_Mower_1";

#endif

#if defined(BT_ENABLE)
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;
#endif




#if defined(VNH5019)
#include "DualVNH5019MotorShield.h"

//Pin map 5019
#define PIN_INA1  35
#define PIN_INB1  33
#define PIN_PWM1  34
#define PIN_EN1DIAG1  14
#define PIN_CS1  9
#define PIN_INA2  27
#define PIN_INB2  25
#define PIN_PWM2  32
#define PIN_EN2DIAG2  12
#define PIN_CS2  8
// commande de deplacement
#define STOP 0
#define AVANT_G 400
#define AVANT_D 400
#define AVANT_REDUIT 200
#define AVANT_TOUTE 400
#define ARRIERE -400
#define MOT_G 0
#define MOT_D 1
#define DUREE_ROT 1000
#endif

#if defined(IBT2)
// gestion avec 3 ibt2
#define PIN_PWM_MOT1_N  18
#define PIN_PWM_MOT1_R  19
#define PIN_PWM_MOT2_N  16
#define PIN_PWM_MOT2_R  17
#define CANAL_MOT_AV_G 1
#define CANAL_MOT_AR_G 2
#define CANAL_MOT_AV_D 3
#define CANAL_MOT_AR_D 4
#define DUREE_ROT 1000
#endif

#define PIN_MOT_LAME 2

#if defined(LAME_IBT2)
#define PIN_PWM_LAME_N  2
#define PIN_PWM_LAME_R  27
#define CANAL_LAME_AV 5
#define CANAL_LAME_AR 6
#endif

#define FREQ_PWM 5000


#if defined(US_SOL)
#define PIN_TRIG_SOL 5
#define PIN_ECHO_SOL 23
#define VAL_WARN_SOL 10
#endif

//pour wemos32 38pin
#define PIN_TRIG_CENT 12
#define PIN_ECHO_CENT 14

#define PIN_TRIG_GAU 32
#define PIN_ECHO_GAU 34

#define PIN_TRIG_DROI 33
#define PIN_ECHO_DROI 35

#define PIN_COL_GAU 13
#define PIN_COL_DROI 15

// valeur de seuil des capteurs
#define VAL_WARN_CENT 20
#define VAL_WARN_GAU 1
#define VAL_WARN_DROI 1

//definition des modes de fonctionnement
#define MODE_UPDATE 1
#define MODE_ARRET 2
#define MODE_TONDEUSE 3
#define MODE_SIMU 4
#define MODE_ARRIERE 5
#define MODE_DROITE 6
#define MODE_GAUCHE 7
#define MODE_DISTANCE 8
#define MODE_AVANCE 9

//definition des capteurs us
#define CAPTEUR_CENTRE 0b00000010 /*2*/
#define CAPTEUR_GAUCHE 0b00000001 /*1*/
#define CAPTEUR_DROITE 0b00000100 /*3*/
#define CAPTEUR_SOL 0b00001000 /*4*/


#if defined(LCD1306)
// if you have an LCD screen
#include <Wire.h>
#include "SSD1306Wire.h"        // legacy: #include "SSD1306.h"
SSD1306Wire Mydisplay(0x3c, SDA, SCL);   // ADDRESS, SDA, SCL  -  SDA and SCL usually populate automatically based on your board's pins_arduino.h
#define DEBUT_RECT_DROIT 118
#define DEBUT_RECT_GAUCHE 0
#define DEBUT_RECT_VERTICAL 20
#define LARGEUR_RECT 7
#define HAUTEUR_RECT 30
#define RAYON_LAME 20
#endif

#if defined(TIMER)
// if you have an RTC clock for start everything programaticaly
#include "uRTCLib.h"
uRTCLib rtc;
char daysOfTheWeek[7][12] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };
#endif


//variables for collision
long echo_cent, long_cent;
long echo_gau, long_gau;
long echo_droi, long_droi;
long long_sol;
int warn_cent = 0, warn_gau = 0, warn_droi = 0;
//int check_bump(int);
int vitesse_actu = 0;
int pin_trig, pin_echo;
long echo = 0;

//menu du setup
String etapes[] = { "Etape 1", "Etape 2", "Etape 3", "Etape 4" };
bool etapesValidees[] = { false, false, false, false };


#if defined(IBT2)

int pwm_tondeuse(int canal_pwm, int broche_pwm, int freq_pwm)
{
  ledcAttachPin(broche_pwm, canal_pwm); // broche 18, canal 0.
  ledcSetup(canal_pwm, freq_pwm, 8); // canal = 0, frequence = 5000 Hz, resolution = 12 bits
  //ledcWrite(0, 2048);   //  canal = 0, rapport cyclique = 2048
  return (1);
}
#endif

#define DEBOUNCE_TIME 50 // temps de suppression de rebond en millisecondes

volatile int bumper_state = 0;
volatile unsigned long last_debounce_time = 0;
volatile unsigned long debounce_delay = DEBOUNCE_TIME;

void bumper_interrupt_handler(void) {
  unsigned long current_time = millis();

  if (current_time - last_debounce_time > debounce_delay) {
    bumper_state = !bumper_state;
  }
  last_debounce_time = current_time;
  //stop_mot();
#if defined(IBT2)
  ledcWrite(CANAL_MOT_AR_D, 0);
  ledcWrite(CANAL_MOT_AR_G, 0);
  ledcWrite(CANAL_MOT_AV_D, 0);
  ledcWrite(CANAL_MOT_AV_G, 0);
#endif
  //  if (digitalRead(PIN_COL_DROI) && digitalRead(PIN_COL_GAU)) bumper_state = 0;
  //  else if (digitalRead(PIN_COL_DROI)) bumper_state = 2;
  //  else if (digitalRead(PIN_COL_GAU)) bumper_state = 1;
}

void check_bump_init() {
  pinMode(PIN_COL_GAU, INPUT);
  pinMode(PIN_COL_DROI, INPUT);
  // attachInterrupt(digitalPinToInterrupt(PIN_COL_GAU), bumper_interrupt_handler, FALLING);
   //attachInterrupt(digitalPinToInterrupt(PIN_COL_DROI), bumper_interrupt_handler, FALLING);
}

int check_bump() {
  bumper_state = 0;
  if (!digitalRead(PIN_COL_DROI) && !digitalRead(PIN_COL_GAU)) bumper_state = 3;
  else if (!digitalRead(PIN_COL_DROI)) bumper_state = 2;
  else if (!digitalRead(PIN_COL_GAU)) bumper_state = 1;
  return bumper_state;
}
/*
int check_bump(int mode_bmp)
{
  int i = 0, j = 0;
  //     SerialBT.print(" check_bump mode=");
  //     SerialBT.println(mode_bmp);

  if (mode_bmp == -1) {
  //initialisation des bumpers
  pinMode(PIN_COL_GAU, INPUT);
  pinMode(PIN_COL_DROI, INPUT);
  return (0);
  }
  // prevoir le debounce peut etre via interrupt
  i = digitalRead(PIN_COL_DROI);
  j = digitalRead(PIN_COL_GAU);
  //     SerialBT.print("choc gauche=");
  //   SerialBT.print(j);
  //   SerialBT.print(" choc droit=");
  //   SerialBT.println(i);

  // retourne 1 par defaut et 0 si contact
  if (i && j)return (0);
  if (i) return (2);
  if (j) return (1);
  return (3);
}
*/

#if defined(WIFIOTA)
void startOTA() { // Start the OTA service
  ArduinoOTA.setHostname(OTAName);
  ArduinoOTA.setPassword(OTAPassword);

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
    });
  ArduinoOTA.onEnd([]() {
    Serial.println("\r\nEnd");
    });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
  ArduinoOTA.begin();
  Serial.println("OTA ready\r\n");
}
#endif


#if defined(VNH5019)

void analogWrite(int port, int canal, int val)
{
#if defined(BT_ENABLE)
  SerialBT.print("analog write port/canal :");
  SerialBT.print(port);
  SerialBT.print("/");
  SerialBT.print(canal);
  SerialBT.print(" valeur :");
  SerialBT.println(val);
#endif
  ledcAttachPin(port, canal); // broche 18, canal 0.
  ledcSetup(canal, 20000, 8); // canal = 0, frequence = 5000 Hz, resolution = 12 bits
  ledcWrite(canal, val);   //  canal = 0, rapport cyclique = 2048
}
void motor_init() {
  pinMode(PIN_INA1, OUTPUT);
  pinMode(PIN_INB1, OUTPUT);
  pinMode(PIN_PWM1, OUTPUT);
  pinMode(PIN_EN1DIAG1, INPUT);
  pinMode(PIN_CS1, INPUT);
  pinMode(PIN_INA2, OUTPUT);
  pinMode(PIN_INB2, OUTPUT);
  pinMode(PIN_PWM2, OUTPUT);
  pinMode(PIN_EN2DIAG2, INPUT);
  pinMode(PIN_CS2, INPUT);
}

// Set speed for motor 1, speed is a number betwenn -400 and 400
void setSpeed(int speed1, int speed2)
{
  unsigned char reverse = 0;

#if defined(BT_ENABLE)

  SerialBT.print("set speed :");
  SerialBT.print(speed1);
  SerialBT.print("/");
  SerialBT.println(speed2);
#endif
  if (speed1 < 0)
  {
    speed1 = -speed1;  // Make speed a positive quantity
    reverse = 1;  // Preserve the direction
  }
  if (speed1 > 400)speed1 = 400; // Max PWM dutycycle

  analogWrite(PIN_PWM1, 0, speed1 * 51 / 80); // map 400 to 255
  if (speed1 == 0)
  {
    digitalWrite(PIN_INA1, LOW);   // Make the motor coast no
    digitalWrite(PIN_INB1, LOW);   // matter which direction it is spinning.
  }
  else if (reverse)
  {
    digitalWrite(PIN_INA1, LOW);
    digitalWrite(PIN_INB1, HIGH);
  }
  else
  {
    digitalWrite(PIN_INA1, HIGH);
    digitalWrite(PIN_INB1, LOW);
  }
  if (speed2 < 0)
  {
    speed2 = -speed2;  // Make speed a positive quantity
    reverse = 1;  // Preserve the direction
  }
  if (speed2 > 400)speed2 = 400;  // Max PWM dutycycle

  analogWrite(PIN_PWM2, 1, speed2 * 51 / 80); // map 400 to 255
  if (speed2 == 0)
  {
    digitalWrite(PIN_INA2, LOW);   // Make the motor coast no
    digitalWrite(PIN_INB2, LOW);   // matter which direction it is spinning.
  }
  else if (reverse)
  {
    digitalWrite(PIN_INA2, LOW);
    digitalWrite(PIN_INB2, HIGH);
  }
  else
  {
    digitalWrite(PIN_INA2, HIGH);
    digitalWrite(PIN_INB2, LOW);
  }
}
#endif

void vitesse_lame(int valeur) {
  int remplissage = 0;
#if defined(IBT2)
  if (valeur > 100)valeur = 100;
#if defined(LCD1306)
  // Calculer les coordonnÃ©es du centre de l'Ã©cran
  int centerX = Mydisplay.getWidth() / 2;
  int centerY = Mydisplay.getHeight() / 2;
  // Tracer un cercle au centre de l'Ã©cran
  int radius = RAYON_LAME;
  Mydisplay.setColor(BLACK);
  Mydisplay.fillCircle(centerX, centerY, radius + 1);

  Mydisplay.display();
  Mydisplay.setColor(WHITE);

#endif
  for (int i = vitesse_actu; i <= valeur; i++) {
    Serial.print("Valeur Lame");
    Serial.println(i);
    ledcWrite(CANAL_LAME_AR, 0);
    ledcWrite(CANAL_LAME_AV, i * 2.55);
    vitesse_actu = i;
#if defined(LCD1306)
    remplissage = radius * (i / 100);
    Mydisplay.fillCircle(centerX, centerY, remplissage);
    Mydisplay.display();
#endif
    delay(50);
  }
  if (valeur == 0) {
    ledcWrite(CANAL_LAME_AV, 0);
    ledcWrite(CANAL_LAME_AR, 0);
    vitesse_actu = 0;
#if defined(LCD1306)
    // Calculer les coordonnÃ©es du centre de l'Ã©cran
    Mydisplay.setColor(BLACK);
    Mydisplay.fillCircle(centerX, centerY, radius);
    Mydisplay.setColor(WHITE);
    Mydisplay.drawCircle(centerX, centerY, radius);
    // Afficher le contenu de l'Ã©cran
    Mydisplay.display();
#endif

  }
  else {


  }

#else
  if (valeur > 50) {
    digitalWrite(PIN_MOT_LAME, HIGH);
  }
  else {
    digitalWrite(PIN_MOT_LAME, LOW);
  }
  //demarre la lame
#endif

}


void avance_mot(int valeur) {
#if defined(VNH5019)

  setSpeed(AVANT_G, AVANT_D);
#endif
#if defined(IBT2)
  if (valeur > 100)valeur = 100;
  ledcWrite(CANAL_MOT_AR_D, 0);
  ledcWrite(CANAL_MOT_AR_G, 0);
  ledcWrite(CANAL_MOT_AV_D, valeur * 2.55);
  ledcWrite(CANAL_MOT_AV_G, valeur * 2.55);
#endif
#if defined(LCD1306)
  // Tracer un rectangle plein Ã  gauche de l'Ã©cran
  Mydisplay.setColor(BLACK);

  Mydisplay.fillRect(DEBUT_RECT_DROIT, DEBUT_RECT_VERTICAL, LARGEUR_RECT, HAUTEUR_RECT);
  Mydisplay.fillRect(DEBUT_RECT_GAUCHE, DEBUT_RECT_VERTICAL, LARGEUR_RECT, HAUTEUR_RECT);
  Mydisplay.setColor(WHITE);
  //Mydisplay.drawProgressBar(DEBUT_RECT_DROIT, DEBUT_RECT_VERTICAL, LARGEUR_RECT, HAUTEUR_RECT,valeur);
  //Mydisplay.drawProgressBar(DEBUT_RECT_GAUCHE, DEBUT_RECT_VERTICAL, LARGEUR_RECT, HAUTEUR_RECT,valeur);

  Mydisplay.drawRect(DEBUT_RECT_DROIT, DEBUT_RECT_VERTICAL, LARGEUR_RECT, HAUTEUR_RECT);
  Mydisplay.drawRect(DEBUT_RECT_GAUCHE, DEBUT_RECT_VERTICAL, LARGEUR_RECT, HAUTEUR_RECT);
  Mydisplay.fillRect(DEBUT_RECT_DROIT, DEBUT_RECT_VERTICAL, LARGEUR_RECT, HAUTEUR_RECT);
  Mydisplay.fillRect(DEBUT_RECT_GAUCHE, DEBUT_RECT_VERTICAL, LARGEUR_RECT, HAUTEUR_RECT);
  // Afficher le contenu de l'Ã©cran
  Mydisplay.display();
#endif
#if defined(BT_ENABLE)

  SerialBT.println("AVANCE MOTEUR");
#endif
}


void stop_mot() {
#if defined(LCD1306)
  Mydisplay.drawString(30, 0, "STOP");
  Mydisplay.display();
#endif
#if defined(VNH5019)
  setSpeed(STOP, STOP);
#endif
#if defined(LCD1306)
  // Tracer un rectangle plein Ã  gauche de l'Ã©cran
  Mydisplay.setColor(BLACK);
  Mydisplay.fillRect(DEBUT_RECT_DROIT, DEBUT_RECT_VERTICAL, LARGEUR_RECT, HAUTEUR_RECT);
  Mydisplay.fillRect(DEBUT_RECT_GAUCHE, DEBUT_RECT_VERTICAL, LARGEUR_RECT, HAUTEUR_RECT);
  Mydisplay.setColor(WHITE);

  Mydisplay.drawRect(DEBUT_RECT_DROIT, DEBUT_RECT_VERTICAL, LARGEUR_RECT, HAUTEUR_RECT);
  Mydisplay.drawRect(DEBUT_RECT_GAUCHE, DEBUT_RECT_VERTICAL, LARGEUR_RECT, HAUTEUR_RECT);
  // Afficher le contenu de l'Ã©cran
  Mydisplay.display();
#endif

#if defined(IBT2)
  ledcWrite(CANAL_MOT_AR_D, 0);
  ledcWrite(CANAL_MOT_AR_G, 0);
  ledcWrite(CANAL_MOT_AV_D, 0);
  ledcWrite(CANAL_MOT_AV_G, 0);
#endif
}


void gauche_mot() {
#if defined(LCD1306)
  Mydisplay.drawString(30, 10, "GAUCHE");
  Mydisplay.display();
#endif
#if defined(VNH5019)
  setSpeed(AVANT_G, ARRIERE);
#endif
#if defined(IBT2)
  ledcWrite(CANAL_MOT_AR_G, 0);
  ledcWrite(CANAL_MOT_AV_D, 0);
  ledcWrite(CANAL_MOT_AR_D, 128);
  ledcWrite(CANAL_MOT_AV_G, 128);
#endif
#if defined(LCD1306)
  // Tracer un rectangle plein Ã  gauche de l'Ã©cran
  Mydisplay.setColor(BLACK);

  Mydisplay.fillRect(DEBUT_RECT_DROIT, DEBUT_RECT_VERTICAL, LARGEUR_RECT, HAUTEUR_RECT);
  Mydisplay.fillRect(DEBUT_RECT_GAUCHE, DEBUT_RECT_VERTICAL, LARGEUR_RECT, HAUTEUR_RECT);
  Mydisplay.setColor(WHITE);

  Mydisplay.fillRect(DEBUT_RECT_DROIT, DEBUT_RECT_VERTICAL, LARGEUR_RECT, HAUTEUR_RECT);
  Mydisplay.drawRect(DEBUT_RECT_GAUCHE, DEBUT_RECT_VERTICAL, LARGEUR_RECT, HAUTEUR_RECT);
  // Afficher le contenu de l'Ã©cran
  Mydisplay.display();
#endif

#if defined(BT_ENABLE)
  SerialBT.println("tourne a gauche");
#endif
}


void droite_mot() {
#if defined(LCD1306)
  Mydisplay.drawString(30, 20, "DROITE");
  Mydisplay.display();
#endif
#if defined(VNH5019)
  setSpeed(ARRIERE, AVANT_G);
#endif
#if defined(IBT2)
  ledcWrite(CANAL_MOT_AV_G, 0);
  ledcWrite(CANAL_MOT_AR_D, 0);
  ledcWrite(CANAL_MOT_AR_G, 128);
  ledcWrite(CANAL_MOT_AV_D, 128);
#endif
#if defined(LCD1306)
  // Tracer un rectangle plein Ã  gauche de l'Ã©cran
  Mydisplay.setColor(BLACK);
  Mydisplay.fillRect(DEBUT_RECT_DROIT, DEBUT_RECT_VERTICAL, LARGEUR_RECT, HAUTEUR_RECT);
  Mydisplay.fillRect(DEBUT_RECT_GAUCHE, DEBUT_RECT_VERTICAL, LARGEUR_RECT, HAUTEUR_RECT);
  Mydisplay.setColor(WHITE);
  Mydisplay.drawRect(DEBUT_RECT_DROIT, DEBUT_RECT_VERTICAL, LARGEUR_RECT, HAUTEUR_RECT);
  Mydisplay.fillRect(DEBUT_RECT_GAUCHE, DEBUT_RECT_VERTICAL, LARGEUR_RECT, HAUTEUR_RECT);
  // Afficher le contenu de l'Ã©cran
  Mydisplay.display();
#endif
#if defined(BT_ENABLE)

  SerialBT.println("tourne a droite");
#endif
}


void arriere_mot() {
#if defined(LCD1306)
  Mydisplay.drawString(30, 30, "ARRIERE");
  Mydisplay.display();
#endif
#if defined(LCD1306)
  // Tracer un rectangle plein Ã  gauche de l'Ã©cran
  Mydisplay.setColor(BLACK);
  Mydisplay.fillRect(DEBUT_RECT_DROIT, DEBUT_RECT_VERTICAL, LARGEUR_RECT, HAUTEUR_RECT);
  Mydisplay.fillRect(DEBUT_RECT_GAUCHE, DEBUT_RECT_VERTICAL, LARGEUR_RECT, HAUTEUR_RECT);
  Mydisplay.setColor(WHITE);
  Mydisplay.fillRect(DEBUT_RECT_DROIT, DEBUT_RECT_VERTICAL, LARGEUR_RECT, HAUTEUR_RECT);
  Mydisplay.fillRect(DEBUT_RECT_GAUCHE, DEBUT_RECT_VERTICAL, LARGEUR_RECT, HAUTEUR_RECT);
  // Afficher le contenu de l'Ã©cran
  Mydisplay.display();
#endif
#if defined(VNH5019)
  setSpeed(ARRIERE, ARRIERE);
#endif
#if defined(IBT2)
  ledcWrite(CANAL_MOT_AV_D, 0);
  ledcWrite(CANAL_MOT_AV_G, 0);
  ledcWrite(CANAL_MOT_AR_D, 128);
  ledcWrite(CANAL_MOT_AR_G, 128);
#endif
#if defined(BT_ENABLE)

  SerialBT.println("recule");
#endif
}

long calcul_distance_univ(byte capteur)
{
  switch (capteur) {
  case CAPTEUR_GAUCHE:// capteur gauche
    pin_trig = PIN_TRIG_GAU;
    pin_echo = PIN_ECHO_GAU;
    break;
  case CAPTEUR_CENTRE:// capteur central
    pin_trig = PIN_TRIG_CENT;
    pin_echo = PIN_ECHO_CENT;
    break;
  case CAPTEUR_DROITE://capteur droite
    pin_trig = PIN_TRIG_DROI;
    pin_echo = PIN_ECHO_DROI;
    break;
  case CAPTEUR_SOL://capteur sol
    pin_trig = PIN_TRIG_SOL;
    pin_echo = PIN_ECHO_SOL;
    break;

  }
#if defined(BT_ENABLE)
  SerialBT.print("PIN TRIG :");
  SerialBT.print(pin_trig);
  SerialBT.print(" PIN ECHO :");
  SerialBT.print(pin_echo);
#endif

  digitalWrite(pin_trig, LOW);
  delayMicroseconds(5);
  digitalWrite(pin_trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin_trig, LOW);
  echo = pulseIn(pin_echo, HIGH);

#if defined(BT_ENABLE)
  SerialBT.print(" echo :");
  SerialBT.print(echo);
#endif

  echo = echo * 0.034 / 2; //58

#if defined(BT_ENABLE)
  SerialBT.print(" longueur-->");
  SerialBT.println(echo);
#endif
  return (echo);
}

void calcul_distance()
{
  long_cent = calcul_distance_univ(CAPTEUR_CENTRE);
  long_gau = calcul_distance_univ(CAPTEUR_GAUCHE);
  long_droi = calcul_distance_univ(CAPTEUR_DROITE);
#if defined(US_SOL)
  long_sol = calcul_distance_univ(CAPTEUR_SOL);
#endif
  Serial.print("distance mesurÃ©es  : "); Serial.print(long_gau); Serial.print("   "); Serial.print(long_cent); Serial.print("   "); Serial.println(long_droi);

#if defined(BT_ENABLE)
  SerialBT.print("distance mesurÃ©es  : "); SerialBT.print(long_gau); SerialBT.print(" / "); SerialBT.print(long_cent); SerialBT.print(" / "); SerialBT.println(long_droi);
#if defined(US_SOL)
  SerialBT.print("distance sol  : "); SerialBT.println(long_sol);
#endif
#endif
}

// on verifie autour
int verif_autour(int mode_mesure) {
  // on verifie autour
  // plusieurs mode 
  // devant uniquement
  // a droite uniquement
  // a gauche uniquement
  // devant et si besoin a droite et a gauche
   //calcul_distance();
   //
   /*

   Bits 76543210
         ^--->bit capteur sol
        ^---->bit capteur droit
       ^----->bit capteur centre
      ^------>bit capteur gauche
       ^------->bit non utilisé
      ^-------->bit non utilisé
     ^--------->bit non utilisé
     ^---------->bit non utilisé
   */
  int monmode;

  // si mode_mesure a le bit de poids faible a 1 uniquement alors on calcul la distance au centre et on retourne la distance
  long_cent = calcul_distance_univ(CAPTEUR_CENTRE);

  long_gau = calcul_distance_univ(CAPTEUR_GAUCHE);
  long_droi = calcul_distance_univ(CAPTEUR_DROITE);



  Serial.print("distance mesurees  : "); SerialBT.print(long_gau); SerialBT.print(" / "); SerialBT.print(long_cent); SerialBT.print(" / "); SerialBT.println(long_droi);

#if defined(BT_ENABLE)
  SerialBT.print("distance mesurees  : "); SerialBT.print(long_gau); SerialBT.print(" / "); SerialBT.print(long_cent); SerialBT.print(" / "); SerialBT.println(long_droi);
#endif
return(0);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("setup start");
  // Initialising the UI will init the display too.
#if defined(LCD1306)
  Wire.begin();
  Mydisplay.init();
  Mydisplay.clear();
  Mydisplay.flipScreenVertically();
  Mydisplay.setFont(ArialMT_Plain_10);
  Mydisplay.drawString(0, 0, "SETUP LCD OK");
  Mydisplay.display();
  /*
    for (int i = 0; i < sizeof(etapes) / sizeof(etapes[0]); i++) {
    Mydisplay.drawString(etapes[i], 0, i * 10);
    if (etapesValidees[i]) {
    Mydisplay.drawCircle(120, i * 10 + 4, 4, WHITE);
    } else {
    Mydisplay.drawCircle(120, i * 10 + 4, 4, BLACK);
    }
  }
  Mydisplay.display();
  */
#endif

#if defined(BT_ENABLE)
  SerialBT.begin("Tondeuse"); //Bluetooth device name
  Serial.println();
  Serial.println("The device started, now you can pair it with bluetooth!");
#if defined(LCD1306)
  Mydisplay.drawString(0, 10, "BT OK");
  Mydisplay.display();
#endif
#endif


  //les capteurs us
  pinMode(PIN_TRIG_CENT, OUTPUT);
  digitalWrite(PIN_TRIG_CENT, LOW);
  pinMode(PIN_ECHO_CENT, INPUT);

  pinMode(PIN_TRIG_GAU, OUTPUT);
  digitalWrite(PIN_TRIG_GAU, LOW);
  pinMode(PIN_ECHO_GAU, INPUT);

  pinMode(PIN_TRIG_DROI, OUTPUT);
  digitalWrite(PIN_TRIG_DROI, LOW);
  pinMode(PIN_ECHO_DROI, INPUT);

#if defined(US_SOL)
  pinMode(PIN_TRIG_SOL, OUTPUT);
  digitalWrite(PIN_TRIG_SOL, LOW);
  pinMode(PIN_ECHO_SOL, INPUT);
#endif

  // les capteurs de chocs
  //  pinMode(PIN_COL_GAU, INPUT);
  //  pinMode(PIN_COL_DROI, INPUT);
   // check_bump(-1);
  check_bump_init();
#if defined(LCD1306)
  Mydisplay.drawString(0, 20, "BUMPER OK");
  Mydisplay.display();
#endif

  //le relais de lame

#if defined(LAME_IBT2)
  pwm_tondeuse(CANAL_LAME_AV, PIN_PWM_LAME_N, FREQ_PWM);
  pwm_tondeuse(CANAL_LAME_AR, PIN_PWM_LAME_R, FREQ_PWM);
#if defined(LCD1306)
  Mydisplay.drawString(0, 30, "LAME OK");
  Mydisplay.display();
#endif

#else
  pinMode(PIN_MOT_LAME, OUTPUT);
  digitalWrite(PIN_MOT_LAME, LOW);
#endif


#if defined(IBT2)

  //#define PIN_PWM_MOT1_N  25
  //#define PIN_PWM_MOT1_R  27
  //#define PIN_PWM_MOT2_N  32
  //#define PIN_PWM_MOT2_R  33
  //#define CANAL_MOT1 1
  //#define CANAL_MOT2 2
  //#define CANAL_LAME 3
  //#define FREQ_PWM 5000
  pwm_tondeuse(CANAL_MOT_AV_G, PIN_PWM_MOT1_N, FREQ_PWM);
  pwm_tondeuse(CANAL_MOT_AR_G, PIN_PWM_MOT1_R, FREQ_PWM);

  pwm_tondeuse(CANAL_MOT_AV_D, PIN_PWM_MOT2_N, FREQ_PWM);
  pwm_tondeuse(CANAL_MOT_AR_D, PIN_PWM_MOT2_R, FREQ_PWM);
#if defined(LCD1306)
  Mydisplay.drawString(0, 40, "MOTEUR OK");
  Mydisplay.display();
#endif

#endif
#if defined(WIFIOTA)

  for (uint8_t t = 4; t > 0; t--) {
    Serial.printf("[SETUP] WAIT %d...\n", t);
    Serial.flush();
    delay(1000);
  }

  WiFi.mode(WIFI_STA);
  WiFiMulti.addAP("SFR_6190", "catmaruvtofabricnak4");
  WiFiMulti.addAP("Nokia 6.1", "Voyageur1974");
  WiFiMulti.addAP("SFR_6190_5GEXT", "catmaruvtofabricnak4");
#if defined(LCD1306)
  Mydisplay.drawString(0, 50, "WIFI OTA OK");
  Mydisplay.display();
#endif

#endif
#if defined(LCD1306)
  delay(2000);
  Mydisplay.clear();
  Mydisplay.setTextAlignment(TEXT_ALIGN_LEFT);
  Mydisplay.setFont(ArialMT_Plain_10);
#endif
}
String inString = "";
String saisie = "";
int mon_mode = MODE_TONDEUSE;
bool mode_simu = 0;

void loop() {
  //display.clear();
  Serial.println("debut boucle");
  int statut_choc = -1;
  statut_choc = check_bump();
  if (statut_choc > 0)stop_mot();

#if defined(BT_ENABLE)
  SerialBT.print(" statut choc");
  SerialBT.println(statut_choc);
#endif
  // on verifie autour
  //calcul_distance();
  long_cent = calcul_distance_univ(CAPTEUR_CENTRE);
  Serial.print("Distance devant:");
  Serial.println(long_cent);
#if defined(LCD1306)
  Mydisplay.drawString(42, 30, "        ");
  Mydisplay.display();
  Mydisplay.drawString(42, 30, String(long_cent));
  Mydisplay.display();
#endif

  if (long_cent < VAL_WARN_CENT) {
    long_cent = calcul_distance_univ(CAPTEUR_CENTRE);
    if (long_cent < VAL_WARN_CENT) {
      stop_mot();
      long_gau = calcul_distance_univ(CAPTEUR_GAUCHE);
      long_droi = calcul_distance_univ(CAPTEUR_DROITE);
#if defined(BT_ENABLE)
      SerialBT.print("distance mesurÃ©es  : "); SerialBT.print(long_gau); SerialBT.print(" / "); SerialBT.print(long_cent); SerialBT.print(" / "); SerialBT.println(long_droi);
#endif
      if (long_gau < long_droi) {
        // on part a droite
#if defined(BT_ENABLE)
        SerialBT.println(" on va a droite");
#endif
        droite_mot();
        delay(DUREE_ROT);// TO DO: modifier delay pour ne pas bloquer l'ESP
      }
      else {
        // on part a gauche
#if defined(BT_ENABLE)
        SerialBT.println(" on va a gauche");
#endif
        gauche_mot();
        delay(DUREE_ROT);
      }
    }
  }
  //}
#if defined(US_SOL)
  long_sol = calcul_distance_univ(CAPTEUR_SOL);
  Serial.print("long_sol:");
  Serial.println(long_sol);
  if (long_sol > VAL_WARN_SOL) {
    vitesse_lame(0);
    stop_mot();
    mon_mode = MODE_ARRET;
  }
#endif
  if (Serial.available() > 0) {
    saisie = Serial.readStringUntil('\n');
  }
#if defined(BT_ENABLE)
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {
    Serial.println("reception bt serie");
    saisie = SerialBT.readStringUntil('\n');
    Serial.println(saisie);
  }
#endif
  delay(2);
  Serial.print("saisie:"); Serial.println(saisie);
  if (saisie != "") {
    if (saisie.indexOf("stop") >= 0) {
      mon_mode = MODE_ARRET;
#if defined(BT_ENABLE)
      SerialBT.println("STOP");
#endif
    }
    if (saisie.indexOf("D90") >= 0) {
      droite_mot();
      delay(1000);
      stop_mot();
    }
    if (saisie.indexOf("G90") >= 0) {
      gauche_mot();
      delay(1000);
      stop_mot();
    }
    if (saisie.indexOf("arret") >= 0) {
      stop_mot();
    }
    if (saisie.indexOf("simu") >= 0) {
      mon_mode = MODE_SIMU;
    }
    if (saisie.indexOf("tond") >= 0) {
      mon_mode = MODE_TONDEUSE;
    }
    if (saisie.indexOf("version") >= 0) {
#if defined(BT_ENABLE)
      SerialBT.println("version 2");
      delay(5000);
#endif

    }
    if (saisie.indexOf("avance") >= 0) {
      mon_mode = MODE_AVANCE;
    }
    if (saisie.indexOf("arriere") >= 0) {
      mon_mode = MODE_ARRIERE;
    }
    if (saisie.indexOf("droite") >= 0) {
      mon_mode = MODE_DROITE;
    }
    if (saisie.indexOf("gauche") >= 0) {
      mon_mode = MODE_GAUCHE;
    }
    if (saisie.indexOf("distance") >= 0) {
      mon_mode = MODE_DISTANCE;
    }
    if (saisie.indexOf("update") >= 0) {
      mon_mode = MODE_UPDATE;
    }
    saisie = "";
  }
  else {
    // pas de saisie
  }
  Serial.print("mon_mode:");
  Serial.println(mon_mode);
#if defined(BT_ENABLE)
  SerialBT.print("mode: ");
  SerialBT.println(mon_mode);
#endif

  switch (mon_mode) {
  case MODE_ARRET:    // mode arret
    Serial.println("stop");
    digitalWrite(PIN_MOT_LAME, LOW);
    vitesse_lame(0);
    //display.drawString(40, 30, "O");
    stop_mot();
    mode_simu = 0;
    break;
  case MODE_SIMU:    // mode avance
    Serial.println("simulation");
    mode_simu = 1;
  case MODE_TONDEUSE:    // mode tondeuse
    Serial.println("tondeuse");

#if defined(US_SOL)
    long_sol = calcul_distance_univ(CAPTEUR_SOL);
    Serial.print("Dist_sol:");
    Serial.println(long_sol);
    if (long_sol > VAL_WARN_SOL) {
      // on teste pour savoir si on est hors sol
      vitesse_lame(0);
      stop_mot();
      mon_mode = MODE_ARRET;
      //on passe en mode simu pour ne pas faire tourner la lame
       mode_simu = 1;
    }
#endif

    //test des capteurs de chocs
    statut_choc = check_bump();
    if (statut_choc > 0) {
      //les capteurs de chocs sont activé
      stop_mot();
#if defined(BT_ENABLE)
      SerialBT.print(" statut choc");
      SerialBT.println(statut_choc);
#endif
    }
    else {
      // les capteurs de chocs ne sont pas activés
      if (!mode_simu)vitesse_lame(100);

    }
    // on verifie autour
    //calcul_distance();
    //long_cent = calcul_distance_univ(2);

    //if (long_cent < VAL_WARN_CENT) {
    long_cent = calcul_distance_univ(CAPTEUR_CENTRE);
    if (long_cent < VAL_WARN_CENT) {
      stop_mot();
      long_gau = calcul_distance_univ(CAPTEUR_GAUCHE);
      long_droi = calcul_distance_univ(CAPTEUR_DROITE);
#if defined(BT_ENABLE)
      SerialBT.print("distance mesurÃ©es  : "); SerialBT.print(long_gau); SerialBT.print(" / "); SerialBT.print(long_cent); SerialBT.print(" / "); SerialBT.println(long_droi);
#endif
      if (long_gau < long_droi) {
        // on part a droite
#if defined(BT_ENABLE)
        SerialBT.println(" on va a droite");
#endif
        droite_mot();
        delay(DUREE_ROT);
      }
      else {
        // on part a gauche
#if defined(BT_ENABLE)
        SerialBT.println(" on va a gauche");
#endif
        gauche_mot();
        delay(DUREE_ROT);
      }
    }else{
      avance_mot(100);
    }
    //}
    //}
    //avance_mot(100);

    break;
  case MODE_ARRIERE:    // mode reduit
    Serial.println("arriere");
    arriere_mot();
    break;
  case MODE_DROITE:    // mode reduit
    Serial.println("a droite");
    droite_mot();
    break;
  case MODE_DISTANCE:    // verifie la distance
    calcul_distance();
    delay(2000);
    break;
  case MODE_GAUCHE:    // mode reduit
    Serial.println("a gauche");
    gauche_mot();
    break;
  case MODE_AVANCE:    // mode avance
    Serial.println("avance");
    avance_mot(100);
    break;
  case MODE_UPDATE:    // mode mise a jour firmware
#if defined(WIFIOTA)
    Serial.println("mise a jour");
#if defined(BT_ENABLE)
    SerialBT.print("Mise a jour ");
    //SerialBT.println(mon_mode);
#endif
    Serial.println("stop");
    digitalWrite(PIN_MOT_LAME, LOW);
    vitesse_lame(0);
    //display.drawString(40, 30, "O");
    stop_mot();


    if ((WiFiMulti.run() != WL_CONNECTED)) {
      startOTA();                  // Start the OTA service
      //initOTA();

    }

    // wait for WiFi connection
    if ((WiFiMulti.run() == WL_CONNECTED)) {
      /*
        WiFiClient client;
        //httpUpdate.setLedPin(LED_BUILTIN, LOW);
        t_httpUpdate_return ret = httpUpdate.update(client, "http://partage.local/Documents/updates/tondeuse/tondeuse.bin");
        //               t_httpUpdate_return ret = httpUpdate.update(client, "\\PARTAGE-LOCAL\Documents\updates\tondeuse\tondeuse.bin");
        // Or:
        //t_httpUpdate_return ret = httpUpdate.update(client, "server", 80, "/file.bin");
        switch (ret) {
        case HTTP_UPDATE_FAILED:
          Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
        #if defined(BT_ENABLE)
        SerialBT.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
        //SerialBT.println(mon_mode);
        #endif
          break;
        case HTTP_UPDATE_NO_UPDATES:
        #if defined(BT_ENABLE)
        SerialBT.println("pas de mise a jour");
        //SerialBT.println(mon_mode);
        #endif
          break;
        case HTTP_UPDATE_OK:
        #if defined(BT_ENABLE)
        SerialBT.println("mise a jour OK");
        //SerialBT.println(mon_mode);
        #endif
          break;
      */
      ArduinoOTA.handle();                        // listen for OTA events
    }
    //delay(10000);
    //mon_mode=MODE_ARRET;
    // }
#else
    Serial.println("mise a jour pas disponible");
    mon_mode = MODE_ARRET;
#endif
    break;
  }

  //mode_tondeuse();


}

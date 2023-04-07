#define LCD1306
#define IBT2
#define LAME_IBT2
#define TIMER
#define WIFIOTA
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
#else

#endif
#define FREQ_PWM 5000


#if defined(US_SOL)
#define PIN_TRIG_SOL 5
#define PIN_ECHO_SOL 23
#define VAL_WARN_SOL 6
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

#if defined(LCD1306)
// if you have an LCD screen
#include <Wire.h>
#include "SSD1306Wire.h"        // legacy: #include "SSD1306.h"
SSD1306Wire display(0x3c, SDA, SCL);   // ADDRESS, SDA, SCL  -  SDA and SCL usually populate automatically based on your board's pins_arduino.h
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
volatile int bumper_state = 0;

int vitesse_actu = 0;

#if defined(IBT2)

int pwm_tondeuse(int canal_pwm, int broche_pwm, int freq_pwm)
{
  ledcAttachPin(broche_pwm, canal_pwm); // broche 18, canal 0.
  ledcSetup(canal_pwm, freq_pwm, 8); // canal = 0, frequence = 5000 Hz, resolution = 12 bits
  //ledcWrite(0, 2048);   //  canal = 0, rapport cyclique = 2048
  return (1);
}
#endif


/*
int check_bump(int mode_bmp)
{
  int i = 0, j = 0;

  // If mode_bmp is -1, initialize the bumper pins and return 0
  if (mode_bmp == -1) {
    pinMode(PIN_COL_GAU, INPUT);
    pinMode(PIN_COL_DROI, INPUT);
    return (0);
  }

  // Read the state of the bumper pins and store in variables i and j
  i = digitalRead(PIN_COL_DROI);
  j = digitalRead(PIN_COL_GAU);

  // Check the state of the bumper pins and return appropriate value
  // 0 indicates a collision, 1 indicates a collision on the left bumper,
  // 2 indicates a collision on the right bumper, and 3 indicates no collision
  if (i && j) return (0);
  if (i) return (2);
  if (j) return (1);
  return (3);
}
*/


void bumper_interrupt_handler(void) {
  int i = digitalRead(PIN_COL_DROI);
  int j = digitalRead(PIN_COL_GAU);

  if (i && j) bumper_state = 0;
  else if (i) bumper_state = 2;
  else if (j) bumper_state = 1;
}

void check_bump_init() {
  pinMode(PIN_COL_GAU, INPUT);
  pinMode(PIN_COL_DROI, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_COL_GAU), bumper_interrupt_handler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_COL_DROI), bumper_interrupt_handler, CHANGE);
}

int check_bump() {
  return bumper_state;
}


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


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("setup start");
  // Initialising the UI will init the display too.
#if defined(LCD1306)
  Wire.begin();
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "SETUP LCD OK");
  display.display();
#endif

#if defined(BT_ENABLE)
  SerialBT.begin("Tondeuse"); //Bluetooth device name
#endif

  Serial.println();
  Serial.println("The device started, now you can pair it with bluetooth!");
#if defined(LCD1306)
  display.drawString(0, 10, "BT OK");
  display.display();
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
  //check_bump(-1);
  check_bump_init();

#if defined(LCD1306)
  display.drawString(0, 20, "BUMPER OK");
  display.display();
#endif

  //le relais de lame

#if defined(LAME_IBT2)
  pwm_tondeuse(CANAL_LAME_AV, PIN_PWM_LAME_N, FREQ_PWM);
  pwm_tondeuse(CANAL_LAME_AR, PIN_PWM_LAME_R, FREQ_PWM);
#if defined(LCD1306)
  display.drawString(0, 30, "LAME OK");
  display.display();
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
  display.drawString(0, 40, "MOTEUR OK");
  display.display();
#endif

#endif
#if defined(WIFIOTA)

  for (uint8_t t = 4; t > 0; t--) {
    Serial.printf("[SETUP] WAIT %d...\n", t);
    Serial.flush();
    delay(1000);
  }

  WiFi.mode(WIFI_STA);
  WiFiMulti.addAP("MyPhone", "MyPWD");
 
#endif
#if defined(LCD1306)
  display.drawString(0, 50, "WIFI OK");
  display.display();
#endif
}


void vitesse_lame(int valeur) {
#if defined(IBT2)
  if (valeur > 100)valeur = 100;
  for (int i = vitesse_actu; i <= valeur; i++) {
    Serial.print("Valeur Lame");
    Serial.println(i);
    ledcWrite(CANAL_LAME_AR, 0);
    ledcWrite(CANAL_LAME_AV, i * 2.55);
    vitesse_actu = i;
    delay(50);
  }
  if (valeur == 0) {
    ledcWrite(CANAL_LAME_AV, 0);
    ledcWrite(CANAL_LAME_AR, 0);
    vitesse_actu = 0;
  }

#else
  if (valeur > 50) {
    digitalWrite(PIN_MOT_LAME, HIGH);
  } else {
    digitalWrite(PIN_MOT_LAME, LOW);
  }
  //demarre la lame
#endif

}


void avance_mot(int valeur) {
#if defined(IBT2)
  if (valeur > 100)valeur = 100;
  ledcWrite(CANAL_MOT_AR_D, 0);
  ledcWrite(CANAL_MOT_AR_G, 0);
  ledcWrite(CANAL_MOT_AV_D, valeur * 2.55);
  ledcWrite(CANAL_MOT_AV_G, valeur * 2.55);
#endif
#if defined(BT_ENABLE)

  SerialBT.println("AVANCE MOTEUR");
#endif
}


void stop_mot() {
#if defined(LCD1306)
  display.drawString(30, 0, "STOP");
  display.display();
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
  display.drawString(30, 10, "GAUCHE");
  display.display();
#endif
#if defined(IBT2)
  ledcWrite(CANAL_MOT_AR_G, 0);
  ledcWrite(CANAL_MOT_AV_D, 0);
  ledcWrite(CANAL_MOT_AR_D, 128);
  ledcWrite(CANAL_MOT_AV_G, 128);
#endif
#if defined(BT_ENABLE)
  SerialBT.println("tourne a gauche");
#endif
}


void droite_mot() {
#if defined(LCD1306)
  display.drawString(30, 20, "DROITE");
  display.display();
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
#if defined(BT_ENABLE)

  SerialBT.println("tourne a droite");
#endif
}


void arriere_mot() {
#if defined(LCD1306)
  display.drawString(30, 20, "ARRIERE");
  display.display();
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

long calcul_distance_univ(int capteur)
{
  int pin_trig, pin_echo;
  long echo = 0;
  switch (capteur) {
    case 1:// capteur gauche
      pin_trig = PIN_TRIG_GAU;
      pin_echo = PIN_ECHO_GAU;
      break;
    case 2:// capteur central
      pin_trig = PIN_TRIG_CENT;
      pin_echo = PIN_ECHO_CENT;
      break;
    case 3://capteur droite
      pin_trig = PIN_TRIG_DROI;
      pin_echo = PIN_ECHO_DROI;
      break;
    case 4://capteur sol
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
  long_cent = calcul_distance_univ(2);
  long_gau = calcul_distance_univ(1);
  long_droi = calcul_distance_univ(3);
#if defined(US_SOL)
  long_sol = calcul_distance_univ(4);
#endif
  Serial.print("distance mesurées  : "); Serial.print(long_gau ); Serial.print("   "); Serial.print(long_cent ); Serial.print("   "); Serial.println(long_droi );

#if defined(BT_ENABLE)
  SerialBT.print("distance mesurées  : "); SerialBT.print(long_gau ); SerialBT.print(" / "); SerialBT.print(long_cent ); SerialBT.print(" / "); SerialBT.println(long_droi );
#if defined(US_SOL)
  SerialBT.print("distance sol  : "); SerialBT.println(long_sol );
#endif
#endif
}
/*
  void evite_obstacle(){
    long_gau = calcul_distance_univ(1);
    long_droi = calcul_distance_univ(3);

  #if defined(BT_ENABLE)
        SerialBT.print(" WARNING sup :");
        SerialBT.println(warn_cent);
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
    }

  }

  void mode_tondeuse(int lame)
  {
    int statut_choc = -1;
    statut_choc = check_bump(0);

    if (lame) {
        vitesse_lame(100);
        //digitalWrite(PIN_MOT_LAME, HIGH);
        //display.drawString(40, 30, "X");
    }
    else {
        vitesse_lame(0);
        //digitalWrite(PIN_MOT_LAME, LOW);
        //display.drawString(40, 30, "O");
    }

    // on verifie autour
    //calcul_distance();
    long_cent = calcul_distance_univ(2);
    //long_gau = calcul_distance_univ(1);
    //long_droi = calcul_distance_univ(3);
  #if defined(US_SOL)
    long_sol = calcul_distance_univ(4);
  #endif

  #if defined(BT_ENABLE)
    SerialBT.print("distance mesurées  : "); SerialBT.print(long_gau); SerialBT.print(" / "); SerialBT.print(long_cent); SerialBT.print(" / "); SerialBT.println(long_droi);
  #endif

    if (statut_choc > 0) {

  #if defined(BT_ENABLE)

        SerialBT.print(" statut choc");
        SerialBT.println(statut_choc);
  #endif
        long_cent = 0;
        statut_choc = 0;
        warn_cent = 3;
    }
    if (long_sol > VAL_WARN_SOL) {
        vitesse_lame(0);
        warn_cent = 3;}

    if (long_cent < VAL_WARN_CENT) {
        warn_cent++;

  #if defined(BT_ENABLE)

        SerialBT.print(" WARNING :");
        SerialBT.println(warn_cent);
  #endif
    }

    if (warn_cent > 3) {
        warn_cent = 0;
    //display.clear();
            stop_mot();
  evite_obstacle();
    avance_mot(100);
  }

*/

String inString = "";
String saisie = "";
int mon_mode = 0;
void loop() {
  int statut_choc = -1;
  //display.clear();
  Serial.println("debut boucle");
  statut_choc = check_bump(0);
  if (statut_choc > 0)stop_mot();
#if defined(BT_ENABLE)
  SerialBT.print(" statut choc");
  SerialBT.println(statut_choc);
#endif
  // on verifie autour
  //calcul_distance();
  long_cent = calcul_distance_univ(2);

  if (long_cent < VAL_WARN_CENT) {
    long_cent = calcul_distance_univ(2);
    if (long_cent < VAL_WARN_CENT) {
      stop_mot();
      long_gau = calcul_distance_univ(1);
      long_droi = calcul_distance_univ(3);
#if defined(BT_ENABLE)
      SerialBT.print("distance mesurées  : "); SerialBT.print(long_gau); SerialBT.print(" / "); SerialBT.print(long_cent); SerialBT.print(" / "); SerialBT.println(long_droi);
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
    }
  }
  //}
#if defined(US_SOL)
  long_sol = calcul_distance_univ(4);
  if (long_sol > VAL_WARN_SOL) {
    vitesse_lame(0);
    stop_mot();
    mon_mode = MODE_ARRET;
  }
#endif
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
#if defined(LCD1306)
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
#endif
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
  }
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
      break;
    case MODE_TONDEUSE:    // mode tondeuse
      Serial.println("tondeuse");
      vitesse_lame(100);
      statut_choc = check_bump(0);
      if (statut_choc > 0)stop_mot();
#if defined(BT_ENABLE)
      SerialBT.print(" statut choc");
      SerialBT.println(statut_choc);
#endif
      // on verifie autour
      //calcul_distance();
      long_cent = calcul_distance_univ(2);

      if (long_cent < VAL_WARN_CENT) {
        long_cent = calcul_distance_univ(2);
        if (long_cent < VAL_WARN_CENT) {
          stop_mot();
          long_gau = calcul_distance_univ(1);
          long_droi = calcul_distance_univ(3);
#if defined(BT_ENABLE)
          SerialBT.print("distance mesurées  : "); SerialBT.print(long_gau); SerialBT.print(" / "); SerialBT.print(long_cent); SerialBT.print(" / "); SerialBT.println(long_droi);
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
        }
      }
      //}
#if defined(US_SOL)
      long_sol = calcul_distance_univ(4);
      if (long_sol > VAL_WARN_SOL) {
        vitesse_lame(0);
        stop_mot();
        mon_mode = MODE_ARRET;
      }
#endif
      break;
    case MODE_SIMU:    // mode avance
      Serial.println("simulation");
      //mode_tondeuse(0);
      vitesse_lame(0);
      statut_choc = check_bump(0);
      if (statut_choc > 0)stop_mot();
#if defined(BT_ENABLE)
      SerialBT.print(" statut choc");
      SerialBT.println(statut_choc);
#endif
      // on verifie autour
      //calcul_distance();
      long_cent = calcul_distance_univ(2);

      if (long_cent < VAL_WARN_CENT) {
        long_cent = calcul_distance_univ(2);
        if (long_cent < VAL_WARN_CENT) {
          stop_mot();
          long_gau = calcul_distance_univ(1);
          long_droi = calcul_distance_univ(3);
#if defined(BT_ENABLE)
          SerialBT.print("distance mesurées  : "); SerialBT.print(long_gau); SerialBT.print(" / "); SerialBT.print(long_cent); SerialBT.print(" / "); SerialBT.println(long_droi);
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
        }
      }
      //}
#if defined(US_SOL)
      long_sol = calcul_distance_univ(4);
      if (long_sol > VAL_WARN_SOL) {
        vitesse_lame(0);
        stop_mot();
        mon_mode = MODE_ARRET;
      }
#endif
      //      avance_mot();
      break;
    case MODE_ARRIERE:    // mode arriere
      Serial.println("arriere");
      arriere_mot();
      break;
    case MODE_DROITE:    // mode droite
      Serial.println("a droite");
      droite_mot();
      break;
    case MODE_DISTANCE:    // verifie la distance
      calcul_distance();
      delay(2000);
      break;
    case MODE_GAUCHE:    // mode gauche
      Serial.println("a gauche");
      gauche_mot();
      break;
    case MODE_AVANCE:    // mode avance
      Serial.println("avance");
      avance_mot(100);
      break;
    case MODE_UPDATE:    // mode mise a jour firmware
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

#if defined(WIFIOTA)
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

#endif
      break;
  }

  //mode_tondeuse();


}
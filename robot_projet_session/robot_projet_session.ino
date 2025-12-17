#include <MeAuriga.h>
#include <Adafruit_seesaw.h>
#include <ArduinoJson.h>

#define NB_IR 5
#define PULSE 9
#define RATIO 39.267
#define CIRC_WHEEL 202.6
#define BUZZER_PIN 45

#define ALL_LEDS 0
#define LEDNUM  12 // Auriga on-board light ring has 12 LEDs
#define LED_PIN 44

struct Capteur{
  int  valeurMin = 1023;
  int valeurMax = 0;
  int valeurLue;
  int valeurNormalisee;
  int seuil;
};

Capteur capteurs[NB_IR];

MeGyro gyro(0, 0x69);
MeUltrasonicSensor ultraSensor(PORT_10);
MeRGBLed led(0, LEDNUM);

Adafruit_seesaw ss;

MeEncoderOnBoard encoderRight(SLOT1);
MeEncoderOnBoard encoderLeft(SLOT2);

float distance = 0;
const int lastValue = 400;
int distanceObjet = 30;

short vitesse = 100;
short vitessePivot = 100;
short vitesseVig = 0.7 * vitesse;
short vitesseRecul = 100;

unsigned long currentTime = 0;
unsigned long chrono = 0;
unsigned long initial = 0;

short maxAngle = 360;
short semiCirc = 180;

int tolerance = 12; 
short ecart = 10;

float consigne = 0.0f;
float adjustment = 0;
float position;

short red = 0;
short blue = 0;
short green = 0;

bool onGreen = 0;
unsigned long exitGreen = 0;

enum Mode {AUTO, MANUAL, ARRET, CALIBRATION};
Mode mode = ARRET;

enum StateAuto {DEMARRAGE, SEARCH, CHECKPOINTA, CHECKPOINTB, CHECKPOINTC1, CHECKPOINTE, CHECKPOINTF};
StateAuto stateAuto = DEMARRAGE;

String precedent = "no";

enum StateManual {AVANCER, RECULER, GAUCHE, DROITE, KLAXONNER, STOP};
StateManual stateManual = STOP;
StateManual precedentManual = STOP;

const short sizeArriere = 5;
short ledsArriere[sizeArriere] = {7, 8, 9, 10, 11};

short ledTarget = 0;


enum C1State {
    C1_FOLLOW_LINE,
    C1_WAIT_BEFORE_LEFT,
    C1_TURN_LEFT,
    C1_WAIT_AFTER_LEFT,
    C1_TURN_RIGHT,
    C1_WAIT_AFTER_RIGHT
};


bool isFromBLE = false;

String lastCommand = "S";
String currentCommand = "S";
String serialMsg = "";

#pragma region configuration - encodeur

void rightEncoderInterrupt(void)
{
  if(digitalRead(encoderRight.getPortB()) == 0)
  {
    encoderRight.pulsePosMinus();
  }
  else
  {
    encoderRight.pulsePosPlus();;
  }
}

void leftEncoderInterrupt(void) {
  if(digitalRead(encoderLeft.getPortB()) == 0)
  {
    encoderLeft.pulsePosMinus();
  }
  else
  {
    encoderLeft.pulsePosPlus();
  }
}

// ************* DÉBUT ************

void encoderConfig() {
  attachInterrupt(encoderRight.getIntNum(), rightEncoderInterrupt, RISING);
  attachInterrupt(encoderLeft.getIntNum(), leftEncoderInterrupt, RISING);
  
  encoderRight.setPulse(PULSE);
  encoderLeft.setPulse(PULSE);
  
  encoderRight.setRatio(RATIO);
  encoderLeft.setRatio(RATIO);
  
  encoderRight.setPosPid(1.8,0,1.2);
  encoderLeft.setPosPid(1.8,0,1.2);
  
  encoderRight.setSpeedPid(0.18,0,0);
  encoderLeft.setSpeedPid(0.18,0,0);
  
  // DÉBUT : Ne pas modifier ce code!
  // Configuration de la fréquence du PWM
  // Copier-coller ce code si on désire
  // travailler avec les encodeurs
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  // FIN : Ne pas modifier ce code!
}

void offMotors(){
  encoderLeft.setMotorPwm(0);
  encoderRight.setMotorPwm(0); 
}

#pragma endregion

#pragma region Leds

void onAllLeds(){
    led.setColor(red, green, blue);
    led.show();
}

void onLed(int idx){
  led.setColor(idx, red, green, blue);
  led.show();
}

void offLeds(){
  led.setColor(0, 0, 0);
  led.show();
}

void onLedsArray(short array[], int size){
    for( int i = 0; i < size; i++){
        led.setColor(array[i], red, green, blue);
    }

    led.show();
}

void offBuzzer(){
  analogWrite(BUZZER_PIN, 0);
}

#pragma endregion

#pragma region updates

void gyroTask() {
  gyro.update();
}

void encodersTask(unsigned long ct) {
  encoderRight.loop();
  encoderLeft.loop();
}

#pragma endregion

#pragma region PID

void goStraight(short speed = 100, short firstRun = 0) {

  static double zAngleGoal = 0.0;  
  static double error = 0.0;
  static double previousError = 0.0;
  static double output = 0;
  static double errorSum = 0.0;
    
  // PD Controller
  // Change les valeurs selon tes besoins
  // higher kp = plus réactive, peu osciller
  // lowewr kp = sluggish, moins d'oscillation
  // higher kd = limite l'oscillation, la bonne valeur arrête l'oscillation
  const double kp = 6.5;
  // const double ki = 1.0;
  const double kd = 1.1;
    
  if (firstRun) {

    gyro.resetData();
    zAngleGoal = gyro.getAngleZ();

    encoderLeft.setMotorPwm(speed);
    encoderRight.setMotorPwm(-speed); 
      
    return;
  }
    
  error = gyro.getAngleZ() - zAngleGoal;
    
  // Google : ELI5 PID
  // Astuce web : ELI5 = Explain Like I'm 5
  output = kp * error + kd * (error - previousError);
    
  previousError = error;       
 
  encoderLeft.setMotorPwm(speed - output);
  encoderRight.setMotorPwm(-speed - output);
}

#pragma endregion

#pragma region Line-tracker

void calibrer(){
  
  for(int i = 0; i < NB_IR; i++){
    capteurs[i].valeurLue =  ss.analogRead(i);

    if(capteurs[i].valeurLue < capteurs[i].valeurMin)
      capteurs[i].valeurMin = capteurs[i].valeurLue;

    if(capteurs[i].valeurLue > capteurs[i].valeurMax)
      capteurs[i].valeurMax = capteurs[i].valeurLue;

    capteurs[i].seuil = (capteurs[i].valeurMin + capteurs[i].valeurMax) / 2;
  }

}

bool spinRight(short speed, short targetAngle, bool firstRun = 0) {

    static double zAngleGoal = 0.0;
    static int turn_goal = 0;
    static int turn = 0;
    static int last_zone = 0;
    const int intervalle = 20;

    if (firstRun) {
        gyro.resetData();
        zAngleGoal = gyro.getAngleZ() + targetAngle;
        
        if(targetAngle > 0)
          zAngleGoal -= tolerance;
        else
          zAngleGoal += tolerance;

        if(targetAngle > semiCirc) 
            zAngleGoal -= maxAngle;

        turn_goal = (abs(targetAngle) - tolerance) / semiCirc;
        turn = 0;
        last_zone = 0;

        return false;
    }

    encoderLeft.setMotorPwm(speed);
    encoderRight.setMotorPwm(speed);

    double angle = gyro.getAngleZ();
    static int previousSign = 0;

    int sign = (angle > 0) - (angle < 0);

    if (semiCirc - abs(angle) < intervalle && sign != previousSign && previousSign != 0)
        turn++;

    previousSign = sign;

    if(targetAngle > 0)
      return turn == turn_goal && angle > zAngleGoal;
    else
      return turn == turn_goal && angle < zAngleGoal;

}


int capteurNormalisee(int index){
  return ((capteurs[index].valeurLue - capteurs[index].valeurMin) * 1.0) / (capteurs[index].valeurMax - capteurs[index].valeurMin) * 1000.0;
}

void normaliserValeurs(){

  for(int i = 0; i < NB_IR; i++){
    capteurs[i].valeurNormalisee = capteurNormalisee(i);
  }
}

float retournerPosition(){

  float numerateur = 0;
  float denominateur = 0;

  for(int i = 0; i < NB_IR; i++){
    numerateur += capteurs[i].valeurNormalisee * (i -2);
    denominateur += capteurs[i].valeurNormalisee;
  }

  return numerateur / denominateur * 1000;
}

float computePID(float position, float consigne = 0.0f) {

  //0.5 0.01 0.01

    // Ajuster les coefficients selon vos besoins
    static float kp = 0.5; // Coefficient proportionnel
    static float ki = 0.01; // Coefficient intégral
    static float kd = 0.01; // Coefficient dérivé

    static float integral = 0;
    static float derivative = 0;
    static float lastError = 0;

    float error = position - consigne;

    if(!isnan(error))
      integral += error;

    // Adapter cette valeur selon les besoins de votre application
    const float integralLimit = 1000;
    
    // Limiter l'intégrale pour éviter l'emballement intégral
    integral = constrain(integral, -integralLimit, integralLimit);

    derivative = error - lastError;
    lastError = error;

    float output = kp * error + ki * integral + kd * derivative;
    
    return output;
}

void suivreLigne(short speed, float adjustment = 0){

  encoderLeft.setMotorPwm(speed - (int)adjustment);
  encoderRight.setMotorPwm(-speed - (int)adjustment);

}

bool oneOnLine(){


  for(int i = 0; i < NB_IR; i++){

    if(capteurs[i].valeurLue < capteurs[i].seuil)
      return 1;

  }

  return 0;
}

bool allOnLine(){

  for(int i = 0; i < NB_IR; i++){

    if(capteurs[i].valeurLue > capteurs[i].seuil)
      return 0;

  }

  return 1;

}

bool capteLineLeft(){

  const int nb_ir = 3;

  for(int i = 0; i < nb_ir; i++){

    if(capteurs[i].valeurLue > capteurs[i].seuil)
      return 0;

  }

  return 1;

}

bool capteLineRight(){

  const int nb_ir = 2;

  for(int i = nb_ir; i < NB_IR; i++){

    if(capteurs[i].valeurLue > capteurs[i].seuil)
      return 0;

  }

  return 1;

}

#pragma endregion

#pragma region Helpers

int countCharOccurrences(const String &str, char ch) {
  int count = 0;
  for (int i = 0; i < str.length(); i++) {
    if (str[i] == ch) {
      count++;
    }
  }
  return count;
}

void reinitialiserState(){
  stateManual = STOP;
  precedentManual = STOP;
  stateAuto = DEMARRAGE;
  offMotors();
}

float retournerDistance(unsigned long ct){

  static unsigned long lastTime = 0;
  static float lastDistance = 0;
  const int delay = 250;

  if (ct - lastTime < delay) return lastDistance;

  lastTime = ct;

  float localDistance = ultraSensor.distanceCm();
  
  lastDistance = localDistance;

  return localDistance;
}

bool attente(unsigned long ct, int rate = 80) {

    static unsigned long lastTime = 0;
    static bool firstTime = 1;
    // const int rate = 200;

    if (firstTime) {
      lastTime = ct;
      firstTime = 0;
      return false;
    }

    if (ct - lastTime >= rate) {
      lastTime = ct;
      firstTime = 1;
      return true;
    }

    return false;
}


void serialTask(unsigned long ct){

  StaticJsonDocument<128> doc;
  static unsigned long lastSend = ct;
  const int timeSend = 1000;

  if(ct - lastSend < timeSend) return;

  lastSend = ct;

  doc["ts"] = ct; 
  doc["chrono"] = chrono;
  doc["gz"] = gyro.getAngleZ();
  doc["etat"] = mode; 

  JsonObject pwm = doc.createNestedObject("pwm"); 
  pwm["l"] = encoderLeft.getCurPwm(); 
  pwm["r"] = encoderRight.getCurPwm(); 

  JsonArray capt = doc.createNestedArray("capt"); 
  
  for(int i = 0; i < NB_IR; i++){
    capt.add(capteurs[i].valeurNormalisee);
  }

  doc["cp"] = ledTarget;

  serialMsg = "";
  serializeJson(doc, serialMsg);  
  Serial.println(serialMsg); 

}

void serialTask2(unsigned long ct){

  StaticJsonDocument<128> doc;
  doc["ts"] = ct; 
  doc["chrono"] = chrono;
  doc["etat"] = mode; 

  doc["cp"] = ledTarget;

  serialMsg = "";
  serializeJson(doc, serialMsg); 
  Serial.println(serialMsg); 

}

#pragma endregion

#pragma region Manual-State

void avancerState(unsigned long ct){

    static bool firstTime = 0;

    if(precedentManual == STOP)
        firstTime = 1;

    if(firstTime){
        firstTime = 0;

        goStraight(vitesse, 1);
    }

    goStraight(vitesse);

}

void reculerState(unsigned long ct){

    static bool firstTime = 0;

    if(precedentManual == STOP)
        firstTime = 1;

    if(firstTime){
        firstTime = 0;

        goStraight(-vitesseRecul, 1);
    }

    goStraight(-vitesseRecul);

}

void gaucheState(unsigned long ct){

    static bool firstTime = 0;

    if(precedentManual == STOP)
        firstTime = 1;

    if(firstTime){
        firstTime = 0;

        encoderLeft.setMotorPwm(-vitessePivot);
        encoderRight.setMotorPwm(-vitessePivot); 
    }

}

void droiteState(unsigned long ct){

    static bool firstTime = 0;

    if(precedentManual == STOP)
        firstTime = 1;

    if(firstTime){
        firstTime = 0;

        encoderLeft.setMotorPwm(vitessePivot);
        encoderRight.setMotorPwm(vitessePivot); 
    }

}

void klaxonnerState(unsigned long ct){

    short klaxonPower = 200;

    static bool firstTime = 0;

    if(precedentManual != STOP)
        firstTime = 1;

    if(firstTime){
        firstTime = 0;

        analogWrite(BUZZER_PIN, klaxonPower);
    }

}

void stopState(unsigned long ct){

    static bool firstTime = 1;

    if(precedentManual != STOP){
        firstTime = 1;
        precedentManual = STOP;
    }

    if(firstTime){
        firstTime = 0;

        offMotors();
        offBuzzer();
    }

}

void manualManager(unsigned long ct) {

    switch (stateManual){

        case AVANCER:
            avancerState(ct);
            break;

        case RECULER:
            reculerState(ct);
            break;
        
        case GAUCHE:
            gaucheState(ct);
            break;
        
        case DROITE:
            droiteState(ct);
            break;

        case KLAXONNER:
            klaxonnerState(ct);
            break;

        case STOP:
            stopState(ct);
            break;
    
    }
    
}

#pragma endregion

#pragma region Auto-State

void demarrageState(unsigned long ct){

  static bool firstTime = 1;
  static unsigned long initialTime = 0;
  static int lastSec = -1;
  const int secondes = 1000;


  if(firstTime){
    initialTime = ct;
    firstTime = 0;
    lastSec = -1;
  }

  unsigned long elapsed = ct - initialTime;

  int sec = (elapsed) / 1000;
  if (sec != lastSec) {
    lastSec = sec;

    switch (sec) {
      case 0: Serial.println("3"); break;
      case 1: Serial.println("2"); break;
      case 2: Serial.println("1"); break;
      case 3: Serial.println("GO!"); break;
    }
  }

  if(elapsed < 2 * secondes){

    red = 255;
    green = 0;
    blue = 0;

    onAllLeds();
  }

  else if(elapsed < 3 * secondes){

    red = 255;
    green = 255;
    blue = 0;

    onAllLeds();
  }

  bool transition = elapsed > 3 * secondes;

  if(transition){

    firstTime = 1;

    red = 0;
    green = 255;
    blue = 0;

    onAllLeds();

    onGreen = 1;
    exitGreen = ct + secondes;

    stateAuto = SEARCH;

  }

}

void searchState(unsigned long ct) {

    static bool firstTime = 1;

    if (firstTime) {
        firstTime = 0;
        initial = ct;
        goStraight(vitesse, 1);
    }

    if (distance < distanceObjet + ecart)
        goStraight(vitesseVig);
    else
        goStraight(vitesse);

    
    bool transition = allOnLine();

    if (transition) {
        stateAuto = CHECKPOINTA;
        firstTime = 1;
    }
}



void checkpointAState(unsigned long ct) {

    static bool firstTime = 1;
    static bool onTurn = 0;
    static bool afterTurn = 0;

    const int angle = 90;
    const int distanceColis = 50;

    if (firstTime) {
        firstTime = 0;
        onTurn = 1;

        red = 50;
        green = 0;
        blue = 50;

        onLed(++ledTarget);
        serialTask2(ct);

        spinRight(vitessePivot, angle, 1);
        return;
    }

    if (onTurn) {

        bool turnFinish = spinRight(vitessePivot, angle);

        if (!turnFinish)
            return;

        onTurn = 0;
        afterTurn = 1;
        goStraight(vitesse, 1);
        return;
    }

    if (afterTurn) {

        suivreLigne(vitesse, adjustment);

        if (distance < distanceColis) {
            offMotors();
            stateAuto = CHECKPOINTB;
            firstTime = 1;
            onTurn = 0;
            afterTurn = 0;
        }
    }
}


void checkpointBState(unsigned long ct){

  static bool firstTime = 1;
  const int distTake = 20;
  static bool onTurn = 0;
  static bool turnFinish = 0;
  static bool turnAndWait = 0;
  const int angle = 170;

  if(firstTime){
    firstTime = 0;
    goStraight(vitesseVig, 1);
  }

  if(!onTurn && !turnFinish){

    goStraight(vitesseVig);
    
    if(distance < distTake){
      onTurn = 1;

      onLed(++ledTarget);
      serialTask2(ct);


      red = 255;
      green = 165;
      blue = 0;

      onLedsArray(ledsArriere, sizeArriere);

      spinRight(vitessePivot, angle, 1);
    }

    return;
  }

  if(onTurn){

    turnFinish = spinRight(vitessePivot, angle);

    if(turnFinish){
      onTurn = 0;  
    }

  }
  
  if(!turnFinish) return;

  if(!turnAndWait){

    if(attente(ct, 100)){
      turnAndWait = 1;
      goStraight(vitesseVig, 1);
    }
    return;

  }

  goStraight(vitesseVig);

  bool transition = oneOnLine();

  if(transition){
    offMotors();
    stateAuto = CHECKPOINTC1;
    firstTime = 1; 
    onTurn = 0;
    turnFinish = 0;

  }

}


void checkpointC1State(unsigned long ct) {

    static C1State c1 = C1_FOLLOW_LINE;

    const short rightAngle = 180;
    const short leftAngle  = -90;

    int speed = vitesse;

    if (distance < distanceObjet + ecart)
        speed = vitesseVig;

    switch (c1) {

    case C1_FOLLOW_LINE:

        suivreLigne(speed, adjustment);

        if (allOnLine()) {
            goStraight(speed, 1);
            c1 = C1_WAIT_BEFORE_LEFT;
        }
        break;

    case C1_WAIT_BEFORE_LEFT:

        goStraight(speed);

        if (!attente(ct))
            return;

        spinRight(-vitessePivot, leftAngle, 1);
        c1 = C1_TURN_LEFT;
        break;

    case C1_TURN_LEFT:

        if (!spinRight(-vitessePivot, leftAngle))
            return;

        offMotors();

        c1 = C1_WAIT_AFTER_LEFT;
        break;

    case C1_WAIT_AFTER_LEFT:

        if (!attente(ct))
            return;
          
        if (distance < distanceObjet) {
            c1 = C1_TURN_RIGHT;
            spinRight(vitessePivot, rightAngle, 1);
        } else {
            stateAuto = CHECKPOINTC1;
            c1 = C1_FOLLOW_LINE;

            red = 50;
            green = 0;
            blue = 50;

            onLed(++ledTarget);
            serialTask2(ct);

        }
        break;

    case C1_TURN_RIGHT:

      if (!spinRight(vitessePivot, rightAngle))
          return;

      offMotors();
      c1 = C1_WAIT_AFTER_RIGHT;
      break;


    case C1_WAIT_AFTER_RIGHT:

      if (!attente(ct))
          return;

      if (distance < distanceObjet) {
          stateAuto = CHECKPOINTE;
      } else {
          stateAuto = CHECKPOINTC1;

          red = 50;
          green = 0;
          blue = 50;

          onLed(++ledTarget);
          serialTask2(ct);

      }

      c1 = C1_FOLLOW_LINE;
      break;

    }
}

void checkpointEState(unsigned long ct) {

    enum EState { E_PIVOT_INIT, E_PIVOT, E_BACK_INIT, E_BACK };
    static EState es = E_PIVOT_INIT;

    const short backAngle = 85;

    switch (es) {

    case E_PIVOT_INIT:
      spinRight(vitessePivot, backAngle, 1);
      es = E_PIVOT;
      break;

  case E_PIVOT:
    if (!spinRight(vitessePivot, backAngle))
      return;

    es = E_BACK_INIT;
    break;

  case E_BACK_INIT:
    goStraight(-vitesseVig, 1);
    es = E_BACK;
    break;

  case E_BACK:
    goStraight(-vitesseVig);

    if(attente(ct, 500)) {

      offMotors();
      onLed(++ledTarget);
      serialTask2(ct);


      red = 0;
      green = 255;
      blue = 0;

      onLedsArray(ledsArriere, sizeArriere);

      es = E_PIVOT_INIT;
      mode = MANUAL;
      vitesse = 200;
      reinitialiserState();
    }

    break;
  }

}

void checkpointFState(unsigned long ct){

  static bool firstTime = 1;
  static bool firstOn = 1;
  const int distIssue = 10;
  int vitesseVig1 = 60;
  int tolerance = 3;

  if(firstTime){
    firstTime = 0;

    red = 50;
    green = 0;
    blue = 50;

    goStraight(vitesseVig1, 1);

  }

  Serial.println(distance);

  if(distance > distIssue + tolerance){
    goStraight(vitesseVig1);
    return;
  }

  if(firstOn){
    onLed(++ledTarget); 
    serialTask2(ct);

    offMotors();
    firstOn = 0;
  }
  

}


void calibrationState(unsigned long ct){
  static bool firstTime = 1;
  int speed = 90;

  if(firstTime){
    firstTime = 0;

    spinRight(speed, maxAngle, 1);
  }

  calibrer();

  bool transition = spinRight(speed, maxAngle);
  

  if (transition){
    mode = ARRET;
    offMotors();
    firstTime = 1;
  }

}


void arretState(unsigned long ct){

  static bool firstTime = 1;

  if(firstTime){

    firstTime = 0;
    offMotors();
  
  } 

  ledLoopTask();

}




void autoManager(unsigned long ct) {

  calibrer();

  normaliserValeurs();

  position = retournerPosition();

  adjustment = computePID(position, consigne);

  switch (stateAuto) {

    case DEMARRAGE:
      demarrageState(ct);
      break;

    case SEARCH:
      searchState(ct);
      break; 

    case CHECKPOINTA:
      checkpointAState(ct);
      break;

    case CHECKPOINTB:
      checkpointBState(ct);
      break;

    case CHECKPOINTC1:
      checkpointC1State(ct);
      break;

    case CHECKPOINTE:
      checkpointEState(ct);
      break;

    case CHECKPOINTF:
      checkpointFState(ct);
      break;

  }

}

void manageMode(unsigned long ct){

  switch(mode){

    case CALIBRATION:
      calibrationState(ct);
      break;

    case AUTO:
      autoManager(ct);
      break;

    case MANUAL:
      manualManager(ct);
      break;

    case ARRET:
      arretState(ct);
      break;
  }
}


#pragma endregion

#pragma region receive_Data

// Événement qui se déclenche lorsqu'il y a réception de données via le port série
void serialEvent() {
  static String receivedData = "";

  if (!Serial.available()) return;

  receivedData = Serial.readStringUntil('\n');
  parseData(receivedData);
}

/**
  Fonction servant à analyser les données reçues.
  "parse" veut dire analyser
*/
void parseData(String& receivedData) {

  if (receivedData.length() >= 2) {
    // Vérifier si les deux premiers octets sont 0xFF55 (BLE)
    if ((uint8_t)receivedData[0] == 0xFF && (uint8_t)receivedData[1] == 0x55) {
      isFromBLE = true;
      // Supprimer les deux premiers octets
      receivedData.remove(0, 2);
    }
    // Vérifier si les deux premiers caractères sont "!!" (Moniteur Série)
    else if (receivedData.startsWith("!!")) {
      // Supprimer les deux premiers caractères
      receivedData.remove(0, 2);
    } else {
      // En-tête non reconnue
      Serial.print(F("Données non reconnues : "));
      
      Serial.println(receivedData);
      return;
    }
  } else {
    Serial.print(F("Données trop courtes : "));
    Serial.println(receivedData);
    return;
  }

  lastCommand = currentCommand;
  currentCommand = receivedData;

  // Découpage de la commande et des paramètres
  int firstComma = receivedData.indexOf(',');

  if (firstComma == -1) {
    // Pas de virgule, donc c'est une commande sans paramètres
    handleCommand(receivedData);
  } else {
    // Il y a des paramètres
    String command = receivedData.substring(0, firstComma);
    String params = receivedData.substring(firstComma + 1);
    handleCommandWithParams(command, params);
  }
}

// Fonction pour gérer une commande sans paramètres
void handleCommand(String command) {

    precedentManual = stateManual;
    
    if(command.startsWith("GO")){
      chrono = 0;
      initial = 0;
      mode = AUTO;
      reinitialiserState();
      return;
    }

    else if(command.startsWith("STOP")){
      mode = ARRET;
      reinitialiserState();
      return;
    }

    else if(command.startsWith("MAN")){
      mode = MANUAL;
      reinitialiserState();
      offLeds();
      return;
    }

    else{

      char cmd = command[0];
      switch (cmd) {
        case 'F':   
          stateManual = AVANCER;
          break;

        case 'B':   
          stateManual = RECULER;
          break;

        case 'L':   
          stateManual = GAUCHE;
          break;

        case 'R':   
          stateManual = DROITE;
          break;

        case 'K': 
          stateManual = KLAXONNER;
          break;

        case 'C':   
          mode = CALIBRATION;
          break;
        
        case 'T':   
          mode = AUTO;
          stateAuto = CHECKPOINTF;
          break;
        
        case 'S':   
          stateManual = STOP;
          break;

        default:
          Serial.print(F("Commande inconnue sans paramètres : "));
          Serial.println(command);
          break;
      }
    }

}

// Fonction pour gérer une commande avec paramètres
void handleCommandWithParams(String command, String params) {


    if(command == "pBack"){
      commandSpeedBack(params);
    }
    else if(command == "pPivot"){
      commandSpeedPivot(params);
    }
    else{
        char cmd = command[0];
        switch (cmd) {

          case 'l':  // Commande "LIGHT" pour définir la couleur de l'anneau LED
            commandLight(params);
            break;

          case 'p':  // Commande "SPEED" pour définir la vitesse du robot
            commandSpeed(params);
            break;

          default:
            Serial.print(F("Commande inconnue avec paramètres : "));
            Serial.print(command);
            Serial.print(F(", "));
            Serial.println(params);
            break;
        }
    }
    
}

#pragma endregion

#pragma region COMMANDES

void ledAction() {
  led.setColor(red, green, blue);
  led.show();   
}

void ledAction(int idx) {
  // Mettre à jour la couleur de toutes les LEDs de l'anneau
  if (idx == 0) {
    led.setColor(red, green, blue);  
  }
  else {
    led.setColor(idx, red, green, blue);  
  }
  
  led.show(); 
}

void ledLoopTask()
{
  static float j;
  static float f;
  static float k;
  
  for (uint8_t t = 0; t < LEDNUM; t++ )
  {
    uint8_t red	= 8 * (1 + sin(t / 2.0 + j / 4.0) );
    uint8_t green = 8 * (1 + sin(t / 1.0 + f / 9.0 + 2.1) );
    uint8_t blue = 8 * (1 + sin(t / 3.0 + k / 14.0 + 4.2) );
    led.setColorAt( t, red, green, blue );
  }
  led.show();

  j += random(1, 6) / 6.0;
  f += random(1, 6) / 6.0;
  k += random(1, 6) / 6.0;
}


void commandLight(String params) {
  int commaCount = countCharOccurrences(params, ',');
  
  // Vérifie le nombre de paramètres en comptant les virgules
  if (commaCount == 2) {
    // Trois paramètres (r, g, b) pour définir toute la couleur de l'anneau
    red = params.substring(0, params.indexOf(',')).toInt();
    params = params.substring(params.indexOf(',') + 1);
    green = params.substring(0, params.indexOf(',')).toInt();
    blue = params.substring(params.indexOf(',') + 1).toInt();
    
    ledAction();  // Appel pour affecter l'ensemble de l'anneau
  } 
  else if (commaCount == 3) {
    // Quatre paramètres (idx, r, g, b) pour définir une LED spécifique
    int idx = params.substring(0, params.indexOf(',')).toInt();
    params = params.substring(params.indexOf(',') + 1);
    red = params.substring(0, params.indexOf(',')).toInt();
    params = params.substring(params.indexOf(',') + 1);
    green = params.substring(0, params.indexOf(',')).toInt();
    blue = params.substring(params.indexOf(',') + 1).toInt();
    
    ledAction(idx);  // Appel pour affecter une LED spécifique
  } 
  else {
    Serial.println(F("Commande lumière invalide"));
  }
}

void commandSpeed(String params) {

  int arg = params.toInt();

  if(arg >= 50 && arg <= 255)
    vitesse = params.toInt();
  else{
    Serial.print(params);
    Serial.println(F(" n'est pas un nombre compris entre 50 et 255"));
  }
}

void commandSpeedBack(String params) {

  int arg = params.toInt();

  if(arg >= 50 && arg <= 255)
    vitesseRecul = params.toInt();
  else{
    Serial.print(params);
    Serial.println(F(" n'est pas un nombre compris entre 50 et 255"));
  }
}

void commandSpeedPivot(String params) {

  int arg = params.toInt();

  if(arg >= 50 && arg <= 255)
    vitessePivot = params.toInt();
  else{
    Serial.print(params);
    Serial.println(F(" n'est pas un nombre compris entre 50 et 255"));
  }
}


#pragma endregion


#pragma region setup-loop

void setup() {

  Serial.begin(115200);
  encoderConfig();
  gyro.begin();
  led.setpin(LED_PIN);

  if (!ss.begin()) {
    Serial.println("Erreur de connexion au LyneTracker");
    while (1);
  }
  Serial.println("Connexion réussie au LyneTracker!");

}

void loop() {

  currentTime = millis();

  distance = retournerDistance(currentTime);

  manageMode(currentTime);

  if(currentTime > exitGreen && onGreen){
    onGreen = 0;
    offLeds();
  }

  if(mode != ARRET && mode != CALIBRATION && initial != 0){
    chrono = currentTime - initial;
  }

  serialTask(currentTime);

  gyroTask();
  encodersTask(currentTime);
}

#pragma endregion

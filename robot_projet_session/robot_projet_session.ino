#include <MeAuriga.h>
#include <Adafruit_seesaw.h>
#include <ArduinoJson.h>

#define NB_IR 5
#define PULSE 9
#define RATIO 39.267
#define CIRC_WHEEL 202.6

struct Capteur{
  int  valeurMin = 1023;
  int valeurMax = 0;
  int valeurLue;
  int valeurNormalisee;
};

Capteur capteurs[NB_IR];

MeGyro gyro(0, 0x69);
MeUltrasonicSensor ultraSensor(PORT_10);

Adafruit_seesaw ss;

StaticJsonDocument<128> doc;

MeEncoderOnBoard encoderRight(SLOT1);
MeEncoderOnBoard encoderLeft(SLOT2);

float distance = 0;
const int lastValue = 400;
int distanceObjet = 30;

short vitesse = 100;
short vitessePivot = 100;


unsigned long currentTime = 0;
unsigned long chrono = 0;

short maxAngle = 360;
short semiCirc = 180;

int tolerance = 2;

float consigne = 0.0f;
float position;

bool isOff = 1;

enum State {CALIBRATION, ONLINE, VIGILENCE, EMBRANCHEMENT, LEFT, RIGHT, ARRET};
State state = CALIBRATION;

enum StateT {LEFT2, RIGHT2, STOP};
StateT stateT = LEFT2;

String serialMessage = "";

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

#pragma endregion


void offMotors(){
  encoderLeft.setMotorPwm(0);
  encoderRight.setMotorPwm(0); 
}

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
        zAngleGoal = gyro.getAngleZ() + targetAngle - tolerance;

        if(targetAngle > semiCirc) 
            zAngleGoal -= maxAngle;

        turn_goal = (targetAngle - tolerance) / semiCirc;
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

    return turn == turn_goal && angle > zAngleGoal;
}


bool spinLeft(short speed, short targetAngle, bool firstRun = 0){  // à modifier

  static double zAngleGoal = 0.0;
  short cummulAngle = 0;

  if (firstRun) {
  
    gyro.resetData();

    zAngleGoal = gyro.getAngleZ() + targetAngle;

    return false;
  }

  encoderLeft.setMotorPwm(-speed);
  encoderRight.setMotorPwm(-speed);
  
  if(targetAngle > maxAngle / 2){

    if(gyro.getAngleZ() < 0){
      cummulAngle = maxAngle + gyro.getAngleZ();
      return cummulAngle > zAngleGoal - tolerance;
    }

    return 0;
  }

  return gyro.getAngleZ() < zAngleGoal + tolerance;

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

bool checkLine(unsigned long ct){

  static unsigned long lastTime = ct;
  const int tolerenceInterval = 150;
  const int rate = 200;


  for(int i = 0; i < NB_IR; i++){

    if(capteurs[i].valeurMax - capteurs[i].valeurLue > tolerenceInterval)
      lastTime = ct;

  }

  return ct - lastTime > rate;

}

bool allOnLine(unsigned long ct){

  bool onLine = 1;
  const int tolerenceInterval = 150;

  for(int i = 0; i < NB_IR; i++){

    if(capteurs[i].valeurMax - capteurs[i].valeurLue < tolerenceInterval)
      onLine = 0;

  }

  return onLine;

}

bool capteLineLeft(){

  bool onLine = 1;
  const int tolerenceInterval = 150;
  const int nb_ir = 3;
  

  for(int i = 0; i < nb_ir; i++){

    if(capteurs[i].valeurMax - capteurs[i].valeurLue < tolerenceInterval)
      onLine = 0;

  }

  return onLine;

}

bool capteLineRight(){

  bool onLine = 1;
  const int tolerenceInterval = 150;
  const int nb_ir = 2;
  

  for(int i = nb_ir; i < NB_IR; i++){

    if(capteurs[i].valeurMax - capteurs[i].valeurLue < tolerenceInterval)
      onLine = 0;

  }

  return onLine;

}

bool turn(short angle){

  static bool firstTime = 1;

  if(firstTime){
    firstTime = 0;

    if(angle >= 0)
      spinRight(vitessePivot, angle, 1);
    else
      spinLeft(vitessePivot, angle, 1);
  }

  if(angle >= 0)
    firstTime = spinRight(vitessePivot, angle);
  else
    firstTime = spinLeft(vitessePivot, angle);

  return firstTime;

}

void attente(bool firstTime = 0){

  const int rate = 500;
  static unsigned long lastTime = millis();

  if(firstTime) {
    lastTime = millis();
    offMotors();
  }

  while(millis() - lastTime < rate){}

  lastTime = millis();

}

void suivreLigne(short speed, float adjustment = 0){



  encoderLeft.setMotorPwm(speed - (int)adjustment);
  encoderRight.setMotorPwm(-speed - (int)adjustment);

}


#pragma endregion

#pragma region state

void calibrationState(unsigned long ct){
  static bool firstTime = 1;

  if(firstTime){
    firstTime = 0;

    spinRight(vitessePivot, maxAngle, 1);
  }

  calibrer();

  bool transition = spinRight(vitessePivot, maxAngle);
  

  if (transition){
    state = ONLINE;
    offMotors();
    firstTime = 1;
  }

}

void onlineState(unsigned long ct, float adjustment){

  static bool firstTime = 1;
  short ecart = 10;

  suivreLigne(vitesse, adjustment);

  bool transition = distance < distanceObjet + ecart;

  if(transition){

    state = VIGILENCE;
    firstTime = 1;
    
  }

  bool transitionEnd = checkLine(ct);

  if(transitionEnd){

    state = ARRET;
    firstTime = 1;
    
  }
  
}

void vigilenceState(unsigned long ct, float adjustment){

  static bool firstTime = 1;
  static bool isallOn = 0;
  static bool isLeftOn = 0;
  static bool isRightOn = 0;

  short vitesseVig;
  

  if(firstTime){
    firstTime = 0;
    vitesseVig = vitesse * 0.7;
  }

  suivreLigne(vitesseVig, adjustment);

  if(allOnLine(ct))
    isallOn = 1;

  bool transition = isallOn && checkLine(ct);

  if(transition){

    isallOn = 0;
    offMotors();
    state = EMBRANCHEMENT;
    firstTime = 1;
    return; 
  }


  if(capteLineLeft())
    isLeftOn = 1;

  bool transitionLeft = isLeftOn && checkLine(ct);

  if(transitionLeft){

    isLeftOn = 0;
    offMotors();
    state = LEFT;
    firstTime = 1;
    return;
    
  }

  if(capteLineRight())
    isRightOn = 1;

  bool transitionRight = isRightOn && checkLine(ct);

  if(transitionRight){

    isRightOn = 0;
    offMotors();
    state = RIGHT;
    firstTime = 1;
    return;
    
  }
  
}

void leftState(unsigned long ct){

  const short leftAngle = -90;

  bool transition = turn(leftAngle);

  if(transition){
    state = ONLINE;
    offMotors();
  }

}

void rightState(unsigned long ct){

  const short rightAngle = 90;

  bool transition = turn(rightAngle);

  if(transition){
    state = ONLINE;
    offMotors();
  }

}

void embranchementState(unsigned long ct){

  static bool firstTime = 1;
  const short leftAngle = -90;
  const short rightAngle = 180;

  if(stateT == LEFT2){

    if(!turn(leftAngle) ) return;
    attente(firstTime);

    if(distance > distanceObjet){
      firstTime = 1;
      state = ONLINE;
      return;
    } 
    else {
      stateT = RIGHT2;
    }

  }
    

 if(stateT == RIGHT2){
    if(!turn(rightAngle)) return;
    
    attente(firstTime);

    if(distance <= distanceObjet){

      if(!turn(leftAngle)) return;

      Serial.println("Pris");
      stateT = STOP;

    } else{
      firstTime = 1;
      state = ONLINE;
      stateT = LEFT2;

    }
        
  }

  firstTime = 0;

}

void arretState(unsigned long ct){

  static bool firstTime = 1;

  if(firstTime){
    offMotors();
  }

}

void stateManager(unsigned long ct, float adjustment) {

  static bool firstTime = 1;
  static unsigned long initial;

  switch (state){

    case CALIBRATION:
      calibrationState(ct);
      break;

    case ONLINE:

      if(firstTime){
        initial = ct;
        firstTime = 0;
      }

      onlineState(ct, adjustment);
      break;

    case VIGILENCE:
      vigilenceState(ct, adjustment);
      break;

    case EMBRANCHEMENT:
      embranchementState(ct);
      break;

    case LEFT:
      leftState(ct);
      break;

    case RIGHT:
      rightState(ct);
      break;

    case ARRET:
      arretState(ct);
      break;
    
  }
    
}

#pragma endregion

#pragma region setup-loop

void setup() {

  Serial.begin(115200);
  encoderConfig();
  gyro.begin();

  if (!ss.begin()) {
    Serial.println("Erreur de connexion au LyneTracker");
    while (1);
  }
  Serial.println("Connexion réussie au LyneTracker!");

}

void loop() {

  currentTime = millis();

  distance = retournerDistance(currentTime);

  calibrer();

  normaliserValeurs();

  position = retournerPosition();

  float adjustment = computePID(position, consigne);

  stateManager(currentTime, adjustment);

  serialTask(currentTime);

  gyroTask();
  encodersTask(currentTime);
}

#pragma endregion

#pragma region Helpers

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

void serialTask(unsigned long ct){

  unsigned long lastSend = 0;
  const int timeSend = 1000;

  if(ct - lastSend < timeSend) return;

  lastSend = ct;

  doc["ts"] = ct; 
  doc["gz"] = gyro.getAngleZ();
  doc["etat"] = state; 

  JsonObject pwm = doc.createNestedObject("pwm"); 
  pwm["l"] = encoderLeft.getCurPwm(); 
  pwm["r"] = encoderRight.getCurPwm(); 

  JsonArray capt = doc.createNestedArray("capt"); 
  
  for(int i = 0; i < NB_IR; i++){
    capt.add(capteurs[i].valeurNormalisee);
  }

  serializeJson(doc, serialMessage); 
  Serial.println(serialMessage); 

}

#pragma endregion

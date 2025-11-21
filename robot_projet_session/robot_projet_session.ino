#include <MeAuriga.h>
#include <Adafruit_seesaw.h>

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

MeEncoderOnBoard encoderRight(SLOT1);
MeEncoderOnBoard encoderLeft(SLOT2);

float distance = 0;
const int lastValue = 400;

short vitesse = 110;
short vitessePivot = 100;
unsigned long currentTime = 0;

short maxAngle = 360;
short anglePivot = 180;

int tolerance = 2;

float consigne = 0.0f;
float position;

bool isOff = 1;

enum State {CALIBRATION, ONLINE, VIGILENCE, TURN};
State state = ONLINE;

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


bool spin(short speed, short targetAngle, bool firstRun = 0){

  static double zAngleGoal = 0.0;
  short cummulAngle = 0;

  if (firstRun) {
  
    gyro.resetData();

    zAngleGoal = gyro.getAngleZ() + targetAngle;

    return false;
  }

  encoderLeft.setMotorPwm(speed);
  encoderRight.setMotorPwm(speed);

  if(targetAngle > maxAngle / 2){

    if(gyro.getAngleZ() < 0){
      cummulAngle = maxAngle + gyro.getAngleZ();
      return cummulAngle > zAngleGoal - tolerance;
    }

    return 0;
  }

  return gyro.getAngleZ() > zAngleGoal - tolerance;

}

void calibrationAutomatique(){

  spin(vitesse, anglePivot, 1);


  while(!spin(vitesse, anglePivot)){
    Serial.println("ici");
    calibrer();
    //gyroTask();
  }

  offMotors();

   for(int i = 0; i < NB_IR; i++){
        Serial.print("IR"); Serial.print(i); Serial.print(":");
        Serial.print(" min  ");
        Serial.print(capteurs[i].valeurMin);
        Serial.print(" max  ");
        Serial.println(capteurs[i].valeurMax);
   }

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

void suivreLigne(short speed, float adjustment){

  encoderLeft.setMotorPwm(speed - (int)adjustment);
  encoderRight.setMotorPwm(-speed - (int)adjustment);

}


#pragma endregion

#pragma region state

void calibrationState(unsigned long ct){
  static bool firstTime = 1;

  if(firstTime){
    firstTime = 0;

    spin(vitesse, anglePivot, 1);
  }

  calibrer();

  bool transition = spin(vitesse, anglePivot);

  if (transition){
    state = ONLINE;
    offMotors();
    firstTime = 1;
  }

}

void onlineState(unsigned long ct, float adjustment){

  static bool firstTime = 1;

  // if(firstTime){
  //   firstTime = 0;
  // }

  suivreLigne(vitesse, adjustment);

  bool transition = distance < 50;

  if(transition){

    state = VIGILENCE;
    firstTime = 1;
    
  }
  
}

void vigilenceState(unsigned long ct, float adjustment){

  static bool firstTime = 1;
  short vitesseVig;
  

  if(firstTime){
    firstTime = 0;
    vitesseVig = vitesse * 0.5;
  }

  suivreLigne(vitesseVig, adjustment);

  bool transition = checkLine(ct);

  if(transition){

    offMotors();
    state = TURN;
    firstTime = 1;
    
  }
  
}

void turnState(unsigned long ct){

  static bool firstTime = 1;
  // static unsigned long lastTime = ct;
  // const int rate = 1000;

  // if (ct - lastTime < rate) return;

  // lastTime = ct;

  if(firstTime){
    firstTime = 0;

    spin(vitessePivot, anglePivot, 1);
  }

  bool transition = spin(vitessePivot, anglePivot);

  if(transition){

    offMotors();
    state = ONLINE;
    firstTime = 1;
    
  }
  
}



void stateManager(unsigned long ct, float adjustment) {

  switch (state){

    case CALIBRATION:
      calibrationState(ct);
      break;

    case ONLINE:
      onlineState(ct, adjustment);
      break;

    case VIGILENCE:
      vigilenceState(ct, adjustment);
      break;

    case TURN:
      turnState(ct);
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

#pragma endregion


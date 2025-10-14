#include <MeAuriga.h>
MeUltrasonicSensor ultraSensor(PORT_10);

#define ALL_LEDS 0
#define LEDNUM  12 // Auriga on-board light ring has 12 LEDs
#define LED_PIN 44

#define DIST_WHEEL 151
#define DIA_WHEEL 64.5
#define PULSE 9
#define RATIO 39.267
#define FULL_TURN_CIRC 948.8
#define FULL_SPIN_CIRC 474.4
#define CIRC_WHEEL 202.6
#define RAYON 3.25

// on-board LED ring, at PORT0 (onboard)
MeRGBLed led( 0, LEDNUM );
MeGyro gyro(0, 0x69);

int distance = 0;
float distanceParcourue = 0;
float distanceX = 180;
float distanceY = 60;
float backDistance = 0;

unsigned long currentTime;

MeEncoderOnBoard encoderRight(SLOT1);
MeEncoderOnBoard encoderLeft(SLOT2);

const int maxAngle = 360;
const int anglePivot = 90;

const float pi = 3.14159;

const int lastValue = 400;

short vitesse = 100;
short vitessePivot = 80;

const int minDistance = 0;
const int minLed = 1;

short tolerance = 2;

short red = 0;
short blue = 0;
short green = 255; 

enum State {SEGMENT1, PIVOT, SEGMENT2, LIVRAISON, RETOUR, FIN};
State state = SEGMENT1;
State precedentState = SEGMENT1;


// ********* INTERRUPTIONS ***********

#pragma region configuration - encodeur

void rightEncoderInterrupt(void)
{
  if(digitalRead(encoderRight.getPortB()) == 0)
  {
    encoderRight.pulsePosMinus();
  }
  else
  {
    encoderRight.pulsePosPlus();
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

#pragma region updates

void gyroTask(unsigned long ct) {
  gyro.update();
}

void encodersTask(unsigned long ct) {
  encoderRight.loop();
  encoderLeft.loop();
}

#pragma endregion

int retournerDistance(unsigned long ct){

  static unsigned long lastTime = 0;
  static int lastDistance = 0;
  const int delay = 250;

  if (ct - lastTime < delay) return lastDistance;

  lastTime = ct;

  int localDistance = ultraSensor.distanceCm();
  
  lastDistance = localDistance;

  return localDistance;
}

float calculDistancePar(unsigned long ct){

  static unsigned long lastTime = 0;
  static int lastDistance = 0;
  const int delay = 100;

  if (ct - lastTime < delay) return lastDistance;

  lastTime = ct;

  long posLeft = encoderLeft.getCurPos();
  long posRight = encoderRight.getCurPos();

  float posMoyenne = (abs(posLeft) + abs(posRight)) / 2.0;

  float localDistance = (posMoyenne / maxAngle) * (CIRC_WHEEL / 10);  //distance en cm
  
  lastDistance = localDistance;

  return localDistance;

}

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
    Serial.println ("Setting Vitesse");

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

bool spin(short speed, int targetAngle, bool firstRun = 0){

  static double zAngleGoal = 0.0;

  if (firstRun) {
  
    gyro.resetData();

    zAngleGoal = gyro.getAngleZ() + targetAngle;
    return false;
  }

  encoderLeft.setMotorPwm(speed);
  encoderRight.setMotorPwm(speed);

  return gyro.getAngleZ() > zAngleGoal - tolerance;

}


#pragma endregion

#pragma region DEL

void graduateLed(){

  int index = map(distanceParcourue, minDistance, distanceX, minLed, LEDNUM);

  led.setColor(index, red, green, blue);
  led.show();

}

void clignoteTask(unsigned long ct){

  static unsigned long lastTime = 0;
  static bool onOff = 1; 
  const int rate = 200;

  if(ct - lastTime < rate) return;

  lastTime = ct;

  if(onOff)
    led.setColor(red, green, blue);
  else
    led.setColor(0, 0, 0);

  led.show();

  onOff = !onOff;

}

void ledLivre(unsigned long ct){

  static unsigned long lastTime = 0;
  const int rate = 150;
  static short idx = 1;

  if(ct - lastTime < rate) return;

  lastTime = ct;

  led.setColor(0, 0, 0);
  led.setColor(idx, red, green, blue);

  idx = idx >= LEDNUM ? 1 : idx + 1;

  led.show();

}

void offLeds(){
  led.setColor(0, 0, 0);
  led.show();
}

#pragma endregion

#pragma region State

void segment1State(unsigned long ct){
  static bool firstTime = 1;
  static float position = 0;

  if(firstTime){
    Serial.println("Entrée etat: Segment1");
    firstTime = 0;

    position = distanceParcourue;

    goStraight(vitesse, 1);
  }

  if(precedentState == SEGMENT1){
    graduateLed();
  }

  else if(precedentState == PIVOT){
    ledLivre(ct);
  }
  
  goStraight(vitesse);

  bool transition = distanceParcourue - position > distanceX;

  if(transition){

    State temp = state;
    state = precedentState == PIVOT ? FIN : PIVOT;
    precedentState = temp; 

    offLeds();
    offMotors();

    firstTime = 1;

    Serial.println("Sortie etat: Segment1");
    return;
  }
}


void pivotState(unsigned long ct){
  static bool firstTime = 1;
  static unsigned long lastTime = 0;
  const int rate = 500;


  if(firstTime){
    Serial.println("Entrée etat: Pivot");
    firstTime = 0;

    spin(vitessePivot, anglePivot, 1);
  //   goPivot(vitesse, 1, anglePivot);

  }
 
  if(precedentState == RETOUR)
    ledLivre(ct);

  bool transition = spin(vitessePivot, anglePivot);

  if(transition){

    State temp = state;
    state = precedentState == SEGMENT1 ? SEGMENT2 : SEGMENT1;
    precedentState = temp;

    firstTime = 1;

    offMotors();

    Serial.println("Sortie etat: Pivot");
    return;
  }
}

void segment2State(unsigned long ct){
  static bool firstTime = 1;
  static unsigned long lastTime = 0;
  short speedRalenti = 0.5 * vitesse;
  int distanceRalenti = 65;

  if(firstTime){
    Serial.println("Entrée etat: Segment2");
    firstTime = 0;

    backDistance = distanceParcourue;

    goStraight(vitesse, 1);

  }

  distance = retournerDistance(currentTime);
  
  if(distance < distanceRalenti){
    goStraight(speedRalenti);
  }
  else{
    goStraight(vitesse);
  }

  bool transition = distance <= distanceY;

  if(transition){

    precedentState = state;
    state = LIVRAISON;
    firstTime = 1;

    backDistance = distanceParcourue - backDistance;

    offMotors();

    Serial.println("Sortie etat: Segment2");
    return;
  }
}

void livraisonState(unsigned long ct){
  static bool firstTime = 1;
  static unsigned long exitTime;
  const int rate = 3000;

  if(firstTime){
    Serial.println("Entrée etat: Livraison");
    firstTime = 0;

    exitTime = ct + rate;
  }

  clignoteTask(ct);

  bool transition = ct > exitTime;

  if(transition){

    precedentState = state;
    state = RETOUR;
    firstTime = 1;

    offLeds();

    Serial.println("Sortie etat: Livraison");
    return;
  }
}


void retourState(unsigned long ct){
  static bool firstTime = 1;
  static float position = 0;

  if(firstTime){
    Serial.println("Entrée etat: Retour");
    firstTime = 0;

    position = distanceParcourue;

    goStraight(-vitesse, 1);

  }

  ledLivre(ct);

  goStraight(-vitesse);

  bool transition = position - distanceParcourue > backDistance;

  if(transition){

    precedentState = state;
    state = PIVOT;
    firstTime = 1;

    offMotors();

    Serial.println("Sortie etat: Retour");
    return;
  }
}


void finState(unsigned long ct){
  static bool firstTime = 1;

  if(firstTime){
    Serial.println("Entrée etat: Fin");
    firstTime = 0;

    red = 255;
    green = 0;
    blue = 0;

    led.setColor(red, green, blue);
    led.show();

    // offLeds();
    offMotors();
  }
  

}

void manageState(unsigned long ct){

  switch(state){
    case SEGMENT1:
      segment1State(ct);
      break;
    
    case PIVOT:
      pivotState(ct);
      break;

    case SEGMENT2:
      segment2State(ct);
      break;

    case LIVRAISON:
      livraisonState(ct);
      break; 

    case RETOUR:
      retourState(ct);
      break;

    case FIN:
      finState(ct);
      break;
  }
  
}

#pragma endregion


void setup() {
  
  Serial.begin(115200);
  led.setpin( LED_PIN );
  encoderConfig();
  gyro.begin();
  offLeds();


  delay(3000);

}

void loop() {
  
  currentTime = millis();

  distanceParcourue = calculDistancePar(currentTime);

  manageState(currentTime);
 

  gyroTask(currentTime);
  encodersTask(currentTime);

}
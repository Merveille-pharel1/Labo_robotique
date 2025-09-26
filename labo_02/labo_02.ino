#include <MeAuriga.h>
MeUltrasonicSensor ultraSensor(PORT_10);

#define ALL_LEDS 0
#define LEDNUM  12 // Auriga on-board light ring has 12 LEDs
#define LED_PIN 44

// on-board LED ring, at PORT0 (onboard)
MeRGBLed led( 0, LEDNUM );

int distance;
unsigned long currentTime;

int red;
int green;
int blue;

const int lastValue = 400;

const int maxVitesse = 255; 
float vitesse;

const int m1_pwm = 11;
const int m1_in1 = 48; 
const int m1_in2 = 49;

const int m2_pwm = 10;
const int m2_in1 = 47; // M1 ENB
const int m2_in2 = 46; // M1 ENA

const int minLed = 6;
const int maxLed = 12;

const float propNormal = 0.7;
const float propRalenti = 0.5;

const int maxDanger = 40;
const int maxRalentir = 80;

enum State {NORMAL, RALENTI, DANGER, RONDE};
enum StateDanger {ARRET, RECUL, PIVOT};

State state = NORMAL;
StateDanger stateDanger = ARRET;

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

void normalState(unsigned long ct){

  static unsigned long enterRonde; 
  static bool firstTime = 1;
  static unsigned long lastTime = 0;
  const int countTime = 5000;

  if(firstTime){
    Serial.println("Entrée etat: Normal");
    firstTime = 0;

    enterRonde = ct + countTime;

    //Allumer les Leds
    red = 0; 
    green = 255; 
    blue = 0;

    onLeds();

    // Moteur 
    vitesse = propNormal * maxVitesse;
    digitalWrite(m1_in2, LOW);
    digitalWrite(m1_in1, HIGH);
    analogWrite(m1_pwm, vitesse);

    digitalWrite(m2_in2, LOW);
    digitalWrite(m2_in1, HIGH);
    analogWrite(m2_pwm, vitesse);
  }

  bool transitionRonde = ct > enterRonde;

  if(transitionRonde){
    state = RONDE;
    firstTime = 1;

    Serial.println("Sortie etat: Normal");
    return;
  }

  bool transition = distance < maxRalentir;

  if(transition){

    state = RALENTI;
    firstTime = 1;
    
    Serial.println("Sortie etat: Normal");
  }


}

void ralentiState(unsigned long ct){

  static bool firstTime = 1;
  static unsigned long lastTime = 0;

  if(firstTime){
    Serial.println("Entrée etat: Ralenti");
    firstTime = 0;
    
    //Allumer les Leds
    red = 0; 
    green = 0; 
    blue = 255;

    onLeds();

    //Moteur
    vitesse = propRalenti * maxVitesse;

    digitalWrite(m1_in2, LOW);
    digitalWrite(m1_in1, HIGH);
    analogWrite(m1_pwm, vitesse);

    digitalWrite(m2_in2, LOW);
    digitalWrite(m2_in1, HIGH);
    analogWrite(m2_pwm, vitesse);
  }

  bool transitionDanger = distance < maxDanger;

  if(transitionDanger){

    state = DANGER;
    firstTime = 1;

    Serial.println("Sortie etat: Ralenti");
    return;
  }


  bool transitionNormal = distance >= maxRalentir;

  if(transitionNormal){

    state = NORMAL;
    firstTime = 1;

    Serial.println("Sortie etat: Ralenti");
    return;
  }

}

void dangerState(unsigned long ct){

  static bool firstTime = 1;
  static unsigned long lastTime = 0;

  if(firstTime){
    Serial.println("Entrée etat: Danger");
    firstTime = 0;

    vitesse = propNormal * maxVitesse;

    red = 255;
    green = 0;
    blue = 0;

    led.setColor(red, green, blue);
    led.show();

    digitalWrite(m1_in2, HIGH);
    digitalWrite(m1_in1, LOW);

    digitalWrite(m2_in2, HIGH);
    digitalWrite(m2_in1, LOW);
  }

  if(stateDanger == ARRET){
    arretState(ct);
    return;
  }

  if(stateDanger == RECUL){
    reculState(ct);
    return;
  }

  if(stateDanger == PIVOT){
    pivotState(ct);
    if(stateDanger != ARRET) return;
  }

  
  bool transitionRalentir = distance >= maxDanger && distance < maxRalentir;

  if(transitionRalentir){

    state = RALENTI;
    firstTime = 1;

    Serial.println("Sortie etat: Danger");
    return;
  }

  bool transitionNormal = distance >= maxRalentir;

  if(transitionNormal){

    state = NORMAL;
    firstTime = 1;

    Serial.println("Sortie etat: Danger");
  }

}

void arretState(unsigned long ct){
  static unsigned long exitTime;
  const int rate = 500;
  static bool firstTime = 1;


  if(firstTime){
    analogWrite(m1_pwm, 0);
    analogWrite(m2_pwm, 0);

    exitTime = ct + rate;
    firstTime = 0;
  } 

  bool transition = ct > exitTime;

  if (transition){
    stateDanger = RECUL;
    firstTime = 1;
  }
}

void reculState(unsigned long ct){
  static unsigned long exitTime;
  const int rate = 1000;
  static bool firstTime = 1;

  if(firstTime){
    analogWrite(m1_pwm, vitesse);
    analogWrite(m2_pwm, vitesse);

    exitTime = ct + rate;
    firstTime = 0;
  } 

  bool transition = ct > exitTime;

  if (transition){
    stateDanger = PIVOT;
    firstTime = 1;
  }
}

void pivotState(unsigned long ct){
  static unsigned long exitTime;
  const int rate = 1500;
  static bool firstTime = 1;

  if(firstTime){
    analogWrite(m1_pwm, 0);
    analogWrite(m2_pwm, vitesse);

    exitTime = ct + rate;
    firstTime = 0;
  } 

  bool transition = ct > exitTime;

  if (transition){
    stateDanger = ARRET;
    firstTime = 1;
  }
}

void rondeState(unsigned long ct){

  static unsigned long exitTime;
  const int rate = 2000;
  static bool firstTime = 1;

  if(firstTime){
    Serial.println("Entré etat: Ronde");
    
    red = 0;
    green = 255;
    blue = 0;

    exitTime = ct + rate;
    firstTime = 0;
  } 

  clignoteTask(ct);

  bool transition = ct > exitTime;

  if (transition){
    state = NORMAL;
    stateDanger = ARRET;
    firstTime = 1;
    Serial.println("Sortie etat: Ronde");
  }

}

void clignoteTask(unsigned long ct){

  static unsigned long lastTime = 0;
  static bool onOff = 1; 
  const int rate = 125;

  if(ct - lastTime < rate) return;

  lastTime = ct;

  if(onOff)
    led.setColor(red, green, blue );
  else
    led.setColor(0, 0, 0);

  led.show();

  onOff = !onOff;

}

void onLeds(){

  led.setColor(0, 0, 0);

  for(int i = minLed; i < maxLed; i++){
    led.setColorAt(i, red, green, blue);
  }

  led.show();
}

void manageState(unsigned long ct){

  switch(state){
    case NORMAL:
      normalState(ct);
      break;
    
    case RALENTI:
      ralentiState(ct);
      break;

    case DANGER:
      dangerState(ct);
      break;

    case RONDE:
      rondeState(ct);
      break;  
  }
  
}

void manageDanger(unsigned long ct){

  switch(stateDanger){
    case ARRET:
      arretState(ct);
      break;
    
    case RECUL:
      reculState(ct);
      break;

    case PIVOT:
      pivotState(ct);
      break;
  }
  
}

void setup() {
  
  Serial.begin(115200);
  led.setpin( LED_PIN );

}

void loop() {

  currentTime = millis();

  distance = retournerDistance(currentTime);

  manageState(currentTime);
  
}

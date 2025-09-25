#include <MeAuriga.h>
MeUltrasonicSensor ultraSensor(PORT_10);

#define ALL_LEDS 0
#define LEDNUM  12 // Auriga on-board light ring has 12 LEDs
#define LED_PIN 44

// on-board LED ring, at PORT0 (onboard)
MeRGBLed led( 0, LEDNUM );

int distance;
unsigned long currentTime;

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

  static bool firstTime = 1;
  static unsigned long lastTime = 0;

  if(firstTime){
    Serial.println("Entrée etat: Normal");
    firstTime = 0;
    vitesse = 0.7 * maxVitesse;

    onLeds(0, 255, 0);
  }


  digitalWrite(m1_in2, LOW);
  digitalWrite(m1_in1, HIGH);
  analogWrite(m1_pwm, vitesse);

  digitalWrite(m2_in2, LOW);
  digitalWrite(m2_in1, HIGH);
  analogWrite(m2_pwm, vitesse);

  bool transition = distance < 80;

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
    vitesse = 0.5 * maxVitesse;
    onLeds(0, 0, 255 );
  }

  digitalWrite(m1_in2, LOW);
  digitalWrite(m1_in1, HIGH);
  analogWrite(m1_pwm, vitesse);

  digitalWrite(m2_in2, LOW);
  digitalWrite(m2_in1, HIGH);
  analogWrite(m2_pwm, vitesse);
  

  bool transitionDanger = distance < 40;

  if(transitionDanger){

    state = DANGER;
    firstTime = 1;

    Serial.println("Sortie etat: Ralenti");
    return;
  }


  bool transitionNormal = distance >= 80;

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
  const int delayArret = 500;
  const int delayRecul = 1000;
  const int delayPivot = 1800;

  if(firstTime){
    Serial.println("Entrée etat: Danger");
    firstTime = 0;

    vitesse = 0.7 * maxVitesse;

    led.setColor(255, 0, 0 );
    led.show();

    digitalWrite(m1_in2, HIGH);
    digitalWrite(m1_in1, LOW);

    digitalWrite(m2_in2, HIGH);
    digitalWrite(m2_in1, LOW);
  }

  if(stateDanger == ARRET){
    Serial.print(ct - lastTime);
    Serial.println("\tARRET");
    if(ct - lastTime < delayArret) {
      analogWrite(m1_pwm, 0);
      analogWrite(m2_pwm, 0);
      return;
    }

    lastTime = ct;
    stateDanger = RECUL;
  }

  if(stateDanger == RECUL){
    Serial.print(ct - lastTime);
    Serial.println("\tRecul");
    if(ct - lastTime < delayRecul) {
      analogWrite(m1_pwm, vitesse);
      analogWrite(m2_pwm, vitesse);
      return;
    }

    lastTime = ct;
    stateDanger = PIVOT;
  }

  if(stateDanger == PIVOT){
    if(ct - lastTime < delayPivot) { 
      analogWrite(m1_pwm, 0);
      analogWrite(m2_pwm, vitesse);
      return;
    }

    lastTime = ct;
    stateDanger = ARRET;  
  }

  bool transitionRalentir = distance >= 40 and distance < 80;

  if(transitionRalentir){

    state = RALENTI;
    firstTime = 1;

    Serial.println("Sortie etat: Danger");
    return;
  }

  bool transitionNormal = distance >= 80;

  if(transitionNormal){

    state = NORMAL;
    firstTime = 1;

    Serial.println("Sortie etat: Danger");
  }

}

void onLeds(int red, int green, int blue){

  led.setColor(0, 0, 0);

  for(int i = minLed; i < maxLed; i++){
    led.setColorAt(i, red, green, blue);
  }

  led.show();
}

void rondeState(unsigned long ct){

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

void setup() {
  
  Serial.begin(115200);
  led.setpin( LED_PIN );

}

void loop() {
  // put your main code here, to run repeatedly:

  currentTime = millis();

  distance = retournerDistance(currentTime);

  manageState(currentTime);

  // Serial.print("Distance: ");
  // Serial.println(distance);
  // Serial.print("\tState: ");
  // Serial.println(state);
  
}

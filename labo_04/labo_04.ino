#include <MeAuriga.h>

#define BUZZER_PIN 45
#define LED_RING_PIN 44
#define PULSE 9
#define RATIO 39.267
#define CIRC_WHEEL 202.6

#define ALL_LEDS 0
#define LEDNUM  12 // Auriga on-board light ring has 12 LEDs
#define LED_PIN 44

MeRGBLed led(0, LEDNUM);
MeUltrasonicSensor ultraSensor(PORT_10);
MeGyro gyro(0, 0x69);

MeEncoderOnBoard encoderRight(SLOT1);
MeEncoderOnBoard encoderLeft(SLOT2);

float distance = 0;
float distanceParcourue = 0;
float target = 0;

const int lastValue = 400;
const int maxAngle = 360;

short vitesse = 150;
short vitessePivot = 80;
short vitesseRecul = 100;

short red = 0;
short blue = 0;
short green = 0; 

enum State {AVANCER, RECULER, GAUCHE, DROITE, AUTO, KLAXONNER, STOP};
State state = STOP;
State precedentState = STOP;

const short sizeClignotant = 3;
const short sizeArret = 5;

short ledsGauche[sizeClignotant] = {12, 1, 2};
short ledsDroite[sizeClignotant] = {4, 5, 6};
short ledsArret[sizeArret] = {7, 8, 9, 10, 11};

short currentSize;
short currentLeds[sizeArret];

unsigned long currentTime = 0;
bool debugMode = false;
bool isFromBLE = false;  // Indicateur de source des données

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

#pragma region updates

void gyroTask(unsigned long ct) {
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

#pragma region Leds-Buzzer

void onAllLeds(){
    led.setColor(red, green, blue);
    led.show();
}

void offLeds(){
  led.setColor(0, 0, 0);
  led.show();
}

void onLedsArray(){
    for( int i = 0; i < currentSize; i++){
        led.setColor(currentLeds[i], red, green, blue);
    }

    led.show();
}

void clignoteLeds(unsigned long ct, bool isAllLeds = 0){

    static unsigned long lastTime = ct;
    static bool onOff = 1; 
    const int rate = 600;

    if(ct - lastTime >= rate){
      onOff = !onOff;
      lastTime = ct;
    }

    if(onOff){

        if(isAllLeds)
            onAllLeds();
        else
            onLedsArray();

    }
    
    else
        offLeds();

    led.show();

}

void offBuzzer(){
  analogWrite(BUZZER_PIN, 0);
}

void beepTask(unsigned long ct){

    static unsigned long lastBeep = ct;
    static bool onOff = 1; 
    const int rate = 500;

    short klaxonPower = 180;

    if(ct - lastBeep >= rate){
      lastBeep = ct;
      onOff = !onOff;
    }

    if(onOff)
        analogWrite(BUZZER_PIN, klaxonPower);
    else
        offBuzzer();

}

#pragma endregion

#pragma region state

void avancerState(unsigned long ct){

    static bool firstTime = 0;

    if(precedentState == STOP)
        firstTime = 1;

    if(firstTime){
        firstTime = 0;

        offLeds();
        goStraight(vitesse, 1);
    }

    goStraight(vitesse);

}

void reculerState(unsigned long ct){

    static bool firstTime = 0;

    if(precedentState == STOP)
        firstTime = 1;

    if(firstTime){
        firstTime = 0;

        offLeds();

        red = 255;
        green = 255;
        blue = 255;

        currentSize = sizeArret;
        memcpy(currentLeds, ledsArret, currentSize * sizeof(short));
        
        onLedsArray();

        goStraight(-vitesseRecul, 1);
    }

    goStraight(-vitesseRecul);
    beepTask(ct);

}

void gaucheState(unsigned long ct){

    static bool firstTime = 0;

    if(precedentState == STOP)
        firstTime = 1;

    if(firstTime){
        firstTime = 0;

        offLeds();

        red = 255;
        green = 165;
        blue = 0;

        currentSize = sizeClignotant;
        memcpy(currentLeds, ledsGauche, currentSize * sizeof(short));

        encoderLeft.setMotorPwm(-vitessePivot);
        encoderRight.setMotorPwm(-vitessePivot); 
    }

    clignoteLeds(ct);

}

void droiteState(unsigned long ct){

    static bool firstTime = 0;

    if(precedentState == STOP)
        firstTime = 1;

    if(firstTime){
        firstTime = 0;

        offLeds();

        red = 255;
        green = 165;
        blue = 0;

        currentSize = sizeClignotant;
        memcpy(currentLeds, ledsDroite, sizeClignotant * sizeof(short));

        encoderLeft.setMotorPwm(vitessePivot);
        encoderRight.setMotorPwm(vitessePivot); 
    }

    clignoteLeds(ct);

}

void autoState(unsigned long ct){

    static bool firstTime = 1;
    static float position = 0;

    if(firstTime){
      firstTime = 0;

        red = 255;
        green = 255;
        blue = 0;

        position = distanceParcourue;

        goStraight(vitesse, 1);
    }

    goStraight(vitesse);
    clignoteLeds(ct, 1);

    serialMsg = "Distance à parcourir: " + (String) (target + position - distanceParcourue);

    bool transition = distanceParcourue - position > target;

    if(transition){
        precedentState = state;
        state = STOP;
        firstTime = 1;
    }

} 

void klaxonnerState(unsigned long ct){

    short klaxonPower = 200;

    static bool firstTime = 0;

    if(precedentState != STOP)
        firstTime = 1;

    if(firstTime){
        firstTime = 0;

        analogWrite(BUZZER_PIN, klaxonPower);
    }

}

void stopState(unsigned long ct){

    static bool firstTime = 1;

    if(precedentState != STOP){
        firstTime = 1;
        precedentState = STOP;
    }

    if(firstTime){
        firstTime = 0;

        offMotors();
        offLeds();
        offBuzzer();

        red = 255;
        green = 0;
        blue = 0;

        currentSize = sizeArret;
        memcpy(currentLeds, ledsArret, sizeArret * sizeof(short));
          
        onLedsArray();
    }

}

void stateManager(unsigned long ct) {

    switch (state){

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

        case AUTO:
            autoState(ct);
            break;

        case STOP:
            stopState(ct);
            break;
    
    }
    
}

#pragma endregion

#pragma region setup-loop

void setup() {

  Serial.begin(115200);
  led.setpin(LED_PIN);
  encoderConfig();
  gyro.begin();

}

void loop() {

  currentTime = millis();

  distanceParcourue = calculDistancePar(currentTime);

  stateManager(currentTime);

  serialTask(currentTime);

  gyroTask(currentTime);
  encodersTask(currentTime);

  
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

    precedentState = state;
    
    if(command.startsWith("DEBUG_MODE")){
      debugMode = !debugMode;
      Serial.print(F("Mode débogage : "));
      Serial.println(debugMode ? F("activé") : F("désactivé"));
    }
    
    else{

      char cmd = command[0];
      switch (cmd) {
        case 'F':   
          state = AVANCER;
          break;

        case 'B':   
          state = RECULER;
          break;

        case 'L':   
          state = GAUCHE;
          break;

        case 'R':   
          state = DROITE;
          break;
        
        case 'k':   
          state = KLAXONNER;
          break;

        case 'S':   
          state = STOP;
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

    if(command == "AUTO"){
      commandAuto(params);
    }
    else if(command == "pBack"){
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

void commandAuto(String params){
  
  precedentState = state;
  state = AUTO;
  target = params.toFloat();

}

#pragma endregion

#pragma region HELPERS

int countCharOccurrences(const String &str, char ch) {
  int count = 0;
  for (int i = 0; i < str.length(); i++) {
    if (str[i] == ch) {
      count++;
    }
  }
  return count;
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

void serialTask(unsigned long ct){

  static unsigned long lastTime = ct;
  const int rate = 200;

  if(ct - lastTime < rate) return;

  lastTime = ct;

  if (debugMode) {

    if(currentCommand != lastCommand && !currentCommand.startsWith("DEBUG_MODE") && !currentCommand.startsWith("S")){
      Serial.println("ici");
      Serial.print(F("Commande Reçu : "));
      Serial.println(currentCommand);
      Serial.print(F("Source : "));
      Serial.println(isFromBLE ? F("BLE") : F("Moniteur Série"));

    }
    
    if(state == AUTO){
      Serial.println(serialMsg);
    }
  }

}


#pragma endregion
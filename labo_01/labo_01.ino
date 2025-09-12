#define ROBOT 1

#if ROBOT 
  #include <MeAuriga.h>
  MeUltrasonicSensor ultraSensor(PORT_10);

#else
  #include <HCSR04.h>
  HCSR04 ultraSensor(8,9); //(trig_pin, echo_pin);

#endif

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREE_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3c

Adafruit_SSD1306 screen(SCREEN_WIDTH, SCREE_HEIGHT, &Wire, OLED_RESET);

float distance = 0;
unsigned long currentTime = 0;

const int lastValue = 400;

const int maxAngle = 360;
const int minActivation = 10;
const int maxActivation = 50;
int height;

enum sunState {ACTIVATION, ARRET};

sunState state = ARRET;

int retournerDistance(unsigned long ct){

  static unsigned long lastTime = 0;
  static int lastDistance = 0;
  const int delay = 250;

  if (ct - lastTime < delay) return lastDistance;

  lastTime = ct;

  #if ROBOT 
    int localDistance = ultraSensor.distanceCm();
  #else
    int localDistance = ultraSensor.dist();
  #endif

  lastDistance = localDistance;

  return distance;
}

void affichageSerie(unsigned long ct){

  static unsigned long lastTime = 0;
  const int delay = 250;

  if (ct - lastTime < delay) return;

  lastTime = ct;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println("cm");

}

void drawSun(int h){

  int angle = 45;
  int rayon = 9;

  int x = screen.width()-(rayon * 2);
  int y = 2*rayon + constrain(h, 0, screen.height() - 4*rayon);

  screen.fillCircle(x, y, rayon, SSD1306_WHITE);

  for (int i = 0; i < maxAngle; i += angle) {
    double angleRad = i * PI/180;

    int x1 = x + cos(angleRad) * (rayon + 2);
    int y1 = y + sin(angleRad) * (rayon + 2);
    int x2 = x + cos(angleRad) * (rayon + 6);
    int y2 = y + sin(angleRad) * (rayon + 6);
    screen.drawLine(x1, y1, x2, y2, SSD1306_WHITE);
  }

}

void drawHouse(){

  screen.drawTriangle(
      screen.width()/8  , 1,
      0, screen.height()/4,
      screen.width()/4, screen.height()/4, SSD1306_WHITE);

  screen.drawRect(3, screen.height()/4, (screen.width()/4 - 6), 20, SSD1306_WHITE);
  screen.drawRect(screen.width()/8, (screen.height()/4 + 10), 8, 10, SSD1306_WHITE);

  screen.display();
}

void drawName(){

  screen.setTextSize(2);
  screen.setTextColor(SSD1306_WHITE);       
  screen.setCursor(1,50);             
  screen.println(F("Pharel"));

  screen.display();
}

void verifyState(){
  if (distance >= minActivation && distance <= maxActivation){
    state = ACTIVATION;
    return;
  }
  
  state = ARRET;
}

void manageState(){

  static int lastHeight = 0;

  switch(state){
    case ARRET:
      height = lastHeight;
      break;
    
    case ACTIVATION:
      height = map(distance, minActivation, maxActivation, 0, screen.height());
      lastHeight = height;
      break;
  }

}

void draw(unsigned long ct){

  static unsigned long lastTime = 0;
  const int delay = 100;

  if (ct - lastTime < delay) return;

  lastTime = ct;

  if(state == ACTIVATION){
    screen.clearDisplay();
  }

  drawHouse();

  drawName();

  drawSun(height);

  screen.display();
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  if(!screen.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("Échec d'allocation SSD1306"));
    for(;;); // Ne pas continuer, boucler indéfiniment
  }

  screen.display();
  delay(2000);

  screen.clearDisplay();

}


void loop() {
  // put your main code here, to run repeatedly:

  currentTime = millis();

  distance = retournerDistance(currentTime);

  affichageSerie(currentTime);

  verifyState();

  manageState();

  draw(currentTime);

}

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <math.h>

// Initialisation du capteur TCS34725
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

#define MOTOR_DIR 4
#define MOTOR_PULSE	5
#define MOTOR_ENABLE 15
#define batteryPin A0
#define laser1 16
#define laser2 17
long step_time = 250;
int numColors = 4;

SoftwareSerial esp01(7, 6); // RX, TX vers ESP8266

volatile int count_depart = 0;
volatile int count_fin = 0;

bool tapis_en_marche = false;
unsigned long lastSent = 0;
const float ratio = 4.167;
String color;
long batteryCharge;
int increment = 0;

// Structure pour stocker les valeurs de référence
struct ColorRef {
  String name;
  float r_ref;
  float g_ref;
  float b_ref;
};

// Tableau des couleurs de référence 
ColorRef colors[] = {
  {"red", 0.489, 0.257, 0.254},
  {"green", 0.252, 0.417, 0.331},
  {"blue", 0.217, 0.406, 0.377},
  {"yellow", 0.367, 0.416, 0.217}
};

// === Interruptions ===
void departTapis() {
  //count_depart++;
  Serial.println("Faisceau début coupé !");
  tapis_en_marche = true;
  digitalWrite(13, HIGH); // Simule démarrage moteur
}

void finTapis() {
  //count_fin++;
  Serial.println("Faisceau fin coupé !");
  tapis_en_marche = false;
  digitalWrite(13, LOW); // Simule arrêt moteur
}




void senseBatteryCharge(){
  // simulatedBattery = 95 ;
  long sum = 0;
  for(int i=0; i<5; i++){
    sum += analogRead(A0);
  }
  batteryCharge = (ratio * map(sum/5, 0, 1023, 0, 4)*100)/16.8;
  Serial.println(batteryCharge);
}


String DetectedColor(){
  String detectedColor ;
  uint16_t r, g, b, c;
  float r_norm, g_norm, b_norm;

  // Lire les données brutes du capteur
  tcs.getRawData(&r, &g, &b, &c);

  // Normaliser les valeurs RVB
  float sum = r + g + b;
  if (sum == 0) sum = 1; // Éviter la division par zéro
  r_norm = r / sum;
  g_norm = g / sum;
  b_norm = b / sum;

  // Trouver la couleur la plus proche
  String detectedColor = "Inconnue";
  float minDistance = 1500.0; // Valeur initiale élevée
  for (int i = 0; i < numColors; i++) {
    // Calculer la distance euclidienne
    float distance = sqrt(
      pow(r_norm - colors[i].r_ref, 2) +
      pow(g_norm - colors[i].g_ref, 2) +
      pow(b_norm - colors[i].b_ref, 2)
    );
       if (distance < minDistance) {
        minDistance = distance;
        detectedColor = colors[i].name;
    }
   
  }

  // Vérifier le seuil de distance et l'intensité lumineuse
  if (minDistance > 0.3 || c < 900) {
    detectedColor = "Inconnue ";
  }

  return detectedColor;

}

void light_led(String color){
  if(color == "red"){
    digitalWrite(9, HIGH); delay(500); digitalWrite(9, LOW);
  }
  else if(color == "blue"){
    digitalWrite(11, HIGH); delay(500); digitalWrite(11, LOW);
  }
  else if(color == "green"){
    digitalWrite(12, HIGH); delay(500); digitalWrite(12, LOW);
  }
  else if(color == "yellow"){
    digitalWrite(10, HIGH); delay(500); digitalWrite(10, LOW);
  }
  else {
    digitalWrite(9, LOW);  //delay(200);
    digitalWrite(10, LOW); //delay(200);
    digitalWrite(11, LOW); //delay(200);
    digitalWrite(12, LOW); //delay(200);
  }
}

void setup() {
  Serial.begin(9600);
  if (tcs.begin()) {
    Serial.println("Capteur TCS34725 détecté !");
  } else {
    Serial.println("Erreur : capteur TCS34725 non détecté. Vérifiez les connexions !");
    //while (1); // Boucle infinie en cas d'erreur
  }
  pinMode(2, INPUT);  // Faisceau départ
  pinMode(3, INPUT);  // Faisceau fin
  pinMode(13, OUTPUT); // LED moteur simulé
  pinMode(8, OUTPUT);       digitalWrite(8, HIGH);
  //esp01.begin(9600);

	pinMode(MOTOR_DIR, OUTPUT);       digitalWrite(MOTOR_DIR, LOW);
	pinMode(MOTOR_PULSE, OUTPUT);	 		digitalWrite(MOTOR_PULSE, LOW);
	pinMode(MOTOR_ENABLE, OUTPUT); 		digitalWrite(MOTOR_ENABLE, HIGH);
  // Serial.println("Moto");
  attachInterrupt(digitalPinToInterrupt(2), departTapis, RISING);
  attachInterrupt(digitalPinToInterrupt(3), finTapis, RISING);

  digitalWrite(13, LOW);
  Serial.println("Système de tri initialisé.");

  pinMode(9, OUTPUT);       digitalWrite(9, LOW);  //delay(200);
  pinMode(10, OUTPUT);      digitalWrite(10, LOW); //delay(200);
  pinMode(11, OUTPUT);      digitalWrite(11, LOW); //delay(200);
  pinMode(12, OUTPUT);      digitalWrite(12, LOW); //delay(200);

  pinMode(laser1, OUTPUT);      digitalWrite(laser1, HIGH);
  pinMode(laser2, OUTPUT);      digitalWrite(laser2, HIGH);
}

void loop() {
  digitalWrite(MOTOR_ENABLE, HIGH);
	if(tapis_en_marche){
    delay(250);
    digitalWrite(MOTOR_ENABLE, LOW);
    Serial.println(tapis_en_marche);
    while(tapis_en_marche){
      digitalWrite(MOTOR_PULSE, HIGH);    delayMicroseconds(step_time);
      digitalWrite(MOTOR_PULSE, LOW);     delayMicroseconds(step_time);
    }
    digitalWrite(MOTOR_ENABLE, HIGH);
    delay(200);
    color = DetectedColor();
    senseBatteryCharge();
    delay(250);
    // Envoie des données collectées.
		String data = color + "#" + String(batteryCharge);
    esp01.println(data);
    delay(25);
    while(!esp01.available()){}

    if(esp01.available()){
      delay(100);
      String data = esp01.readStringUntil('\n');
      Serial.print("Réponse ESP : ");
      Serial.println(data);
      delay(20);    esp01.flush();    
    }
    delay(150);
    light_led();
  }
	else return ;
}


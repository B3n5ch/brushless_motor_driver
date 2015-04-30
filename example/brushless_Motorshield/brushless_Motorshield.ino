#include "Brushless_Motorshield.h"

void setup() {
  Serial.begin(115200);
  Brushless_Motorshield::init();
}


void loop() {
  //float vel = analogRead(A0)/3.0;
  //Brushless_Motorshield::setVelocity(vel);
  float vel = 150.0;
  Brushless_Motorshield::setVelocity(vel);
  
  float current_velocity = Brushless_Motorshield::getVelocity();
  Serial.println("speed");
  Serial.println(vel);
  Serial.println(current_velocity);

  delay(100); 
}


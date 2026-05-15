int r_led = 52;    
int g_led = 50;     

int switchPin = 8;  // Microswitch NO connected here 

void setup() { 

  pinMode(r_led, OUTPUT); 

  pinMode(g_led, OUTPUT); 

  pinMode(switchPin, INPUT); 

} 

void loop() { 

  int switchState = digitalRead(switchPin); 

  if (switchState == HIGH) { 

    analogWrite(r_led, 0); 

    analogWrite(g_led, 255); 

  } else { 


    // Lever pressed: LED green 
    analogWrite(r_led, 0); 

    analogWrite(g_led, 255); 

  } 

} 

 
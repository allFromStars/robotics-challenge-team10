int r_led = 52;    
int g_led = 50;     

int switchPin = 8;

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

    analogWrite(r_led, 0); 

    analogWrite(g_led, 255); 

  } 

} 

 

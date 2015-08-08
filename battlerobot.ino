#include <PPMrcIn.h>
#include <Statistic.h>
#include <Servo.h>

//Weapon
#define PIN_SERVO 3      // Pin de cotrol del servo
Servo servo;
int sPos;                //Posicion del servo

//Receiver
#define SPEED A5
#define DIRECTION A4
#define WEAPON A3
#define ARMED A2 
Channel c1;
Channel c2;
Channel c3;
Channel c4;
int v1, v2, v3, v4;
boolean robotArmed;

//Motores
#define ENA 5             // Potencia motor A
#define IN1 11            // Control 1 motor A
#define IN2 10            // Control 2 motor A
#define ENB 6             // Potencia motor B
#define IN3 9             // Control 1 motor B
#define IN4 8             // Control 2 motor A


void setup() {
  Serial.begin(9600);
  
  robotArmed = false;
   
  //Motores
  pinMode(ENA, OUTPUT);     
  pinMode(IN1, OUTPUT);     
  pinMode(IN2, OUTPUT);     
  pinMode(ENB, OUTPUT);     
  pinMode(IN3, OUTPUT);     
  pinMode(IN4, OUTPUT);
  
  //Receiver
  pinMode (SPEED, INPUT);
  pinMode (DIRECTION, INPUT);
  pinMode (WEAPON, INPUT);
  pinMode (ARMED, INPUT);
  c1.init(1,SPEED);
  c2.init(1,DIRECTION);
  c3.init(1,WEAPON);
  c4.init(1,ARMED);
  
  //Servo head
  servo.attach(PIN_SERVO);
  sPos=0;
  servo.write(sPos);
  
  Serial.println("Ready:");
}

void loop() {
  //delay(20);
  readReceiver();
  printReceiver();
  if (robotArmed){
    motor(v1,v2);
    weapon(v3); 
  }
  else{
    motorStop();  
  }
}

void readReceiver(){
  c1.readSignal();
  c2.readSignal();
  c3.readSignal();
  c4.readSignal();
  v1 = trimValue(map(c1.getSignal(),835,1715,-255,255),-255,255);
  v2 = trimValue(map(c2.getSignal(),835,1715,-255,255),-255,255);
  v3 = trimValue(map(c3.getSignal(),835,1715,0,255),0,255);
  v4 = trimValue(map(c4.getSignal(),835,1715,0,255),0,255);
  if (v4>200){robotArmed=true;}else{robotArmed=false;}
}

int trimValue(int val, int top, int bot){
  int ret = val;
  if (val>top){
    val=top;
  }
  else if (val<bot) {
    val=bot;
  }
  return ret;
}

void printReceiver(){
  Serial.print(v1);
  Serial.print(" - ");
  Serial.print(v2);
  Serial.print(" - ");
  Serial.print(v3);
  Serial.print(" - ");
  Serial.println(v4);
}

void motor(int speed, int direction){
   if (speed>-25 && speed <25){
     //stick en el centro
     if (direction>-25 && direction <25){
       motorStop();
     }
     //giro sobre si mismo
     else{
       //giro derecha sobre si mismo
       if (direction>0){
         analogWrite(ENA, abs(direction));
         digitalWrite(IN1, HIGH);   
         digitalWrite(IN2, LOW);   
         analogWrite(ENB, abs(direction));   
         digitalWrite(IN3, LOW);   
         digitalWrite(IN4, HIGH);   
       }
       //giro izq sobre si mismo
       else if (direction < 0){
         analogWrite(ENA, abs(direction));
         digitalWrite(IN1, LOW);   
         digitalWrite(IN2, HIGH);   
         analogWrite(ENB, abs(direction));   
         digitalWrite(IN3, HIGH);   
         digitalWrite(IN4, LOW);   
       }  
     }
   }
   //giros suaves adelante
   else if (speed>0){
     //giro derecha suave
     if (direction>0){
       analogWrite(ENA, speed-abs(direction));
       digitalWrite(IN1, LOW);   
       digitalWrite(IN2, HIGH);   
       analogWrite(ENB, speed);   
       digitalWrite(IN3, LOW);   
       digitalWrite(IN4, HIGH);
     }
     //giro izq suave
     else {
       analogWrite(ENA, speed);
       digitalWrite(IN1, LOW);   
       digitalWrite(IN2, HIGH);   
       analogWrite(ENB, speed-abs(direction));   
       digitalWrite(IN3, LOW);   
       digitalWrite(IN4, HIGH);
     }
  }
  //giros suaves atras
  else{
    //giro derecha suave
     if (direction>0){
       analogWrite(ENA, abs(speed));
       digitalWrite(IN1, HIGH);   
       digitalWrite(IN2, LOW);   
       analogWrite(ENB, abs(speed)-abs(direction));   
       digitalWrite(IN3, HIGH);   
       digitalWrite(IN4, LOW);
     }
     //giro izq suave
     else {
       analogWrite(ENA, abs(speed)-abs(direction));
       digitalWrite(IN1, HIGH);   
       digitalWrite(IN2, LOW);   
       analogWrite(ENB, abs(speed));   
       digitalWrite(IN3, HIGH);   
       digitalWrite(IN4, LOW);
     }  
  }
}

void motorStop(){
   analogWrite(ENA, 0);
   digitalWrite(IN1, LOW);   
   digitalWrite(IN2, LOW);   
   analogWrite(ENB, 0);   
   digitalWrite(IN3, LOW);   
   digitalWrite(IN4, LOW);  
}

void weapon(int pos){
  servo.write(pos);
}

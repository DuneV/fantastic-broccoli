#include <ros.h>
#include <std_msgs/String.h>

int in1 = 8;  // Pin que controla el sentido de giro Motor A
int in2 = 9;  // Pin que controla el sentido de giro Motor A
int EnA = 5; // Pin que controla la velocidad del  Motor A

int in3 = 7;
int in4 = 6;
int EnB = 12;


String orden;

ros::NodeHandle nh;
std_msgs::String tecla;

void subscriberCallback(const std_msgs::String& tecla) {
  orden = tecla.data;
}

ros::Subscriber<std_msgs::String> teclas_subscriber("/teclas", &subscriberCallback);

void setup ()
{
  pinMode(in1, OUTPUT);    // Configura  los pines como salida
  pinMode(in2, OUTPUT);
  pinMode(EnA, OUTPUT);
  pinMode(in3, OUTPUT);    // Configura  los pines como salida
  pinMode(in4, OUTPUT);
  pinMode(EnB, OUTPUT);
  pinMode(13,OUTPUT);

  nh.initNode();
  nh.subscribe(teclas_subscriber);
}
void loop()
{
 if(orden == 'w')
  {
    adelante();
    nh.spinOnce();
    }
  if(orden == 'd')
  {
    giroDerecha();
    nh.spinOnce();
    }
  if(orden == 'a')
  {
    giroIzquierda();
    nh.spinOnce();
    }
  if(orden == 's')
  {
    atras();
    nh.spinOnce();
    }
  else{
    detener();
    nh.spinOnce();
    }
  nh.spinOnce(); 
  delay(100);
}


void adelante(){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(EnA, 100); //255
  
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(EnB, 100);
}

void atras(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(EnA, 100);
  
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(EnB, 100);
}

void giroIzquierda(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(EnA, 0);
  
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(EnB, 100);
}

void giroDerecha(){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(EnA, 100);
  
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(EnB, 0);
}

void detener(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(EnA, 0);
  
digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(EnB, 0);
}

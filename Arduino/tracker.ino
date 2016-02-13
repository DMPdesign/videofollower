/*************************************/
/*	       TRACKER               */
/*                                   */
/* Dazzi Martino,                    */
/* Mele Leandro Julian,              */
/* Pilotto Alessandro                */
/*                                   */
/* Università degli Studi di Udine,  */
/* Via delle Scienze 208,            */
/* Udine, Italy                      */
/*                                   */
/*************************************/


#include <Servo.h>

Servo Xservo;
Servo Yservo;

/* Serial port configuration */
const long BAUDRATE = 9600;

/* I/O Pin Configuration */
const int X_SERVO_PIN = 8;
const int Y_SERVO_PIN = 9;
const int GREEN_LED   = 4;
const int YELLOW_LED  = 5;
const int RED_LED     = 6;

/* Available Moves */
const char UP               = 'W';
const char DOWN             = 'S';
const char LEFT             = 'A';
const char RIGHT            = 'D';
const char TO_DEFAULT       = 'C';
const char SPEED_1          = '1';
const char SPEED_2          = '2';
const char SPEED_3          = '3';
const char SPEED_4          = '4';
const char SPEED_5          = '5';
const char GET_XPOS         = 'X';
const char GET_YPOS         = 'Y';
const char GET_LEFT_LIMIT   = 'L';
const char GET_RIGHT_LIMIT  = 'R';
const char GET_UPPER_LIMIT  = 'U';
const char GET_LOWER_LIMIT  = 'G';

/* Boundaries */
/*
const int LEFT_LIMIT  = 100; 
const int RIGHT_LIMIT = 0;
const int UPPER_LIMIT = 50;
const int LOWER_LIMIT = 0;
*/

int LEFT_LIMIT  = 140; 
int RIGHT_LIMIT = 40;
int UPPER_LIMIT = 50;
int LOWER_LIMIT = 0;


/* Default Positions and Increment*/
const int DEFAULT_X = 90;
const int DEFAULT_Y = 10;
int INCREMENT = 1;

/* Global Variables */
const int DELAY = 15;
String inputString = ""; 
boolean stringComplete = false;
int Xposition = DEFAULT_X;
int Yposition = DEFAULT_Y;

/* Setup Function */
void setup ()
{
  /* Serial port initialization */
  Serial.begin(BAUDRATE);
  inputString.reserve(200);
  
  /* LED initialization */
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(YELLOW_LED, HIGH);
  digitalWrite(RED_LED, HIGH);

  /* Servos initialization */
  Xservo.attach(X_SERVO_PIN);
  Yservo.attach(Y_SERVO_PIN);
  Xservo.write(Xposition);
  Yservo.write(Yposition);
  
  delay(40*DELAY);
  
  /* End LED Test */
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(RED_LED, LOW);
}

/* Loop Function */
void loop ()
{
  char move = 0;
  
  digitalWrite(GREEN_LED, HIGH);
  
  /* Get the next move through serial port */
  if (stringComplete) 
  {
    for (int i = 0; i < inputString.length(); i++)
    {
      if ((UP == (char)inputString[i]) || (DOWN == (char)inputString[i]) || (LEFT == (char)inputString[i]) || (RIGHT == (char)inputString[i])
          || (TO_DEFAULT == (char)inputString[i]) || (SPEED_1 == (char)inputString[i]) || (SPEED_2 == (char)inputString[i]) || (SPEED_3 == (char)inputString[i])
          || (SPEED_4 == (char)inputString[i]) || (SPEED_5 == (char)inputString[i]) || (GET_XPOS == (char)inputString[i]) || (GET_YPOS == (char)inputString[i])
          || (GET_RIGHT_LIMIT == (char)inputString[i]) || (GET_LEFT_LIMIT == (char)inputString[i]) || (GET_UPPER_LIMIT == (char)inputString[i]) 
          || (GET_LOWER_LIMIT == (char)inputString[i]))
        move = (char)inputString[i];
    }
    
    inputString = "";
    stringComplete = false;
  }
  
  /* Decide what to do */
  switch(move)
  {
    case UP:              if(Yposition <= (UPPER_LIMIT - INCREMENT))
                          {
                            Yposition += INCREMENT;
                            Yservo.write(Yposition);
                            delay(DELAY);
                          }
                          break;
    
    case DOWN:            if(Yposition >= (LOWER_LIMIT + INCREMENT))
                          {
                            Yposition -= INCREMENT;
                            Yservo.write(Yposition);
                            delay(DELAY);
                          }
                          break;
    
    case RIGHT:           if(Xposition >= (RIGHT_LIMIT + INCREMENT))
                          {
                            Xposition -= INCREMENT;
                            Xservo.write(Xposition);
                            delay(DELAY);
                          }
                          break;
    
    case LEFT:            if(Xposition <= (LEFT_LIMIT - INCREMENT))
                          {
                            Xposition += INCREMENT;
                            Xservo.write(Xposition);
                            delay(DELAY);
                          }
                          break;
          
    case TO_DEFAULT:      digitalWrite(GREEN_LED, LOW);
                          digitalWrite(YELLOW_LED, LOW);
                          digitalWrite(RED_LED, HIGH);
                      
                          while(Xposition > DEFAULT_X)
                          {
                            Xposition--;
                            Xservo.write(Xposition);
                            delay(DELAY);
                          }
                          while(Xposition < DEFAULT_X)
                          {
                            Xposition++;
                            Xservo.write(Xposition);
                            delay(DELAY);
                          }
                          while(Yposition > DEFAULT_Y)
                          {  
                            Yposition--;
                            Yservo.write(Yposition);
                            delay(DELAY);
                          }
                          while(Yposition < DEFAULT_Y)
                          {
                            Yposition++;
                            Yservo.write(Yposition);
                            delay(DELAY);
                          }
                      
                          digitalWrite(RED_LED, LOW);
                          digitalWrite(GREEN_LED, HIGH);
                     
                          break;
                      
    case SPEED_1:         INCREMENT = 1;
                          break;
    
    case SPEED_2:         INCREMENT = 2;
                          break;
                      
    case SPEED_3:         INCREMENT = 3;
                          break;
                      
    case SPEED_4:         INCREMENT = 4;
                          break;
                      
    case SPEED_5:         INCREMENT = 5;
                          break;
                      
    case GET_XPOS:        Serial.write(Xposition - DEFAULT_X);
                          break;
                      
    case GET_YPOS:        Serial.write(Yposition - DEFAULT_Y);
                          break;
                      
    case GET_LEFT_LIMIT:  Serial.write(LEFT_LIMIT - DEFAULT_X);
                          break;
    
    case GET_RIGHT_LIMIT: Serial.write(RIGHT_LIMIT - DEFAULT_X);
                          break;
    
    case GET_UPPER_LIMIT: Serial.write(UPPER_LIMIT - DEFAULT_Y);
                          break;
    
    case GET_LOWER_LIMIT: Serial.write(LOWER_LIMIT - DEFAULT_Y);
                          break;
              
    default:              break;
          
  }
  
  if((UPPER_LIMIT < (Yposition + INCREMENT)) || (LOWER_LIMIT > (Yposition-INCREMENT)) ||
     (LEFT_LIMIT < (Xposition + INCREMENT)) || (RIGHT_LIMIT > (Xposition-INCREMENT)))
  {
    digitalWrite(YELLOW_LED, HIGH);
  }
  else
  {
    digitalWrite(YELLOW_LED, LOW);
  }
}

/* This function is called whenever a new serial transmission occurs */
void serialEvent() 
{
   while (Serial.available()) 
   {
      char inChar = (char)Serial.read();
      inputString += inChar;
      if (inChar == '\n')
      {
         stringComplete = true;
      }
   }
}


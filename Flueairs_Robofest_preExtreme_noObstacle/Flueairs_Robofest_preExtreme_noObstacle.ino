#define sensorNum 8          //always even

#define maxSpeed 255//180  //180 //200
#define sSpeed 255
#define linegapSpeed 255

int blackLimit[sensorNum];
int digitalReading[sensorNum];

const int motorPin1=10,motorPin2=9;        //right motor
const int motorPin3=6,motorPin4=5;       //left motor
const int initIR=1;

float error, prevError=0;
float mappedValue, targetValue = 8;      

float safety=0.35;

float kp=42.5;   //26.7G //26        //38 //37  //39                //45 IF DOESN'T WORK
float kd=65;  //57.3  //50G //50    //67             //70  //42 //87

int rotationSpeed=150;

int motorResponse;
float correction;

int leftSpeed,rightSpeed;

float lastAct;
int time=3;

int allBlack=0;
int leftBlack=0;
int rightBlack=0;

int frontIR=1;                      //change
int leftIR=2;                          //change
int rightIR=3;                         //change

int led = 13;
int insideLoop=0;
//int caveSpeed=;
//int turnSpeed=;

int left=0,right=0; 
int preLeft=0, preRight=0;

int contiBlack = 0;
int stopTrigger = 6 ; //2
int startTrigger = 3;
int extremeCounter = 0;
int extremeTrigger =12;
int preExtremeCounter = 0;
int preExtremeTrigger = 20;

void setup()
{
  //initialize IR pins
  for(int i = initIR; i < sensorNum; i++)          //A0 is leftmost Ir
  {
    pinMode(A0 + i, INPUT);
  }

  pinMode(frontIR,INPUT);
  pinMode(rightIR,INPUT);
  pinMode(leftIR,INPUT);

  pinMode(led, OUTPUT);

  //initialize motor pins
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT); 
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  delay(1000);
  Serial.begin(9600);
  
  Serial.println("Calibrating");
  calibration();
  
  
}





void loop()
{

if(!left && !right)
 digitalWrite(led, LOW);
else
  digitalWrite(led, HIGH);
//sensorRead();
sensorMapping();
Serial.print(" ");
if(stopTrigger < contiBlack) 
{ 
  while(allBlack == 1) {brake(); startTrigger = 0; sensorMapping();}
   //if(mappedValue == 100) {delay(100); sensorMapping(); if(mappedValue == 100) brake();}
}

for(int i = 0; i < sensorNum; i++)
{
  //Serial.print(analogRead(A0+i));
  /*if(analogRead(A0+i)<blackLimit[i])
     Serial.print("1");
  else
     Serial.print("0");   
  Serial.print(" ");*/ 
}

  Serial.print("left: ");
  Serial.print(left);
  Serial.print(" right: ");
  Serial.print(right);
  Serial.println();
 /*
  while(!digitalRead(frontIRpin))    // if reading 0 object is close
  {
  	brake();
  }
 */ 
  if(mappedValue!=100 || (contiBlack > stopTrigger && allBlack == 1))     // add conditions                    
    {
      pid();      
      motor(leftSpeed,rightSpeed);
      
    }
  else                                 // all white
    {
        brake();
        //delay(10);
        //delayMicroseconds();
        //dronzer stop
//        motor(-50,-50);
//        delay(20);
        
        if((left==1)&&(right==0))
        {
          ACRotate(rotationSpeed);   
          while(mappedValue==100) sensorMapping();
          while(digitalReading[initIR+2]!=1)
          {
            sensorMapping();
          pid();
          motor(leftSpeed,rightSpeed);
          }      
           left=0;                                     //changed
        }
        else if((right==1)&&(left==0))
        {
          CRotate(rotationSpeed);   
          while(mappedValue==100) sensorMapping();
          while(digitalReading[sensorNum-3]!=1)
          {
            sensorMapping();
          pid();
          motor(leftSpeed,rightSpeed);
          }
          right=0;                                     //changed
        }
        else
        {
          motor(linegapSpeed,linegapSpeed);
          while(mappedValue==100) sensorMapping();
          pid();
          motor(leftSpeed,rightSpeed);
        }

      
    }
      Serial.print("left");
      Serial.print(left);
      Serial.print(" Right: ");    
      Serial.print(right);

Serial.println();
}


void sensorMapping()
{
int sum=0,coun=0;





 
 for (int i = initIR; i <sensorNum; i++)
  { 
    if (analogRead(A0+i) < blackLimit[i])              //A0 is leftmost IR 
     { 
      
      sum += i*2;
      coun++;
      digitalReading[i]=1;
    }
    else digitalReading[i]=0;
  }
 

   //if((left==1)&&(right==1)) insideLoop=1;             //loop detection
   
   if(coun!=0){                   
  mappedValue = sum / coun;
   }
   else mappedValue=100;         //all White detection 

    //reset corner IRs
  if(extremeCounter > extremeTrigger)
  {
    left = 0;
    right = 0;
  }
  
  //reset secondary corner IRs
  if(preExtremeCounter > preExtremeTrigger)
   {
     preLeft = 0;
     preRight = 0;
   }
  if(digitalReading[initIR]||digitalReading[sensorNum-1])  //if left or right gives black saves their value
   {
     left=digitalReading[initIR];
     right=digitalReading[sensorNum-1];
     extremeCounter = 0; //reset counter
   }
   
   else 
   {
     if(mappedValue != 100)
     extremeCounter++;
   }
   
   if(digitalReading[1] || digitalReading[sensorNum-2])
   {
     preLeft = digitalReading[1];
     preRight = digitalReading[sensorNum-2];
     preExtremeCounter = 0;
   }
   
   else
   {
     if(mappedValue != 100)
     preExtremeCounter++;
   }

   if(coun==sensorNum-initIR) 
   {
    contiBlack++;
   
   allBlack=1;
   
 
   }
   else if(startTrigger == 0 && mappedValue != 7 && coun != sensorNum-initIR)
   {
    allBlack = 1;
   }

    else
    {
    allBlack=0; 
    contiBlack = 0;  
    }
   
}


void pid()
{
  
  error=targetValue-mappedValue;
  correction=(kp*error)+(kd*(error-prevError));
  prevError=error;
  motorResponse=(int)correction;
 
 if(motorResponse>maxSpeed) motorResponse=maxSpeed;
 
if(motorResponse<-maxSpeed) motorResponse=-maxSpeed;

   if(motorResponse>0)
  {
    rightSpeed=maxSpeed;
    leftSpeed=maxSpeed-motorResponse;
  }
  else if(motorResponse<0)
  {
    rightSpeed=maxSpeed+ motorResponse;
    leftSpeed=maxSpeed;
  }
  //non aggressive straight
  else
  {
    rightSpeed= sSpeed;
    leftSpeed= sSpeed;
  }

}


void motor(int left, int right)
{
  
  if(right>0)
  {
  analogWrite(motorPin1,right);
  analogWrite(motorPin2,0);
  }
  else
  {
    analogWrite(motorPin1,0);
    analogWrite(motorPin2,-right);
  }

  if(left>0)
  {
  analogWrite(motorPin3,left);
  analogWrite(motorPin4,0);
  }
  else
  {
   analogWrite(motorPin3,0);
   analogWrite(motorPin4,-left); 
  }

 }

void CRotate(int rotateSpeed)
{
  analogWrite(motorPin1,0);
  analogWrite(motorPin2, rotateSpeed);
  analogWrite(motorPin3, rotateSpeed);
  analogWrite(motorPin4,0);

}

void ACRotate(int rotateSpeed)
{
  analogWrite(motorPin1,rotateSpeed);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0);
  analogWrite(motorPin4,rotateSpeed);

}

void brake(void)
{
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0);
  analogWrite(motorPin4, 0);
}

 //auto calibration
void calibration()
{
  CRotate(180);
  float upSum = 0,lowSum = 0;
  int sensorArray[sensorNum][2];

  for(int i = initIR; i < sensorNum; i++)
    {
      sensorArray[i][0] = analogRead(A0+i);
      sensorArray[i][1] = analogRead(A0+i);
    }
 

  int loopCounter = (int)(time * 1000 / 2.5);  
  while(loopCounter)
  {
    for(int i = initIR; i < sensorNum; i++)
    {
      if(analogRead(A0+i)<sensorArray[i][0]) sensorArray[i][0]=analogRead(A0+i);
      if(analogRead(A0+i)>sensorArray[i][1]) sensorArray[i][1]=analogRead(A0+i);
    }
  loopCounter--;

  }

 for(int i=initIR; i < sensorNum; i++)
  blackLimit[i] = (int)(sensorArray[i][0] + safety * (sensorArray[i][1] - sensorArray[i][0]));

  brake();
  delay(1000);

}

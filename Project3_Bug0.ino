 /***********************************************************************
 * Project 3 - Bug0 
 * Ramiro Velazquez, Uthiria Mohan, Tariq Kottai
 * 05/04/2016
 * 
 * 
 * 

 ***********************************************************************/

  #include <RedBot.h>  // This line "includes" the RedBot library into your sketch.
  // Provides special objects, methods, and functions for the RedBot.

  RedBotSensor left = RedBotSensor(A3); // initialize a sensor object on A3
  RedBotSensor center = RedBotSensor(A6); // initialize a sensor object on A6
  RedBotSensor right = RedBotSensor(A7); // initialize a sensor object on A7
  #define LINETHRESHOLD 850 //Threshold to detect black line 
    
  RedBotMotors motors; // Instantiate the motor control object. This only needs to be done once.
  RedBotAccel accel;     //declare accelerometer object
  
  RedBotEncoder encoder = RedBotEncoder(A2,10); // turn encoders on 
  int countsPerRot = 192;   // 4 pairs of N-S x 48:1 gearbox = 192 ticks per wheel rev  
  float wheelDiam = 2.56;  // diam = 65mm / 25.4 mm/in
  float wheelCirc = PI*wheelDiam;  // Redbot wheel circumference = pi*D inches (8.04 in)
  float Wheel_Length = 6; 
  
  float x_g =0, y_g = 14; //goal point   
  
  float theta = PI/2; //initialize variable to keep track of spatial angle 
  float x=0, y=0; // initial posistion in spatial frame
  
  float rD = 0; //right wheel distance 
  float lD = 0; //left wheel distance 
  float d_rD; // difference in right wheel distance
  float d_lD; // difference in left wheel distance 
  float vR; // right wheel velocity 
  float vL; // left wheel velocity 
  float delta_theta; //change in theta 

  int obstacle = 1; //flag to control loop when obstacle is first encountered 
  int found = 1; //flag to control loop when obstacle is first encountered 
  
  int k;  //define constant variable to check for leave condition 
  int BlackLine = 0; //flag to activate/deactivate following line 
  float Rem_Distance; // remaining distance before goal is reached  

 int leftSpeed;   // variable used to store the leftMotor speed
 int rightSpeed;  // variable used to store the rightMotor speed

  
  void setup()
  {
   // start serial port and  print column names
   Serial.begin(9600);
   Serial.println(" Project #3 ");
   Serial.print("x\t");  
   Serial.println("y\t");
   Serial.println("============================");
   accel.enableBump();            // accelarameter bump detection enabled
   accel.setBumpThresh(70);       // sensitivity of the bump setpoint
   accel.checkBump();             // clear checkBump() flag 
   encoder.clearEnc(BOTH);  // Reset the encoders
  }
  
  void loop()
  {
   if(accel.checkBump())       // checks for bump input. If checkBump is true, start moving
    {
      while (true) //MAIN LOOP 
      {
      delay(10); //delay for 10 ms 
      
      //distance travelled by each wheel in 10ms 
      d_rD = getDistanceR() - rD; //current distance - previous(old) 
      d_lD = getDistanceL() - lD; 
  
      //velocity of each wheel 
      vR = d_rD/(0.01); //distance traveled divided by time 
      vL = d_lD/(0.01); 
  
      //previous distance 
      rD = getDistanceR(); 
      lD = getDistanceL(); 
  
      delta_theta  = ((vR - vL)/(Wheel_Length))*(.01); //change in theta 
      
      // update current position of robot 
      theta = theta + delta_theta; 
      x = x + (abs(vR + vL)/2)*cos(theta)*.01;
      y = y + (abs(vR + vL)/2)*sin(theta)*.01;

      //Print current location
      Serial.print(x); 
      Serial.print("\t");
      Serial.println(y); 
 
      Go2Point(x_g,y_g,80);  // call function to move towards goal point
      }
    }
  }



//---------------Function to go from "point A to point B"--------------------------------

void Go2Point(float x_g, float y_g,int power)
{
  Rem_Distance = distance(x,y,x_g,y_g); //find distance between robot and goal point 
  
  //define and calculate the remaining angle between robot and target 
  float targetTheta = Angle2Target(x,y,x_g,y_g);

  
  if((left.read() > LINETHRESHOLD) || (center.read() > LINETHRESHOLD ) || (right.read() > LINETHRESHOLD )) //obstacle encountered 
  {
    BlackLine= 1; //raise flag for the robot to pivot and follow obstacle (black line)
  }
        
  if (BlackLine != 1)//OUTER LOOP: STRAIGHT TO PATH SECTION. if not in black line, go straight to goal 
  {
      //if we are within 0.5in of goal, stop 
      if(Rem_Distance< 0.5)
      {
        motors.stop();
        delay(100); 
        return;
      }
      // if within .05 rad, then drive straight forward to goal 
      if(abs(theta - targetTheta) < .05)
      {
        motors.drive(power);
        return; 
      }
  
       //If we are off by 0.25 radians, adjust accordingly to drive straight 
      if(abs(theta - targetTheta) > .25)
      { 
      Rotate2Target(targetTheta, power);
      return;
      }
   }
  else //OUTER LOOP: FOLLOW LINE SECTION 
  {
   if (obstacle == found) // control pivot when line first encounters line 
    {      
      if((left.read()>700)||(center.read()>LINETHRESHOLD)|| (right.read()>700)) // pivot to the left 
       {
        delay(180); //delay for the robot to go a little past black line and make if-loop false 
        
        //Pivot to the left for 150ms 
        motors.leftMotor(90); 
        motors.rightMotor(105);
        delay(150);
        
        //positive values for right and left speed to continue pivoting left when FollowLine Function is called. 
        leftSpeed =100; 
        rightSpeed =115;
        
        FollowLine(88); 
       }
       else 
       {
        //Done with pivoting
        //adjust position of robot when it hits line to compenstate for erros in following line. This is only done once. 
        x = x+.5;
        y = y+3;
        found++; //move counter up for next obstacle and to prevent pivoting from executing while on path 
       }
       return; 
     }
     else //Check for a clear path towards goal 
     {
        if ((leftSpeed<0)&&(rightSpeed<0)&& (abs((theta+k*2*PI)-targetTheta)<.3))//rotate towards goal if rotating right and if current and target angles align within 0.3 radians 
        {
          BlackLine = 0; //reset flag to leave black line 
          obstacle++;  //get ready for next obstacle
          k++;  //constant to keep theta withing 2PI
        }
        else 
        {
            FollowLine(85); //dont compare if we are back at hit point 
        }
          return; 
     }
   } 
 }

  

///-------- Follow Line Function--------------------------------------------

void FollowLine(int SPEED) 
{
      if ((left.read() < LINETHRESHOLD) && (center.read() > LINETHRESHOLD ) && (right.read() < LINETHRESHOLD ) ) //010:drive straight 
        {
          leftSpeed = -SPEED; 
          rightSpeed = SPEED;
        }

      else if((left.read() > LINETHRESHOLD) && (center.read() < LINETHRESHOLD ) && (right.read() < LINETHRESHOLD ))  // 100: turn left 
        {
          leftSpeed = SPEED ;
          rightSpeed = SPEED +15;
        }
      
       else if((left.read() < LINETHRESHOLD)&& (center.read() < LINETHRESHOLD ) && (right.read() > LINETHRESHOLD )) // 001: turn right 
        {
          leftSpeed = -(SPEED +15);
          rightSpeed = -SPEED;
        }
  
       else if((left.read() > LINETHRESHOLD ) && (center.read() > LINETHRESHOLD) && (right.read() < LINETHRESHOLD ))  // 110 turn left 
        {  
         leftSpeed = SPEED ;
         rightSpeed = SPEED +15 ;
         delay(186); 
        }
        
       else if((left.read() < LINETHRESHOLD) && (center.read() > LINETHRESHOLD)&& (right.read() > LINETHRESHOLD ))// 011 turn right 
        { 
          leftSpeed = -(SPEED+15);
          rightSpeed = -SPEED;
          delay(190); 
        }
     
//another if-else-loop 
    //if robot turning left and if either of follwoing conditions are met: 000 (blank), 111 (all black),(left and right censor in black line), then keep turning left 
    if ( ((leftSpeed > 0)&&(rightSpeed > 0)) && (((left.read() < 750) && (center.read() < 750) && (right.read() <750)) || ((left.read() > LINETHRESHOLD) && (center.read() < LINETHRESHOLD) && (right.read() >LINETHRESHOLD)) || ((left.read() > LINETHRESHOLD) && (center.read() > LINETHRESHOLD) && (right.read() > LINETHRESHOLD))) )
      {
        leftSpeed =  SPEED ;
        rightSpeed = SPEED+15 ;
      }
      
   //if robot turning right and if either of following conditions are met: 000 (blank), 111 (all black),(left and right censor in black line), then keep turning right
    else if ( ((leftSpeed < 0)&&(rightSpeed < 0)) && (((left.read() < 750) && (center.read() < 750) && (right.read() <750)) || ((left.read() > LINETHRESHOLD) && (center.read() < LINETHRESHOLD) && (right.read() >LINETHRESHOLD))|| ((left.read() > LINETHRESHOLD) && (center.read() > LINETHRESHOLD) && (right.read() > LINETHRESHOLD))) )
    {
       leftSpeed = -(SPEED+15) ;
       rightSpeed = -(SPEED) ;
    }
    //send current power to drive motors  
    motors.leftMotor(leftSpeed);
    motors.rightMotor(rightSpeed);
    return;
} 

//------------------ROTATE TO TARGET ANGLE FUNCTION---------------------------------
  
void Rotate2Target(float theta_g,int power) // theta_g  = desired theta 
{
 
//  If it's already aligned, don't do anything
  if(abs(theta - theta_g) < .05)
  {
   return;
  }

  //Rotate left if current theta < target 
  if(theta_g > theta)
  {
    //Rotate left
   motors.rightMotor(power);
   motors.leftMotor(power);
  }
  else
  {
    //Rotate right
     motors.rightMotor(-power);
     motors.leftMotor(-power);
  }
  return;  
}

////--------Distance Formula Function-----------------

float distance(float x1, float y1, float x2, float y2)
{
  return sqrt(square(x1-x2)+square(y1-y2));
}

//--------------Function to calculate angle using arctan-------------

float Angle2Target(float x_int, float y_int, float x_tar, float y_tar)
{
  float theta_d = atan((y_tar-y_int)/(x_tar-x_int));
  if( x_tar > x_int)
  {
    return theta_d;
  }
  else{
    return theta_d+PI;
  }
}

//----------------Functions to calculate the distance traveled by each wheel in inches from the last time the encoders were cleared -----------------------

float getDistanceR()
{
  return (float)encoder.getTicks(RIGHT)*wheelCirc/countsPerRot;
}

float getDistanceL()
{
  return (float)encoder.getTicks(LEFT)*wheelCirc/countsPerRot;
}
    

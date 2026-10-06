// Base code variable declarations

float deg=45; // Rotation degree
float s = 0;  //Encoder counts
int sm1 = 0;  //Built-in chanel 1
int sm2 = 0;  //Built-in chanel 2
int r = 0;    //indicator for reading builtin encoder to avoid the reading redundancy
float er;     //Proportional error for PI controller
float eri;    //Integral error for PI controller

int t=0;    //time in ms
int t0=0;   //memory for time in ms

int finish=0;  //finish indicator
int rep=1;     //Repetition indicator

// End of base code variable declarations
/* -------------------------------------------------------------------- */
// Custom variable declarations

#define THRESHOLD 150

byte grayCode[] = {
  0b00000, 0b00001, 0b00011, 0b00010, 0b00110, 0b00111, 0b00101, 0b00100,
  0b01100, 0b01101, 0b01111, 0b01110, 0b01010, 0b01011, 0b01001, 0b01000,
  0b11000, 0b11001, 0b11011, 0b11010, 0b11110, 0b11111, 0b11101, 0b11100,
  0b10100, 0b10101, 0b10111, 0b10110, 0b10010, 0b10011, 0b10001, 0b10000
};
int Sensor1 = A0; // Innermost Sensor
int Sensor2 = A1;
int Sensor3 = A2;
int Sensor4 = A3;
int Sensor5 = A4; // Outermost Sensor
int sensor1val, sensor2val, sensor3val, sensor4val, sensor5val;
byte startPos = 0;
byte prevPos = 0;
byte currPos = 0;
byte endPos = 0;
int prevIndex, currIndex;
double absTheta, relTheta;
String dirn;

// End of custom variables

void setup() {
  // Base code setup
  
  Serial.begin(250000);                                                 //Baud rate of communication 
  Serial.println("Enter the desired rotation in degree.");  
  while (Serial.available() == 0) {                                     //Obtaining data from user 
    //Wait for user input
  }
  
  deg = Serial.readString().toFloat();  //Reading the Input string from Serial port.
  if (deg<0) {
    analogWrite(3,255);                 //change the direction of rotation by applying voltage to pin 3 of arduino
  }
  deg=abs(deg);
  
  // End of base code setup
  /* -------------------------------------------------------------------- */
  // Custom setup

  sensor1val = analogRead(Sensor1);
  sensor2val = analogRead(Sensor2);
  sensor3val = analogRead(Sensor3);
  sensor4val = analogRead(Sensor4);
  sensor5val = analogRead(Sensor5);
  startPos |= ((sensor1val > THRESHOLD) ? 1 : 0) << 5;
  startPos |= ((sensor2val > THRESHOLD) ? 1 : 0) << 4;
  startPos |= ((sensor3val > THRESHOLD) ? 1 : 0) << 3;
  startPos |= ((sensor4val > THRESHOLD) ? 1 : 0) << 2;
  startPos |= ((sensor5val > THRESHOLD) ? 1 : 0) << 1;
  currPos = startPos;
  for (int i = 0; i < 32; i++){
    if (grayCode[i] == currPos){
      currIndex = i;
      break;
    }
  }
  absTheta = currIndex * 11.25;
  Serial.println(startPos, BIN); // FOR TESTING PURPOSES, printing the startpos

  // End of custom setup
}

float kp = .6*90/deg;                         //proportional gain of PI
float ki = .02;                               //integral gain of PI 


void loop() {  
  t=millis();                 //reading time
  t0=t;                       //saving the current time in memory

  // Custom code
  prevPos = currPos;
  prevIndex = currIndex;
  // End of custom code
  
  while (t<t0+4000 && rep<=10) {              // let the code run for 4 seconds each with 10 repetitions
    if (t%10 == 0) {                //PI controller that runs every 10ms 
      if (s < deg * 114 * 2 / 360) {
        er = deg - s *360/228;
        eri = eri + er;
        analogWrite(6, kp * er + ki * eri);
      }
      if (s >= deg * 228/360) {
        analogWrite(6, 0);
        eri = 0;
      }
      delay(1);
    }

    sm1 = digitalRead(7);         //reading chanel 1 
    sm2 = digitalRead(8);         //reading chanel 2

    if (sm1 != sm2 && r == 0) {                     //counting the number changes for both chanels
      s = s + 1;
      r = 1;                    // this indicator wont let this condition, (sm1 != sm2), to be counted until the next condition, (sm1 == sm2), happens
    } if (sm1 == sm2 && r == 1) {
      s = s + 1;
      r = 0;                    // this indicator wont let this condition, (sm1 == sm2), to be counted until the next condition, (sm1 != sm2), happens
    }

    t=millis();           //updating time
    finish=1;             //changing finish indicator
  }
  
  // Custom sensing

  sensor1val = analogRead(Sensor1);
  sensor2val = analogRead(Sensor2);
  sensor3val = analogRead(Sensor3);
  sensor4val = analogRead(Sensor4);
  sensor5val = analogRead(Sensor5);
  currPos |= ((sensor1val > THRESHOLD) ? 1 : 0) << 5;
  currPos |= ((sensor2val > THRESHOLD) ? 1 : 0) << 4;
  currPos |= ((sensor3val > THRESHOLD) ? 1 : 0) << 3;
  currPos |= ((sensor4val > THRESHOLD) ? 1 : 0) << 2;
  currPos |= ((sensor5val > THRESHOLD) ? 1 : 0) << 1;
  for (int i = 0; i < 32; i++){
    if (grayCode[i] == currPos){
      currIndex = i;
      break;
    }
  }

  absTheta = currIndex * 11.25;
  relTheta = (currIndex - prevIndex) * 11.25;
  dirn = (relTheta > 0) ? "CW" : "CCW"; // NEED TO CHECK THESE DIRECTIONS

  // End of custom sensing

  if (finish==1){                                //this part of the code is for displaying the result
    delay(500);                              //half second delay
    rep=rep+1;                               // increasing the repetition indicator
    Serial.print("shaft position from optical absolute sensor from home position: ");
    Serial.println(absTheta);
      
    Serial.print("shaft displacement from optical absolute sensor: ");
    Serial.print(relTheta);
    Serial.println(" , direction: " + dirn);
      
    Serial.print("Shaft displacement from motor's builtin encoder: ");
    Serial.println(s * 360 / 228);                                      //every full Revolution of the shaft is associated with 228 counts of builtin 
                                                                          //encoder so to turn it to degre we can use this formula (s * 360 / 228), "s" is the number of  built-in encoder counts
    float Error=relTheta-s*360/228;
    Serial.print("Error :");
    Serial.println(Error);                                              //displaying error
    Serial.println();
    s = 0;
    finish=0; 
  }
  analogWrite(6,0);                                                         //turning off the motor
}

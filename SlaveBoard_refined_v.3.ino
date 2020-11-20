//Notes from Constance:
//Added all the maze code, commented out a POTENTIALLY DELETE bit (not sure what it does), and added 
//"char recvChar;" back into the code. Commented out bluetooth(sendThis) because of errors
//I haven't changed the turning code, because the code we currently have for maze turning is specifically 
//a 10cm wide turn and you might want it to be different.

//-----------------------------------------------------------------------------------------------------------//
//                                                                                                           //
//  Slave_ELEC1601_Student_2019_v3                                                                           //
//  The Instructor version of this code is identical to this version EXCEPT it also sets PIN codes           //
//  20191008 Peter Jones                                                                                     //
//                                                                                                           //
//  Bi-directional passing of serial inputs via Bluetooth                                                    //
//  Note: the void loop() contents differ from "capitalise and return" code                                  //
//                                                                                                           //
//  This version was initially based on the 2011 Steve Chang code but has been substantially revised         //
//  and heavily documented throughout.                                                                       //
//                                                                                                           //
//  20190927 Ross Hutton                                                                                     //
//  Identified that opening the Arduino IDE Serial Monitor asserts a DTR signal which resets the Arduino,    //
//  causing it to re-execute the full connection setup routine. If this reset happens on the Slave system,   //
//  re-running the setup routine appears to drop the connection. The Master is unaware of this loss and      //
//  makes no attempt to re-connect. Code has been added to check if the Bluetooth connection remains         //
//  established and, if so, the setup process is bypassed.                                                   //
//                                                                                                           //
//-----------------------------------------------------------------------------------------------------------//

#include <SoftwareSerial.h>   //Software Serial Port
#include <Servo.h>

#define RxD 7
#define TxD 6
#define ConnStatus A1

#define DEBUG_ENABLED  1


Servo servoLeft;    
Servo servoRight;  

//variables used in maze program
int Direction[4] = {5,1,-5,-1}; //up, right, down, left
int i = 0; //This is the index for the direction array
int GridIndex = 1; // Current location - see grid:

// 15|  16 17  18 19 |
// 10|  11 12  13 14 |15
// 5 |  6  7   8  9  |10
// 0 |  1  2   3  4  |5

int CellType[20] = {99,0,0,0,0,99,0,0,0,0,99,0,0,0,0,99,0,0,0,0};
//99 cells are illegal cells/end of maze

// ##################################################################################
// ### EDIT THE LINES BELOW TO MATCH YOUR SHIELD NUMBER AND CONNECTION PIN OPTION ###
// ##################################################################################

int shieldPairNumber = 11;

// CAUTION: If ConnStatusSupported = true you MUST NOT use pin A1 otherwise "random" reboots will occur
// CAUTION: If ConnStatusSupported = true you MUST set the PIO[1] switch to A1 (not NC)

boolean ConnStatusSupported = true;   // Set to "true" when digital connection status is available on Arduino pin

// #######################################################

// The following two string variable are used to simplify adaptation of code to different shield pairs

String slaveNameCmd = "\r\n+STNA=Slave";   // This is concatenated with shieldPairNumber later

SoftwareSerial blueToothSerial(RxD,TxD);


void setup()
{
    Serial.begin(9600);
    blueToothSerial.begin(38400);                    // Set Bluetooth module to default baud rate 38400

   //buzzerPOTENTIALLY DELETE Deleted buzzer cause it'a not used and pin 9 is already assigned to smthg

    pinMode(4,OUTPUT); pinMode(5,INPUT); //Right
    pinMode(8,OUTPUT); pinMode(11,INPUT); //Left
    pinMode(0,OUTPUT); pinMode(1,INPUT);  //Front IR sensor
    
   // pinMode(A0, OUTPUT); //light sensorPOTENTIALLY DELETE
    pinMode(RxD, INPUT);
    pinMode(TxD, OUTPUT);
    pinMode(ConnStatus, INPUT);


servoLeft.attach(13);
servoRight.attach(12);



    //  Check whether Master and Slave are already connected by polling the ConnStatus pin (A1 on SeeedStudio v1 shield)
    //  This prevents running the full connection setup routine if not necessary.

    if(ConnStatusSupported) Serial.println("Checking Slave-Master connection status.");

    if(ConnStatusSupported && digitalRead(ConnStatus)==1)
    {
        Serial.println("Already connected to Master - remove USB cable if reboot of Master Bluetooth required.");
    }
    else
    {
        Serial.println("Not connected to Master.");
        
        setupBlueToothConnection();   // Set up the local (slave) Bluetooth module

        delay(1000);                  // Wait one second and flush the serial buffers
        Serial.flush();
        blueToothSerial.flush();
    }
}


void loop()
{
char recvChar;
        char rC;//added back in from the original code
    servoLeft.writeMicroseconds(1500);
    servoRight.writeMicroseconds(1520);
   
    while(1)
    {
        if(blueToothSerial.available())   // Check if there's any data sent from the remote Bluetooth shield
        {
 
            rC = blueToothSerial.read();
            String Signal = ("");
 
      if(rC == 'w'){ //move forward
            servoLeft.writeMicroseconds(1300);  // Left wheel counterclockwise
            servoRight.writeMicroseconds(1720);
    delay(400);
    servoLeft.writeMicroseconds(1500);
    servoRight.writeMicroseconds(1520);
      }
      else if(rC == 's'){ //reverse
        servoLeft.writeMicroseconds(1700);  // Left wheel counterclockwise
        servoRight.writeMicroseconds(1300);
    delay(400);
    servoLeft.writeMicroseconds(1500);
    servoRight.writeMicroseconds(1520);
        }
      else if(rC == 'd'){ //turn right
        servoLeft.writeMicroseconds(1500);  // Left wheel counterclockwise
        servoRight.writeMicroseconds(1700);
    delay(400); //may need to change for desired angle
    servoLeft.writeMicroseconds(1500);
    servoRight.writeMicroseconds(1520);
      }
      else if(rC == 'a'){
        servoLeft.writeMicroseconds(1500);  // Left wheel counterclockwise
        servoRight.writeMicroseconds(1300);
    delay(400); //may need to change for desired angle
    servoLeft.writeMicroseconds(1500);
    servoRight.writeMicroseconds(1520);
        }
      else if(rC =='p'){
         break;
      }
        }

    }

  //Create another infinite while loop with code for automatic maze feature.
while (GridIndex <= 19 && GridIndex != 5 && GridIndex != 10 && GridIndex != 15 && GridIndex != 0) {
//
servoLeft.detach();
servoRight.detach();
delay(200);

  int irLeft = irDetect(8, 11, 38000);       // Check for object
  int irRight = irDetect(4, 5, 38000);
  int irFront = irDetect(0, 1, 38000);

  Serial.print(irLeft); 
  Serial.print(irFront);                // Display 1/0 no detect/detect
  Serial.println(irRight);



if(irFront == 1) //if there's a path forward, move forward
{  MoveForward(); }

else if(irLeft == 1) //Otherwise go left (even if there's 2 paths)
{  TurnLeft(); }

else if(irRight == 1) //Go right if only right is free
{  TurnRight(); }

else 
{
  break; // If everything = 0, it's a dead end.
}
}

servoLeft.detach();
servoRight.detach();

blueToothSerial.print('f');

delay(1000);
PrintMap();
delay(5000);

 while(1)//This is to pause the program until a bluetooth signal is sent
    {
      
        if(blueToothSerial.available())   // Check if there's any data sent from the remote Bluetooth shield
        {
            recvChar = blueToothSerial.read();

      if(recvChar == 'g'){
        break;} // move on to memory maze if signal is sent
      }
      delay(1000);
      }
      
GridIndex = 1; //same as the start position

while (1) {

if(CellType[GridIndex] >=10 && CellType[GridIndex] <= 19) 
{  MoveForward(); }

else if(CellType[GridIndex] >=20 && CellType[GridIndex] <= 29) 
{  TurnLeft(); }

else if(CellType[GridIndex] >=30 && CellType[GridIndex] <= 39) 
{  TurnRight(); }
else
{
  break;
}
}


delay(300); // now the program will loop back to manual control

}//end void loop


void MoveForward()
{

CellType[GridIndex] = 10+i; //(if we want to use the direction 
                         //for something, say printing a map)
  GridIndex = GridIndex + Direction[i];

Serial.println("Moving Forward");

  servoLeft.attach(13);
  servoRight.attach(12);
  servoLeft.writeMicroseconds(1300);
  servoRight.writeMicroseconds(1700);
  delay(2500); //How long it takes to go 28 cm.
}

void TurnLeft()
{

CellType[GridIndex] = 20+i; 

i = increment(i,-1); //different direction
  GridIndex = GridIndex + Direction[i];

Serial.println("Turning Left");

  servoLeft.attach(13);      //turn Left
  servoLeft.writeMicroseconds(1900);
  delay(1500);
  servoRight.attach(12);
  servoLeft.writeMicroseconds(1300); //Move forward
  servoRight.writeMicroseconds(1700);
  delay(1000);
}

void TurnRight()
{

CellType[GridIndex] = 30+i;

i = increment(i,1); //different direction
  GridIndex = GridIndex + Direction[i];


Serial.println("Turning Right");


  servoRight.attach(12);      //turn right
  servoRight.writeMicroseconds(1900);
  delay(1500);
  servoLeft.attach(13);
  servoLeft.writeMicroseconds(1300); //Move forward
  servoRight.writeMicroseconds(1700);
  delay(1000);
}





//This is to make sure i stays between 0 and 3
int increment(int i, int value)
{
  int result = i + value;
  if (result <= -1)
  {
    return 3;
  }
  else if (result >= 4)
  {
    return 0;
  }
  else
  {
    return result;
  }
}


//This function should be on the Masterboard to display
void PrintMap()
{
  //Display
Serial.println();

Serial.print(PrintCell(16));
Serial.print(PrintCell(17));
Serial.print(PrintCell(18));
Serial.println(PrintCell(19));

Serial.print(PrintCell(11));
Serial.print(PrintCell(12));
Serial.print(PrintCell(13));
Serial.println(PrintCell(14));

Serial.print(PrintCell(6));
Serial.print(PrintCell(7));
Serial.print(PrintCell(8));
Serial.println(PrintCell(9));

Serial.print(PrintCell(1));
Serial.print(PrintCell(2));
Serial.print(PrintCell(3));
Serial.println(PrintCell(4));

Serial.println();
}

//This picks shapes to make the map with 
//based on the Cell Type that was stored.

//In a finished project, these would return some 
//letters and send those letters to the master board

const char* PrintCell(int Index)
{
  int Type = CellType[Index];
  if (Type == 0)
  {
    return  "\u25FC";//◼   other options:▤▥▦▧▨▩ (\u25A4 -\u25A9)
  }
  if (Type == 10 || Type == 12)
  {
    return "\u25AF";//▯
  }
  if (Type == 11 || Type == 13)
  {
    return "\u25AD";//▭
  }
  if (Type == 20 || Type == 32)
  {
    return "\u25C1";//◁
  }
  if (Type == 21 || Type == 33)
  {
    return "\u25B3";//△
  }
  if (Type == 30 || Type == 22)
  {
    return "\u25B7";//▷
  }
  if (Type == 31 || Type == 23)
  {
    return "\u25BD";//▽
  }

}

int irDetect(int irLedPin, int irReceiverPin, long frequency)
{
  tone(irLedPin, frequency, 8);              // IRLED 38 kHz for at least 1 ms
  delay(1);                                  // Wait 1 ms
  int ir = digitalRead(irReceiverPin);       // IR receiver -> ir variable
  delay(1);                                  // Down time before recheck
  return ir;                                 // Return 1 no detect, 0 detect
}  
  

void setupBlueToothConnection()
{
    Serial.println("Setting up the local (slave) Bluetooth module.");

    slaveNameCmd += shieldPairNumber;
    slaveNameCmd += "\r\n";

    blueToothSerial.print("\r\n+STWMOD=0\r\n");      // Set the Bluetooth to work in slave mode
    blueToothSerial.print(slaveNameCmd);             // Set the Bluetooth name using slaveNameCmd
    blueToothSerial.print("\r\n+STAUTO=0\r\n");      // Auto-connection should be forbidden here
    blueToothSerial.print("\r\n+STOAUT=1\r\n");      // Permit paired device to connect me
    
    //  print() sets up a transmit/outgoing buffer for the string which is then transmitted via interrupts one character at a time.
    //  This allows the program to keep running, with the transmitting happening in the background.
    //  Serial.flush() does not empty this buffer, instead it pauses the program until all Serial.print()ing is done.
    //  This is useful if there is critical timing mixed in with Serial.print()s.
    //  To clear an "incoming" serial buffer, use while(Serial.available()){Serial.read();}

    blueToothSerial.flush();
    delay(2000);                                     // This delay is required

    blueToothSerial.print("\r\n+INQ=1\r\n");         // Make the slave Bluetooth inquirable
    
    blueToothSerial.flush();
    delay(2000);                                     // This delay is required
    
    Serial.println("The slave bluetooth is inquirable!");
}

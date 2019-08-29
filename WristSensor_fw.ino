// WristSensor with ADXL345 Accelerometer
/////////////////////////////////////////////////////////////////////

// INCLUDE NECESSARY LIBRARIES
//#include "Wire.h" // THIS LIBRARY WORKS WITH ARDUINO BOARDS
#include <i2c_t3.h> // THIS LIBRARY ONLY WORKS WITH TEENSY BOARDS

// DEFINE GLOBAL BOOLEAN VARIABLES
#define TRUE 1
#define FALSE 0

// DEFINE DATA PACKET LENGTH TO SEND OVER WIFI
#define BYTE_PUREDATA_PACKET 29
#define BYTE_WIFI_INCOMING_PACKET 16

/////////////////////////////////////////////////////////////////////

// LIST OF ADXL345 ACCELEROMETER REGISTERS (I2C COMMUNICATION)
#define ADXL345_ADDRESS 0x53

#define ADXL345_POWER_CTL 0x2D
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_RA_BW_RATE 0x2C
#define ADXL345_RESET_CTL 0x00
#define ADXL345_MEAS_CTL 0x08
#define ADXL345_DATAX0 0x32 
#define ADXL345_DATAX1 0x33
#define ADXL345_DATAY0 0x34
#define ADXL345_DATAY1 0x35
#define ADXL345_DATAZ0 0x36
#define ADXL345_DATAZ1 0x37
#define ADXL345_RANGE_2G            0b00
#define ADXL345_RANGE_4G            0b01
#define ADXL345_RANGE_8G            0b10
#define ADXL345_RANGE_16G           0b11
#define ADXL345_RATE_3200           0b1111
#define ADXL345_RATE_1600           0b1110
#define ADXL345_RATE_800            0b1101
#define ADXL345_RATE_400            0b1100
#define ADXL345_RATE_200            0b1011
#define ADXL345_RATE_100            0b1010

#define ADXL345_LSB_G_2G            256.0f
#define ADXL345_LSB_G_4G            128.0f
#define ADXL345_LSB_G_8G             64.0f

// SET SERIAL 1 AS WIFI (CONNECT TO XBEE EXPLORER)
#define WIFI Serial3

// ONBOARD LED FOR DEBUGGING
#define LED_TEENSY 13

// DEFINE REGISTERS TO RESTART TEENSY REMOTELY
/////////////////
#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);
/////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// DATA STRUCTURE OF ADXL345 SENSOR
struct structADXL345Accelerometer
{
  //Little Endian
  float ax;
  float ay;
  float az;
} ADXL345acc;

/* NOT USED
struct structADXL345AccelerometerRAW
{
  //Little Endian
  int16_t ax;
  int16_t ay;
  int16_t az;
} ADXL345accRAW;
*/

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// FUNCTIONS TO READ DATA FROM ADXL345 USING I2C COMMUNICATION
void ADXL345writeCommand(uint8_t address, uint8_t value) 
{
  Wire.beginTransmission(ADXL345_ADDRESS);    // Start transmission to device 
  Wire.write(address);                        // Send register address
  Wire.write(value);                          // Send value to write
  Wire.endTransmission();                     // End transmission
}

// Reads num bytes starting from address register on device in to _buff array
void ADXL345readData(uint8_t address, uint8_t nByte, byte buffer[]) 
{
  Wire.beginTransmission(ADXL345_ADDRESS); // start transmission to device  
  Wire.write(address);             // sends address to read from
  Wire.endTransmission();         // end transmission

  Wire.beginTransmission(ADXL345_ADDRESS); // start transmission to device
  Wire.requestFrom(ADXL345_ADDRESS, nByte);    // request 6 bytes from device

  uint8_t i = 0;
  while(Wire.available())         // device may send less than requested (abnormal)
  { 
    buffer[i] = Wire.read();    // receive a byte
    i++;
  }
  Wire.endTransmission();         // end transmission
}

void ADXL345readAccel() 
{
  uint8_t buffer[6];
  
  ADXL345readData(ADXL345_DATAX0, 0x06, buffer); // read the acceleration data from the ADXL345

  //Least Significat Byte first
  ADXL345acc.ax = (float)((int16_t)((buffer[1] << 8) | buffer[0]))/ADXL345_LSB_G_4G;   
  ADXL345acc.ay = (float)((int16_t)((buffer[3] << 8) | buffer[2]))/ADXL345_LSB_G_4G;
  ADXL345acc.az = (float)((int16_t)((buffer[5] << 8) | buffer[4]))/ADXL345_LSB_G_4G;
  
  // RAW DATA NOT USED 
  //ADXL345accRAW.ax = ((int16_t)((buffer[1] << 8) | buffer[0]));   
  //ADXL345accRAW.ay = ((int16_t)((buffer[3] << 8) | buffer[2]));
  //ADXL345accRAW.az = ((int16_t)((buffer[5] << 8) | buffer[4]));
}

struct structIncomingWifiPacket
{
  uint8_t Start_bytes[3];
  uint8_t val; // 0 - Reboot ; 1 - Start Recording ; 2 - Broadcast Timestamp
  uint8_t Odroid_Timestamp[8];
  uint8_t Odroid_Trigger;
  uint8_t Stop_bytes[3];
};

union unionIncomingWifi
{
  uint8_t vData[BYTE_WIFI_INCOMING_PACKET];
  structIncomingWifiPacket Raw;
} Wifi_Incoming_Data;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// THIS FUNCTION IS TO CREATE THE DATA PACKET THAT IS SENT THROUGH SERIAL OR WIFI
uint8_t dataCommunicationPacket[BYTE_PUREDATA_PACKET];

void createDataPacket_PureData(uint8_t *dataPacket,
unsigned long timestamp,
float ax, float ay, float az,
uint8_t Odroid_Timestamp[8],
uint8_t trigger,
unsigned long timestamp2)
{
  uint8_t* pointer;
  int16_t val;
  
  // HEADER
  dataPacket[0]=0x07;
  dataPacket[1]=0x08;
  dataPacket[2]=0x09;
    
  // Internal Timestamp
  pointer=(uint8_t*)&timestamp;
  dataPacket[3]=pointer[3];
  dataPacket[4]=pointer[2];
  dataPacket[5]=pointer[1];
  dataPacket[6]=pointer[0];
  
  // X-ACC ADXL345_LSB_G_4G
  val=int16_t(ax*ADXL345_LSB_G_4G);
  pointer=(uint8_t*)&val;
  dataPacket[7]=pointer[1];
  dataPacket[8]=pointer[0];
  
  // Y-ACC ADXL345_LSB_G_4G
  val=int16_t(ay*ADXL345_LSB_G_4G);
  pointer=(uint8_t*)&val;
  dataPacket[9]=pointer[1];
  dataPacket[10]=pointer[0];
  
  // Z-ACC ADXL345_LSB_G_4G
  val=int16_t(az*ADXL345_LSB_G_4G);
  pointer=(uint8_t*)&val;
  dataPacket[11]=pointer[1];
  dataPacket[12]=pointer[0];

  //Timestamp_Odroid
  dataPacket[13] = Odroid_Timestamp[0];
  dataPacket[14] = Odroid_Timestamp[1];
  dataPacket[15] = Odroid_Timestamp[2];
  dataPacket[16] = Odroid_Timestamp[3];
  dataPacket[17] = Odroid_Timestamp[4];
  dataPacket[18] = Odroid_Timestamp[5];
  dataPacket[19] = Odroid_Timestamp[6];
  dataPacket[20] = Odroid_Timestamp[7];

  //Trigger
  pointer=(uint8_t*)&trigger;
  dataPacket[21]=pointer[0];  

  //Timestamp2
  pointer = (uint8_t*)&timestamp2;
  dataPacket[22] = pointer[3];
  dataPacket[23] = pointer[2];
  dataPacket[24] = pointer[1];
  dataPacket[25] = pointer[0];

  // TAIL
  dataPacket[26]=0xA;
  dataPacket[27]=0xB;
  dataPacket[28]=0xC;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef SWAP_ELEMENT
#define SWAP_ELEMENT(a,b) { a^=b; b^=a; a^=b; }
#endif

// SOME VARIABLES FOR TIMING
unsigned long startTime;
unsigned long nowTime=0;
unsigned long nowTime2 = 0;

unsigned long currTime=0;
unsigned long precTimestamp;

unsigned long timeInterval = 2000; // microseconds (~500 Hz)

unsigned long int cycles=0;
boolean led_teensy=0;

boolean trigger;
boolean signal;

unsigned long nextTrigger; //[ms]

uint8_t Odroid_Timestamp[8]; //[ms]
uint8_t Odroid_Trigger; //[ms]
unsigned long currenttime;
uint8_t cmd;

// BOOLEAN FUNCTIONS TO CHECK THE INCOMING WIFI PACKETS 
// Check if the received packet has the correct head and tail bytes
boolean checkWIFIPacket(uint8_t *buffer)
{
  return (buffer[0] == 0x01 && buffer[1] == 0x02 && buffer[2] == 0x03  && buffer[BYTE_WIFI_INCOMING_PACKET - 3] == 0x4 && buffer[BYTE_WIFI_INCOMING_PACKET - 2] == 0x5 && buffer[BYTE_WIFI_INCOMING_PACKET - 1] == 0x6);
}

// Check for start command
boolean checkRemoteStart2(uint8_t *buffer)
{
  return (buffer[0] == 0x01 && buffer[1] == 0x02 && buffer[2] == 0x03 && buffer[BYTE_WIFI_INCOMING_PACKET - 3] == 0x4 && buffer[BYTE_WIFI_INCOMING_PACKET - 2] == 0x5 && buffer[BYTE_WIFI_INCOMING_PACKET - 1] == 0x6) && (buffer[3] == 0x01);
}

// Check for reboot command
boolean checkRemoteReboot2(uint8_t *buffer)
{
  return (buffer[0] == 0x01 && buffer[1] == 0x02 && buffer[2] == 0x03 && buffer[BYTE_WIFI_INCOMING_PACKET - 3] == 0x4 && buffer[BYTE_WIFI_INCOMING_PACKET - 2] == 0x5 && buffer[BYTE_WIFI_INCOMING_PACKET - 1] == 0x6) && (buffer[3] == 0x00);
}

// Check for timestamp sent from ODROID
boolean checkOdroidTimestamp(uint8_t *buffer)
{
  return (buffer[0] == 0x01 && buffer[1] == 0x02 && buffer[2] == 0x03 && buffer[BYTE_WIFI_INCOMING_PACKET - 3] == 0x4 && buffer[BYTE_WIFI_INCOMING_PACKET - 2] == 0x5 && buffer[BYTE_WIFI_INCOMING_PACKET - 1] == 0x6) && (buffer[3] == 0x02);
}

////////////////////////////////////////   MAIN CODE   //////////////////////////////////////////////////

void setup()
{
  // Set up baud rate for Serial and WIFI
  Serial.begin(115200);
  WIFI.begin(921600);  
  WIFI.setTimeout(100); 
  
  delay(100);  
  WIFI.flush();
  
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_400);   // THIS ONLY WORKS WITH i2c_t3 LIBRARY (TEENSY BOARDS)
//  Wire.begin(); // THIS ONE WORKS WITH ARDUINO BOARDS

  // Some initial blinks
  pinMode(LED_TEENSY, OUTPUT);   
  for(int i=0;i<20;i++)
  { 
    led_teensy=!led_teensy;
    digitalWrite(LED_TEENSY,led_teensy);
    delay(75);
  }
  
  // SETTING UP ADXL345
  ADXL345writeCommand(ADXL345_POWER_CTL,ADXL345_RESET_CTL);
  ADXL345writeCommand(ADXL345_DATA_FORMAT,ADXL345_RANGE_4G);
  ADXL345writeCommand(ADXL345_RA_BW_RATE,ADXL345_RATE_800);
  ADXL345writeCommand(ADXL345_POWER_CTL,ADXL345_MEAS_CTL);

  // START WITH EMPTY DATA PACKET
  for(int i=0;i<sizeof(dataCommunicationPacket);i++) dataCommunicationPacket[i]=0x00;

  // WAIT FOR START COMMAND FROM WIFI
  boolean StartRec = FALSE;
  uint8_t bWIFI;
  while (StartRec == FALSE)
  {
    if (WIFI.available() > 0)
    {
      bWIFI = 0;
      while (!(bWIFI >= BYTE_WIFI_INCOMING_PACKET))
      {
        if ((WIFI.available() > 0) && (bWIFI < BYTE_WIFI_INCOMING_PACKET))
        {
          Wifi_Incoming_Data.vData[bWIFI] = WIFI.read();
          bWIFI++;
        }
      }
      //check integrity of the packet, process the packet
      if (checkRemoteStart2(Wifi_Incoming_Data.vData))
      {
        StartRec = TRUE;
      }
    }
    led_teensy = !led_teensy;
    digitalWrite(LED_TEENSY, led_teensy);
    delay(75);
  }

  // START COUNTING TIME
  startTime = micros(); 
} 

void loop()
{     
  // CHECK FOR INCOMING WIFI PACKET FOR ODROID TIMESTAMP OR REBOOT COMMAND
  uint8_t bWIFI;
  if (WIFI.available() > 0)
  {
    bWIFI = 0;
    while (!(bWIFI >= BYTE_WIFI_INCOMING_PACKET))
    {
      if ((WIFI.available() > 0) && (bWIFI < BYTE_WIFI_INCOMING_PACKET))
      {
        Wifi_Incoming_Data.vData[bWIFI] = WIFI.read();
        bWIFI++;
      }
    }
    //Serial.println("Received!");
    //check integrity
    if (checkWIFIPacket(Wifi_Incoming_Data.vData))
    {
      //Serial.println("Received and validated!");
      if (checkOdroidTimestamp(Wifi_Incoming_Data.vData))
      {
        nowTime2 = micros() - startTime;  
        for (int i = 0; i < sizeof(Odroid_Timestamp); i++) Odroid_Timestamp[i] = Wifi_Incoming_Data.Raw.Odroid_Timestamp[i];
        Odroid_Trigger = Wifi_Incoming_Data.Raw.Odroid_Trigger;
        //Serial.print(Odroid_Trigger);
        //Serial.print("This is Odroid Timestamp!");
        //Serial.println(Odroid_Timestamp[7]);
      }
      else if (checkRemoteReboot2(Wifi_Incoming_Data.vData))
      {
        //Serial.println("This is Odroid Reboot!"); delay(1000);
        CPU_RESTART;
      }
      else
      {
        //Serial.println("Bad Packet!");
        }
    }
  }

  
  currTime = micros()-startTime;
  //nowTime = micros()-startTime;
  //time1 = micros()-startTime;

  //deltaTime = nowTime - currTime;

  if(((currTime-nowTime) >= timeInterval) && (currTime >= 4000000 )) // CHECK IF THE SAMPLING RATE IS AT 500 HZ (2000 microseconds between each reading)
  {
    nowTime = currTime;
    ADXL345readAccel();

    //delayMicroseconds(200); // ADD THIS DELAY TO REDUCE THE SAMPLING RATE
  
  // PRINT TO SERIAL PORT TO DEBUG. COMMENT OUT THIS PART IF NEEDED.
//   Serial.print(ADXL345acc.ax);
//   Serial.print(" , ");
//   Serial.print(ADXL345acc.ay);
//   Serial.print(" , ");
//   Serial.print(ADXL345acc.az);
//   Serial.print(" , ");
   //Serial.println(deltaTime);

    // CREATE DATA PACKET
    createDataPacket_PureData(dataCommunicationPacket,
    nowTime, // Internal Timestamp of the microcontroller

    // Data from sensor
    ADXL345acc.ax,
    ADXL345acc.ay,
    ADXL345acc.az,

    // Broadcast Synchronization
    Odroid_Timestamp, Odroid_Trigger, nowTime2);

    // SEND DATA PACKET TO WIFI SERIAL PORT
    WIFI.write(dataCommunicationPacket,BYTE_PUREDATA_PACKET);

    // BLINK THE ONBOARD LED EVERY 250 CYCLES
    if ((cycles % 250) ==0)
    {
      led_teensy=!led_teensy;
      digitalWrite(LED_TEENSY,led_teensy);
    } 
  
    cycles++; // CYCLE JUST FOR COUNTING, NOT USED IN DATA PACKET
   }
}

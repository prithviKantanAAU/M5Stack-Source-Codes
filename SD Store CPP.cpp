#define processing_out false
#define AHRS true         // Set to false for basic data read
#define SerialDebug false  // Set to true to get Serial output for debugging
#define LCD
#include <SD.h>
#include <SPI.h>
#include <M5Stack.h>
#include "utility/MPU9250.h"

// GLOBAL VARIABLES
const int chipSelect = 10;
String bodyLocations[5] = {"Trunk", "Thigh_L", "Thigh_R", "Shank_L", "Shank_R"};
short idx_bodyLoc = 0;
String sensorPlacements[4] = {"Front", "Back", "Left", "Right"};
short idx_sensorPos = 0;
bool isSD_OK = false;
bool isRecording = false;
bool isRecording_Z1 = false;
String newFile_NAME = "";
unsigned long currentMill = 0;              
unsigned long prevMill = 0;
float recTime_Elapsed = 0;
long rec_numLines_Written = 0;
int beepFreqs[3] = {400,800,1600};
int beepDurs[3] = {500,250,125};
int sampleInterval = 10;                         // IMU SAMPLING INTERVAL IN MS
MPU9250 IMU;
File imuLog;
int fileCountOnSD = 0;
bool isScreenChanged = false;

// MAYBE REMOVE
int lastBatteryValue = 0;                       // LAST BATTERY LEVEL
int timeElapsedMS = 0;                          // TOTAL TIME ELAPSED (MS)
int screenTimeOutMS = 0;                        // SCREEN TIMEOUT FOR AUTOMATIC TURNOFF
bool isScreenOn = false;                        // IS SCREEN ON?

void setup() 
{
    M5.begin();
    Wire.begin();
    M5.Lcd.setTextSize(2);
    M5.Lcd.setBrightness(10);
    IMU.initMPU9250();
    IMU.initAK8963(IMU.magCalibration);
    M5.Lcd.setCursor(0,0);

    if (SD.begin(chipSelect))
    {
    M5.Lcd.print("SD Card Not Found");
    isSD_OK = true;
    }
}

void clearLine(int x, int y)
{
  M5.Lcd.setCursor(x,y);
  M5.Lcd.print("                      ");
  M5.Lcd.setCursor(x,y);
}

void printDirectory(File dir) 
{
  fileCountOnSD = 0;
  while (true) 
  {
    File entry =  dir.openNextFile();
    if (! entry) {
      break;
    }
    fileCountOnSD++;
    entry.close();
  }
}

int getBatteryLevel() {
    Wire.beginTransmission(0x75);
    Wire.write(0x78);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(0x75, 1)) 
    {
        switch (Wire.read() & 0xF0) 
        {
        case 0xE0: return 25;
        case 0xC0: return 50;
        case 0x80: return 75;
        case 0x00: return 100;
        default: return 0;
        }
    }
    return -1;
}

void updateBatteryInfo()
{
  if(M5.Power.isChargeFull()) 
    { 
      M5.Lcd.setCursor(0,220);
      M5.Lcd.print("Charge: FUL");
      M5.Power.setCharge(false);
    }

    else if(M5.Power.isCharging())
    { 
      M5.Lcd.setCursor(0,220); 
      M5.Lcd.print("Charge: WIP");
    }
    else
    {
      M5.Lcd.setCursor(0,220); 
      M5.Lcd.print("Charge: NOT");
    }

   M5.Lcd.setCursor(170,220); 
   int currentBattery = getBatteryLevel();
   M5.Lcd.print("BAT: " + String(getBatteryLevel()));
   lastBatteryValue = currentBattery;
}

void fetchIMUData()
{
  if (IMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
    {
    IMU.readAccelData(IMU.accelCount);  // Read the x/y/z adc values
    IMU.getAres();
    IMU.ax = (float)IMU.accelCount[0]*IMU.aRes; // - accelBias[0];
    IMU.ay = (float)IMU.accelCount[1]*IMU.aRes; // - accelBias[1];
    IMU.az = (float)IMU.accelCount[2]*IMU.aRes; // - accelBias[2];

    IMU.readGyroData(IMU.gyroCount);  // Read the x/y/z adc values
    IMU.getGres();
    IMU.gx = (float)IMU.gyroCount[0]*IMU.gRes;
    IMU.gy = (float)IMU.gyroCount[1]*IMU.gRes;
    IMU.gz = (float)IMU.gyroCount[2]*IMU.gRes;

    IMU.readMagData(IMU.magCount);  // Read the x/y/z adc values
    IMU.getMres();
    IMU.mx = (float)IMU.magCount[0] * IMU.mRes * IMU.magCalibration[0] -
             IMU.magbias[0];
    IMU.my = (float)IMU.magCount[1] * IMU.mRes * IMU.magCalibration[1] -
             IMU.magbias[1];
    IMU.mz = (float)IMU.magCount[2] * IMU.mRes * IMU.magCalibration[2] -
             IMU.magbias[2];
    
    IMU.updateTime();
    IMU.count = millis();
    IMU.sumCount = 0;
    IMU.sum = 0;
    }
}

void startRecording()
{
  int beepType = -1;
  rec_numLines_Written = 0;
  newFile_NAME =  "/";
  newFile_NAME.concat(bodyLocations[idx_bodyLoc]);
  newFile_NAME.concat(" - ");
  newFile_NAME.concat(sensorPlacements[idx_sensorPos]);
  newFile_NAME.concat(" - ");
  newFile_NAME.concat(String(fileCountOnSD - 1));
  newFile_NAME.concat(".csv");
  imuLog = SD.open(newFile_NAME, FILE_WRITE);
  if (imuLog)
  {
    if (idx_bodyLoc < 1)
    beepType = 0;
    else if (idx_bodyLoc < 3)
    beepType = 1;
    else
    beepType = 2;
    //M5.Speaker.tone(beepFreqs[beepType], beepDurs[beepType]);
  }
}

void stopRecording()
{
  imuLog.close();
  File root = SD.open("/");
  printDirectory(root);
}

String getLineString()
{
  String imuLine = "";
  imuLine.concat(String(IMU.ax,3));
  imuLine.concat(",");
  imuLine.concat(String(IMU.ay,3));
  imuLine.concat(",");
  imuLine.concat(String(IMU.az,3));
  imuLine.concat(",");
  imuLine.concat(String(IMU.gx,3));
  imuLine.concat(",");
  imuLine.concat(String(IMU.gy,3));
  imuLine.concat(",");
  imuLine.concat(String(IMU.gz,3));
  imuLine.concat(",");
  imuLine.concat(String(IMU.mx,3));
  imuLine.concat(",");
  imuLine.concat(String(IMU.my,3));
  imuLine.concat(",");
  imuLine.concat(String(IMU.mz,3));
  imuLine.concat(",");
  return imuLine;
}

void writeLineToFile()
{
  imuLog = SD.open(newFile_NAME, FILE_APPEND);     
    if (imuLog)                                   // If Opened OK
    {
      imuLog.println(getLineString().c_str());  
      isSD_OK = true;

    } 
    else 
    {
       isSD_OK = false;
    }
}

void updateScreen()
{
  // M5.Lcd.clear(BLACK);

  if (!isSD_OK)                         // IF SD CARD NOT FOUND
  {
    clearLine(0,0);
    if (SD.begin(chipSelect))
    {
    M5.Lcd.print("SD Card Not Found");
    isSD_OK = true;
    }
  }

  else
  {
    clearLine(0,0);
    M5.Lcd.print(bodyLocations[idx_bodyLoc].c_str());
    clearLine(0,20);
    M5.Lcd.print(sensorPlacements[idx_sensorPos].c_str());
    updateBatteryInfo();
    if (isRecording)
    {
      clearLine(0,60);
      M5.Lcd.print("Recording in Progress");
      String linesRecorded = "Lines Recorded: ";
      //clearLine(0,80);
      //M5.Lcd.print(linesRecorded.concat(String(rec_numLines_Written)));
    }
    else
    {
      clearLine(0,60);
      M5.Lcd.print("NOT RECORDING - IDLE");
      String numFiles = "Detected Logs: ";
      numFiles.concat(String(fileCountOnSD - 2));
      clearLine(0,80);
      M5.Lcd.print(numFiles.c_str());
    }
  }
}

void loop() 
{
  if (currentMill < 1)
  {
     File root = SD.open("/");
     printDirectory(root);
     updateScreen();
  }
  currentMill = millis();

  if (currentMill - prevMill > sampleInterval)
  {
  
  // UPDATE SCREEN INFO EVERY 30 ms
  // timeElapsedMS = (timeElapsedMS + 10) % 100;
  // if (timeElapsedMS == 0) { updateScreen(); }

   if (isSD_OK)
   {
   
    isScreenChanged = false;

    // HANDLE BUTTON A PRESS 
    if(M5.BtnA.wasPressed()) { idx_bodyLoc = (idx_bodyLoc + 1) % 5; isScreenChanged = true; }

    // HANDLE BUTTON C PRESS
    if(M5.BtnC.wasPressed()) { idx_sensorPos = (idx_sensorPos + 1) % 4; isScreenChanged = true; }

    // HANDLE BUTTON B PRESS 
    if(M5.BtnB.wasPressed()) { isRecording = isRecording ? false : true; isScreenChanged = true; }

    // HANDLE RECORDING STATE CHANGE
    if (isRecording != isRecording_Z1)
    {
      if (isRecording)
      startRecording();
      else
      stopRecording();
    }

    if (isScreenChanged) updateScreen();
  
    // HANDLE IMU AND OSC DATA
    if (isRecording)
    {
      fetchIMUData();
      getLineString();
      writeLineToFile();
    }
  }

  // UPDATE M5 AND SHUFFLE
  
    M5.update();
    prevMill = currentMill;
    isRecording_Z1 = isRecording;
  }
}
/*
 * This code was used for SSI's Project Europa mission.
 * The code monitors data from various sensors and records it to an SD card.
 * 
 * Edward Vendrow, Paxton Scott, Scott Maran
 * Project Europa
 */

#include <SPI.h>
#include "SdFat.h"              
#include "quaternionFilters.h"  //for accelerometer
#include "MPU9250.h"            //acceleromter
#include <Wire.h>               //altimiter
#include "SparkFunMPL3115A2.h"  //altimiter
#include <Adafruit_BMP280.h>    //temp/pressure (BMP280)
#include <Servo.h> 

#include <SoftwareSerial.h>
#include <TinyGPS.h>

#include <IridiumSBD.h>

#define SerialDebug true  // Set to true to get Serial output for debugging

#define SAMPLERATE_DELAY_MS (100)
#define SERIALDEBUG_RATE (20)

int sampleTimer = 0;

int NICHROME_PIN = 5;
int 
int NICHROME_DELAY_MS = 10000;
int nichromeCounter = 0;
bool deploying = false;
bool hasDeployed = false;

float initialAltitude = -1;
float lastAlt = 0;
float vertSpeed = 0;

Servo left_elevator;
Servo right_elevator;
int servo_angle_right = 90;
int servo_angle_left = 90;
int servo_max_change = 15;
int servo_max_angle_right = servo_angle_right + servo_max_change;


//-------------------------------------- Define data structures for sensor data transfer

typedef struct {
  float sats;
  float precision;
  float latitude;
  float longitude;
  float alt;
  float speed__kmph;
  int checksum;
} GPS_t;

typedef struct {
  double temp;
  double pressure;
  double alt;
} Alt_t;

typedef struct {
  double temp;
  double pressure;
  double alt;
} BMP_t;

typedef struct {
  double x;
  double y;
  double z;
} Accel_t;


//-------------------------------------- Define global sensor variables
 
GPS_t gps = {-1, -1};
Alt_t altimiter = {-1, -1, -1};
Accel_t accel = {-1, -1, -1};
BMP_t bmp = {-1, -1, -1};

//-------------------------------------- Create temp/pressure sensor with software SPI
#define BMP_SCK   14
#define BMP_MISO  8
#define BMP_MOSI  7
#define BMP_CS    9

//Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
Adafruit_BMP280 bmpChip(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

//-------------------------------------- Initialize Accelerometer

// Set to false for basic data read
#define AHRS false

// Pin definitions
// These can be changed, 2 and 3 are the Arduinos ext int pins
//int intPin = 12;

#define I2Cclock 400000
#define I2Cport Wire

// Pin definitions
int intPin = 2;  // These can be changed, 2 and 3 are the Arduinos ext int pins

// Use either this line or the next to select which I2C address your device is using
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0   
//#define MPU9250_ADDRESS MPU9250_ADDRESS_AD1

MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);

//-------------------------------------- Initialize GPS
TinyGPS gpsChip;
SoftwareSerial ss(0, 1);
//-------------------------------------- Initialize Baraometer/Altimiter
//Create an instance of the object
MPL3115A2 myPressure;
//--------------------------------------  SD Card setup
// Set USE_SDIO to zero for SPI card access. 
#define USE_SDIO 0

/*
 * Set DISABLE_CHIP_SELECT to disable a second SPI device.
 * For example, with the Ethernet shield, set DISABLE_CHIP_SELECT
 * to 10 to disable the Ethernet controller.
 */
const int8_t DISABLE_CHIP_SELECT = -1;

SdFat sd;
File myFile;

const int chipSelect = 20;//10;

//-------------------------------------- Method Declarations

bool initializeSd();
void setupAltimiter();
void setupIMU();
void writeLineToFile(String filename, String testLine);
void readFileToConsole(String filename);

void logDataToConsole();
void logDataToSD();

void innerLoop();

void updateGPSData();
void updateAccelData();
void updateBMPData();
void updateAltimiterData();
void updateGPSData();
void updateSensorData();

static void smartdelay(unsigned long ms);
static void print_int(unsigned long val, unsigned long invalid, int len);
void print_float(double number, int digits);

//Keep track of time elapsed; when exceeds transmission frequency, make transmission
unsigned long transmissionTime = 0;

void setup()
{
  
  
  //attach servos
  left_elevator.attach(3);
  right_elevator.attach(4);

  left_elevator.write(90);
  right_elevator.write(90);

  //Nichrome pin used for wing deployment
  pinMode(NICHROME_PIN, OUTPUT);
  digitalWrite(NICHROME_PIN, LOW);
  left_elevator.write(90);
  right_elevator.write(90);

  for (pos = 0; pos <=15; pos ++) {
    left_elevator.write(90+pos);
    right_elevator.write(90-pos);
    delay(50)
  }
  delay (1000);
  
  for (pos = 0; pos <=15; pos ++) {
    left_elevator.write(90+pos);
    right_elevator.write(90-pos);
    delay(50)
  }
  
  //Set SPI IO pins
  SPI.setMOSI(11); //DO pin; connected to DI on the sd reader.
  SPI.setMISO(12); //DI pin; connected to DO on the sd reader.
  SPI.setSCK(13); //clock pin

  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  //UNCOMMENT THIS if working on computer; comment out if powering on battery
  //while (!Serial) { delay(1); }

  //If SD fails to initialize, crash :(
  if (!initializeSd()) { return; }

  //Try to initialize BMP
  if (!bmpChip.begin()) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
  }

//  setupAltimiter();
  setupIMU();

  //Begin software serial for GPS
  ss.begin(9600);
  
  //Delay to wait for sensors to load/stabilize
//  smartdelay(SAMPLERATE_DELAY_MS);
    smartdelay(1000); // change to above later

  if (SerialDebug) {
    Serial.println("Testing SD card: reading from test.txt");
    //  writeLineToFile("test.txt", "This is a test line!");
    readFileToConsole("test.txt");
    Serial.println("---------");
  }

  //set initial altitude
  initialAltitude = bmpChip.readAltitude(1013.25);
  lastAlt = initialAltitude;
}

void setupAltimiter() {
  Serial.begin(9600);  // Start serial for output

  myPressure.begin(); // Get sensor online

  //Configure the sensor
  myPressure.setModeAltimeter(); // Measure altitude above sea level in meters
  myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa

  myPressure.setOversampleRate(7); // Set Oversample to the recommended 128
  myPressure.enableEventFlags(); // Enable all three pressure and temp event flags 
}

void loop() {

  updateSensorData();
  logDataToSD();

  //if is deploying (i.e. cutting) currently, stop if exceeds timer
  if (deploying) {
    nichromeCounter += SAMPLERATE_DELAY_MS;
    if (nichromeCounter >= NICHROME_DELAY_MS) {
      digitalWrite(NICHROME_PIN, LOW);
      deploying = false;
    }
  }

  //use exponential moving average to get last vertical speed
  //https://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average
  float alpha = 0.4; //a higher alpha discounts older observations faster
  float newVertSpeed = (bmp.alt - lastAlt) / (float)(SAMPLERATE_DELAY_MS) / 1000.0;
  vertSpeed = alpha * newVertSpeed + (1.0-alpha)*vertSpeed;

  //if hasn't deployed already, check for met conditions
  if (!hasDeployed) {
    //Conditions for wing deployment (altitude and vertical speed)
    if (bmp.alt > initialAltitude + 1000 && vertSpeed < 0) {
      hasDeployed = true;
      deploying = true;
      digitalWrite(NICHROME_PIN, HIGH);
    }
  }
  
  //only turn servos if the wings have deployed
  if(hasDeployed) {
    //keep increasing the servo angle by one until it reaches the max angle 
    if(servo_angle_right < servo_max_angle_right) {
        right_elevator.write(servo_angle_right);
        left_elevator.write(servo_angle_left);
        servo_angle_right = servo_angle_right + 1;
        servo_angle_left = servo_angle_left - 1;
        
        
    }
  }
  

  if (SerialDebug) {
    if (sampleTimer >= SERIALDEBUG_RATE) {
      logDataToConsole();
      Serial.println("");
      sampleTimer = 0;
    }
    sampleTimer++;
  }

  

  // Delay the next loop by a predetermined sample rate
  // We use smartdelay() instead of delay() because we need to constantly
  // update the GPS library with info from the serial ports
  smartdelay(SAMPLERATE_DELAY_MS);
//  delay(1000);
}

/*
 * The gps library needs to constantly read from the serial for some reason
 * So instead of delaying, we just keep feeding it data until the desired
 * delay time has been reached
 */
static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gpsChip.encode(ss.read());
  } while (millis() - start < ms);
}

/*
 * Takes data from varous sensor input data structure and logs to serial
 */
void logDataToConsole() {
  
  //Log bmp data
  Serial.print("[BMP280] Temperature = ");
  Serial.print(bmp.temp);
  Serial.println(" *C");

  Serial.print("[BMP280] Pressure = ");
  Serial.print(bmp.pressure);
  Serial.println(" Pa");

  Serial.print("[BMP280] Approx altitude = ");
  Serial.print(bmp.alt); // this should be adjusted to your local forcase
  Serial.println(" m");


  //Log accelerometer data
  Serial.print("[ACCEL] ");
  Serial.print("X: ");
  Serial.print(accel.x, 4);
  Serial.print("\tY: ");
  Serial.print(accel.y, 4);
  Serial.print("\tZ: ");
  Serial.println(accel.z, 4);

  // Print acceleration values in milligs!
  Serial.print("[ACCEL] X-acceleration: "); Serial.print(1000 * myIMU.ax);
  Serial.print(" mg ");
  Serial.print("[ACCEL] Y-acceleration: "); Serial.print(1000 * myIMU.ay);
  Serial.print(" mg ");
  Serial.print("[ACCEL] Z-acceleration: "); Serial.print(1000 * myIMU.az);
  Serial.println(" mg ");
  
  // Print gyro values in degree/sec
  Serial.print("[ACCEL] X-gyro rate: "); Serial.print(myIMU.gx, 3);
  Serial.print(" degrees/sec ");
  Serial.print("[ACCEL] Y-gyro rate: "); Serial.print(myIMU.gy, 3);
  Serial.print(" degrees/sec ");
  Serial.print("[ACCEL] Z-gyro rate: "); Serial.print(myIMU.gz, 3);
  Serial.println(" degrees/sec");

  // Print mag values in degree/sec
  Serial.print("[ACCEL] X-mag field: "); Serial.print(myIMU.mx);
  Serial.print(" mG ");
  Serial.print("[ACCEL] Y-mag field: "); Serial.print(myIMU.my);
  Serial.print(" mG ");
  Serial.print("[ACCEL] Z-mag field: "); Serial.print(myIMU.mz);
  Serial.println(" mG");

  //Log GPS data
  Serial.print("[GPS] ");
  Serial.print("Satellites: ");
  Serial.print(gps.sats);
  Serial.print("\t precision: ");
  Serial.println(gps.precision);

  Serial.print("[GPS] ");
  Serial.print("latitude: ");
  Serial.print(String(gps.latitude, 5));
  Serial.print("\t longitude: ");
  Serial.println(String(gps.longitude, 5));

  Serial.print("[GPS] ");
  Serial.print("altitude: ");
  Serial.print(gps.alt);
  Serial.print("\t speed: ");
  Serial.print(gps.speed__kmph);
  Serial.print("\t check: ");
  Serial.println(gps.checksum);

  Serial.println("Servo right: ");
  Serial.print(servo_angle_right);
  Serial.println("Servo left: ");
  Serial.print(servo_angle_left);

}

void logDataToSD() {
  // Here we create a single line to input into the log file
  String dataLog = "";
  dataLog += millis();        dataLog += ","; // Timestamp
  
  dataLog += bmp.temp;        dataLog += ","; // Altimiter temperature
  dataLog += bmp.pressure;    dataLog += ","; // Altimiter pressure
  dataLog += bmp.alt;         dataLog += ","; // Altimiter altitude
  
  dataLog += String(accel.x, 4);         dataLog += ","; // Accelerometer x
  dataLog += String(accel.y, 4);         dataLog += ","; // Accelerometer y
  dataLog += String(accel.z, 4);         dataLog += ","; // Accelerometer z

  dataLog += String(myIMU.gx, 4);         dataLog += ","; // Accelerometer gyro x
  dataLog += String(myIMU.gy, 4);         dataLog += ","; // Accelerometer gyro y
  dataLog += String(myIMU.gz, 4);         dataLog += ","; // Accelerometer gyro z

  dataLog += String(myIMU.mx, 4);         dataLog += ","; // Accelerometer mag x
  dataLog += String(myIMU.my, 4);         dataLog += ","; // Accelerometer mag y
  dataLog += String(myIMU.mz, 4);         dataLog += ","; // Accelerometer mag z

  dataLog += servo_angle_right;        dataLog += ",";
  dataLog += servo_angle_left;         dataLog += ",";

  dataLog += gps.sats;        dataLog += ","; // GPS satellites
  dataLog += gps.precision;   dataLog += ","; // GPS reading precision

  // We use String(num, decimals) to force 5 decimal precision
  // According the the internet, 5 decimals gives 1.1132 m precision (very good)
  dataLog += String(gps.latitude, 6);    dataLog += ","; // GPS latitude
  dataLog += String(gps.longitude, 6);   dataLog += ","; // GPS longitude
  
  dataLog += gps.alt;         dataLog += ",";  //GPS altitude
  dataLog += gps.speed__kmph; dataLog += ",";  //GPS speed (kmph)
  dataLog += gps.checksum; //dataLog += ",";   //GPS # of checksum fails

//  Serial.println(dataLog);
//  Serial.println("");
  
  // Write the log to the log file
  // The log file is in CSV format for easy data analysis
   writeLineToFile("log.csv", dataLog);
}

/*
 * Initializes the SD Card at quarter speed
 * Returns true if initialization succeeded
 * 
 * Note: only one file may be opened at one time, so you
 * must close it before opening another file.
 */
bool initializeSd() {
  Serial.print("Initializing SD card...");

  //Initialize at quarter speed (may prevent various issues)
  if (!sd.begin(chipSelect, SPI_QUARTER_SPEED)) {
    Serial.println("initialization failed!");
    return false;
  }
  Serial.println("initialization done.");
  return true;
}

/*
 * Writes a single given line to a file.
 * 
 * This method opens and closes the file on every write,
 * which may cause slowdown but protects data in case of
 * sudden power failure or some other issue.
 */
void writeLineToFile(String filename, String testLine) {
  
  myFile = sd.open(filename, FILE_WRITE);
  // If file is successfully opened, proceed
  if (myFile) {
    Serial.print("Writing to ");
    Serial.print(filename);
    Serial.print("...  ");
    
    myFile.println(testLine);
    
    // Close the file:
    myFile.close();
    Serial.println("done writing.");
  } else {
    // if the file didn't open, print an error:
    Serial.print("error opening ");
    Serial.println(filename);
  }
}

/*
 * This method reads the contents of a file to the Serial Monitor
 */
void readFileToConsole(String filename) {
  
  myFile = sd.open(filename);

  // If file is successfully opened, proceed
  if (myFile) {
    Serial.print(filename);
    Serial.println(":");
    
    // Read from the file  to console until done
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    // Close the file:
    myFile.close();
  } else {
    // If the file didn't open, print an error:
    Serial.print("error opening ");
    Serial.println(filename);
  }
}
//ffff
// May or may not work
/*char* doubleToString(double d, int digits) {
  char str[digits];
  sprintf(str, "%f", d);
  return str;
}*/

/*
 * Updates the GPS data structure with current data
 * The GPS seems like it transmits data asynchronously
 * Sets to -1 if any reading fails
 */
void updateGPSData() {

  float flat, flon;
  unsigned long age;

  
  
  gps.sats = ((gpsChip.satellites() == TinyGPS::GPS_INVALID_SATELLITES) ? -1 : gpsChip.satellites());
  gps.precision = ((gpsChip.hdop() == TinyGPS::GPS_INVALID_HDOP) ? -1 : gpsChip.hdop());

  gpsChip.f_get_position(&flat, &flon, &age);
  gps.latitude = ((flat == TinyGPS::GPS_INVALID_F_ANGLE) ? -1 : flat);
  gps.longitude = ((flon == TinyGPS::GPS_INVALID_F_ANGLE) ? -1 : flon);
  
  gps.alt = ((gpsChip.f_altitude() == TinyGPS::GPS_INVALID_F_ALTITUDE) ? -1 : gpsChip.f_altitude());
  gps.speed__kmph = ((gpsChip.f_speed_kmph() == TinyGPS::GPS_INVALID_F_SPEED) ? -1 : gpsChip.f_speed_kmph());

  unsigned long chars = 0;
  unsigned short sentences = 0, failed = 0;

//  Serial.println("[GPS] Chars | Sentences | Checksum Fail");
  gpsChip.stats(&chars, &sentences, &failed);
  gps.checksum = failed;
//  Serial.print("[GPS] ");
//  print_int(chars, 0xFFFFFFFF, 10);
//  print_int(sentences, 0xFFFFFFFF, 10);
//  print_int(failed, 0xFFFFFFFF, 10);
//  Serial.println();
  
}

/*
 * Update BMP data structure with current reading
 * Make sure readAltitude() is populated with correct value
 */
void updateBMPData() {
  bmp.temp = bmpChip.readTemperature();
  bmp.pressure = bmpChip.readPressure();
  bmp.alt = bmpChip.readAltitude(1013.25); // this should be adjusted to your local forcase
}

/*
 * Updates the accelerometer data structure with current data
 */
void updateAccelData() {
//  sensors_event_t event;
//  bno.getEvent(&event);

//  accel.x = event.orientation.x;
//  accel.y = event.orientation.y;
//  accel.z = event.orientation.z;

  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
    
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
  
    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

    //--- new code
    accel.x = myIMU.ax;
    accel.y = myIMU.ay;
    accel.z = myIMU.az;
    //----
    
    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
  
    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;
  
    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
  
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
               * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
               * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
               * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  
  // Must be called before updating quaternions!
  myIMU.updateTime();
  
  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);
  
  if (!AHRS)
  {
    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > 500)
    {
//      if(SerialDebug)
//      {
//        myIMU.tempCount = myIMU.readTempData();  // Read the adc values
//        // Temperature in degrees Centigrade
//        myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;
//        // Print temperature in degrees Centigrade
//        Serial.print("Temperature is ");  Serial.print(myIMU.temperature, 1);
//        Serial.println(" degrees C");
//      }
  
      myIMU.count = millis();
    } // if (myIMU.delt_t > 500)
  } // if (!AHRS)
  else {
    // Serial print and/or display at 0.5 s rate independent of data rates
    myIMU.delt_t = millis() - myIMU.count;
  
    if (myIMU.delt_t > 500) {
      if(SerialDebug) {
        Serial.print("ax = ");  Serial.print((int)1000 * myIMU.ax);
        Serial.print(" ay = "); Serial.print((int)1000 * myIMU.ay);
        Serial.print(" az = "); Serial.print((int)1000 * myIMU.az);
        Serial.println(" mg");
  
        Serial.print("gx = ");  Serial.print(myIMU.gx, 2);
        Serial.print(" gy = "); Serial.print(myIMU.gy, 2);
        Serial.print(" gz = "); Serial.print(myIMU.gz, 2);
        Serial.println(" deg/s");
  
        Serial.print("mx = ");  Serial.print((int)myIMU.mx);
        Serial.print(" my = "); Serial.print((int)myIMU.my);
        Serial.print(" mz = "); Serial.print((int)myIMU.mz);
        Serial.println(" mG");
  
        Serial.print("q0 = ");  Serial.print(*getQ());
        Serial.print(" qx = "); Serial.print(*(getQ() + 1));
        Serial.print(" qy = "); Serial.print(*(getQ() + 2));
        Serial.print(" qz = "); Serial.println(*(getQ() + 3));
      }
  
  // Define output variables from updated quaternion---these are Tait-Bryan
  // angles, commonly used in aircraft orientation. In this coordinate system,
  // the positive z-axis is down toward Earth. Yaw is the angle between Sensor
  // x-axis and Earth magnetic North (or true North if corrected for local
  // declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the
  // Earth is positive, up toward the sky is negative. Roll is angle between
  // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
  // arise from the definition of the homogeneous rotation matrix constructed
  // from quaternions. Tait-Bryan angles as well as Euler angles are
  // non-commutative; that is, the get the correct orientation the rotations
  // must be applied in the correct order which for this configuration is yaw,
  // pitch, and then roll.
  // For more see
  // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  // which has additional links.
      myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                    * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                    * *(getQ()+3));
      myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                    * *(getQ()+2)));
      myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                    * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                    * *(getQ()+3));
      myIMU.pitch *= RAD_TO_DEG;
      myIMU.yaw   *= RAD_TO_DEG;
  
      myIMU.yaw  -= 13;
  
      //declination of stanford is 13° 19.14' East on 2019-02-22
      myIMU.roll *= RAD_TO_DEG;
  
      if(SerialDebug) {
        Serial.print("Yaw, Pitch, Roll: ");
        Serial.print(myIMU.yaw, 2);
        Serial.print(", ");
        Serial.print(myIMU.pitch, 2);
        Serial.print(", ");
        Serial.println(myIMU.roll, 2);
  
        Serial.print("rate = ");
        Serial.print((float)myIMU.sumCount / myIMU.sum, 2);
        Serial.println(" Hz");
      }
  
      myIMU.count = millis();
      myIMU.sumCount = 0;
      myIMU.sum = 0;
    } // if (myIMU.delt_t > 500)
  } // if (AHRS)
}

/*
 * Update altimiter data structure with current reading
 */
void updateAltimiterData() {

  //Question: Should we adjust altitude/pressure to local conditions?
  //This is the altitude code for BMP280 (temp/pressure)
  //  bmp.alt = bmpChip.readAltitude(1013.25); // this should be adjusted to your local forcase

  altimiter.alt = myPressure.readAltitude();
  altimiter.pressure = myPressure.readPressure();
  altimiter.temp = myPressure.readTemp();
}

/*
 * Updates sensor data 
 */
void updateSensorData() {
  updateAccelData();
//  updateAltimiterData();
  updateBMPData();
  updateGPSData();
}

void setupIMU() {
   Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(38400);

  while(!Serial){};

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print(F("MPU9250 I AM 0x"));
  Serial.print(c, HEX);
  Serial.print(F(" I should be 0x"));
  Serial.println(0x71, HEX);

  if (c == 0x71) // WHO_AM_I should always be 0x71
  {
    Serial.println(F("MPU9250 is online..."));

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.selfTest);
    Serial.print(F("x-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[0],1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[1],1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[2],1); Serial.println("% of factory value");
    Serial.print(F("x-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[3],1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[4],1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 ");
    Serial.print("I AM 0x");
    Serial.print(d, HEX);
    Serial.print(" I should be 0x");
    Serial.println(0x48, HEX);

    if (d != 0x48)
    {
      // Communication failed, stop here
      Serial.println(F("Communication failed, abort!"));
      Serial.flush();
      abort();
    }

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.factoryMagCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");

    if (SerialDebug)
    {
      //  Serial.println("Calibration values: ");
      Serial.print("X-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[2], 2);
    }

    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();

    // The next call delays for 4 seconds, and then records about 15 seconds of
    // data to calculate bias and scale.
//    myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
    Serial.println("AK8963 mag biases (mG)");
    Serial.println(myIMU.magBias[0]);
    Serial.println(myIMU.magBias[1]);
    Serial.println(myIMU.magBias[2]);

    Serial.println("AK8963 mag scale (mG)");
    Serial.println(myIMU.magScale[0]);
    Serial.println(myIMU.magScale[1]);
    Serial.println(myIMU.magScale[2]);
//    delay(2000); // Add delay to see results before serial spew of data

    if(SerialDebug)
    {
      Serial.println("Magnetometer:");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[2], 2);
    }
  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);

    // Communication failed, stop here
    Serial.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  }
}



//-------------- Printing Helper Functions
//From TinyGPS example
static void print_int(unsigned long val, unsigned long invalid, int len)
{
  char sz[32];
  if (val == invalid)
    strcpy(sz, "*******");
  else
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartdelay(0);
}

//from MPU/MPL Example
void print_float(double number, int digits)
{
  // Handle negative numbers
  if (number < 0.0) {
     Serial.print('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  Serial.print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    Serial.print("."); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0) {
    remainder *= 10.0;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint;
  }
}

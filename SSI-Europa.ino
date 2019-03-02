/* Library code from a varietty of sources for the gps, IMU, and baraometer
 */

#include "quaternionFilters.h"
#include "MPU9250.h"
#include <TinyGPS.h>

//---------------------gps setup ------------------

/* This sample code demonstrates the normal use of a TinyGPS object. */
TinyGPS gps;

/* On Teensy, the UART (real serial port) is always best to use. */
/* Unlike Arduino, there's no need to use NewSoftSerial because */
/* the "Serial" object uses the USB port, leaving the UART free. */
HardwareSerial Uart = HardwareSerial();

void gpsdump(TinyGPS &gps);
void printFloat(double f, int digits = 2);

//---------------------------------------------------------

//-------------------------Baraometer setup-----------
#include <Wire.h>
#include "SparkFunMPL3115A2.h"

//Create an instance of the object
MPL3115A2 myPressure;
//-----------------------------------------------

#ifdef LCD
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

// Using NOKIA 5110 monochrome 84 x 48 pixel display
// pin 9 - Serial clock out (SCLK)
// pin 8 - Serial data out (DIN)
// pin 7 - Data/Command select (D/C)
// pin 5 - LCD chip select (CS)
// pin 6 - LCD reset (RST)
Adafruit_PCD8544 display = Adafruit_PCD8544(9, 8, 7, 5, 6);
#endif // LCD

#define AHRS false         // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed  = 13;  // Set up pin 13 led for toggling

#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0   // Use either this line or the next to select which I2C address your device is using
//#define MPU9250_ADDRESS MPU9250_ADDRESS_AD1

MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);


void setup() {

  imuSetup();
  gpsSetup();
  altSetup();

  //servo pins 5 & 6
  //gps 0 & 1
  //sd 11, 12, 13, 20 (11-13 can be shared)
  //19 & 18
  //

}

void loop() {

  //------added code-------
  printAlt();                 //prints altitude and temperature; variable called "altitude"
  float altitude = myPressure.readAltitudeFt();         //altitude variable working
  Serial.println(" ");
  
  printOrientation();
  Serial.println(" ");
  delay(1000);

  float xGryoRate = myIMU.gx;
  float yGryoRate = myIMU.gy;
  float zGryoRate = myIMU.gz;
 
  Serial.print("X data: ");
  Serial.print(myIMU.gx);
  Serial.println(" ");


  //ToDo: implement servo movement, ematch, rest of sensor data

  //use atan2(differenceVector1) to get angle 
  //during flight z-axis won't necessarily be aligned with earth's z-axis
  

  
}

void gpsSetup () {
  Serial.begin(115200);
  Uart.begin(4800);
  
  delay(1000);
  Serial.print("Testing TinyGPS library v. "); Serial.println(TinyGPS::library_version());
  Serial.println("by Mikal Hart");
  Serial.println();
  Serial.print("Sizeof(gpsobject) = "); Serial.println(sizeof(TinyGPS));
  Serial.println();
}


void printGPS() {
  bool newdata = false;
  unsigned long start = millis();

  // Every 5 seconds we print an update
  while (millis() - start < 5000) {
    if (Uart.available()) {
      char c = Uart.read();
      // Serial.print(c);  // uncomment to see raw GPS data
      if (gps.encode(c)) {
        newdata = true;
        // break;  // uncomment to print new data immediately!
      }
    }
  }
  
  if (newdata) {
    Serial.println("Acquired Data");
    Serial.println("-------------");
    gpsdump(gps);
    Serial.println("-------------");
    Serial.println();
  }
}

void gpsdump(TinyGPS &gps)
{
  long lat, lon;
  float flat, flon;
  unsigned long age, date, time, chars;
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned short sentences, failed;

  gps.get_position(&lat, &lon, &age);
  Serial.print("Lat/Long(10^-5 deg): "); Serial.print(lat); Serial.print(", "); Serial.print(lon); 
  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");
  
  // On Arduino, GPS characters may be lost during lengthy Serial.print()
  // On Teensy, Serial prints to USB, which has large output buffering and
  //   runs very fast, so it's not necessary to worry about missing 4800
  //   baud GPS characters.

  gps.f_get_position(&flat, &flon, &age);
  Serial.print("Lat/Long(float): "); printFloat(flat, 5); Serial.print(", "); printFloat(flon, 5);
  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");

  gps.get_datetime(&date, &time, &age);
  Serial.print("Date(ddmmyy): "); Serial.print(date); Serial.print(" Time(hhmmsscc): ");
    Serial.print(time);
  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");

  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  Serial.print("Date: "); Serial.print(static_cast<int>(month)); Serial.print("/"); 
    Serial.print(static_cast<int>(day)); Serial.print("/"); Serial.print(year);
  Serial.print("  Time: "); Serial.print(static_cast<int>(hour)); Serial.print(":"); 
    Serial.print(static_cast<int>(minute)); Serial.print(":"); Serial.print(static_cast<int>(second));
    Serial.print("."); Serial.print(static_cast<int>(hundredths));
  Serial.print("  Fix age: ");  Serial.print(age); Serial.println("ms.");

  Serial.print("Alt(cm): "); Serial.print(gps.altitude()); Serial.print(" Course(10^-2 deg): ");
    Serial.print(gps.course()); Serial.print(" Speed(10^-2 knots): "); Serial.println(gps.speed());
  Serial.print("Alt(float): "); printFloat(gps.f_altitude()); Serial.print(" Course(float): ");
    printFloat(gps.f_course()); Serial.println();
  Serial.print("Speed(knots): "); printFloat(gps.f_speed_knots()); Serial.print(" (mph): ");
    printFloat(gps.f_speed_mph());
  Serial.print(" (mps): "); printFloat(gps.f_speed_mps()); Serial.print(" (kmph): ");
    printFloat(gps.f_speed_kmph()); Serial.println();

  gps.stats(&chars, &sentences, &failed);
  Serial.print("Stats: characters: "); Serial.print(chars); Serial.print(" sentences: ");
    Serial.print(sentences); Serial.print(" failed checksum: "); Serial.println(failed);
}

void printFloat(double number, int digits)
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

void printAlt () {
  //float altitude = myPressure.readAltitude();
  //Serial.print("Altitude(m):");
  //Serial.print(altitude, 2);

  float altitude = myPressure.readAltitudeFt();
  Serial.print(" Altitude(ft):");
  Serial.print(altitude, 2);

  //float pressure = myPressure.readPressure();
  //Serial.print("Pressure(Pa):");
  //Serial.print(pressure, 2);

  //float temperature = myPressure.readTemp();
  //Serial.print(" Temp(c):");
  //Serial.print(temperature, 2);

  float temperature = myPressure.readTempF();
  Serial.print(" Temp(f):");
  Serial.print(temperature, 2);
  Serial.println();
}

void altSetup () {
  Serial.begin(9600);  // Start serial for output

  myPressure.begin(); // Get sensor online

  //Configure the sensor
  myPressure.setModeAltimeter(); // Measure altitude above sea level in meters
  //myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa

  myPressure.setOversampleRate(7); // Set Oversample to the recommended 128
  myPressure.enableEventFlags(); // Enable all three pressure and temp event flags 
}

void printOrientation () {
    // If intPin goes high, all data registers have new data
    // On interrupt, check if data ready interrupt
    if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
    {
      myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
  
      // Now we'll calculate the accleration value into actual g's
      // This depends on scale being set
      myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
      myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
      myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];
  
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
        if(SerialDebug)
        {
          // Print acceleration values in milligs!
          Serial.print("X-acceleration: "); Serial.print(1000 * myIMU.ax);
          Serial.print(" mg ");
          Serial.print("Y-acceleration: "); Serial.print(1000 * myIMU.ay);
          Serial.print(" mg ");
          Serial.print("Z-acceleration: "); Serial.print(1000 * myIMU.az);
          Serial.println(" mg ");
  
          // Print gyro values in degree/sec
          Serial.print("X-gyro rate: "); Serial.print(myIMU.gx, 3);
          Serial.print(" degrees/sec ");
          Serial.print("Y-gyro rate: "); Serial.print(myIMU.gy, 3);
          Serial.print(" degrees/sec ");
          Serial.print("Z-gyro rate: "); Serial.print(myIMU.gz, 3);
          Serial.println(" degrees/sec");
  
          // Print mag values in degree/sec
          Serial.print("X-mag field: "); Serial.print(myIMU.mx);
          Serial.print(" mG ");
          Serial.print("Y-mag field: "); Serial.print(myIMU.my);
          Serial.print(" mG ");
          Serial.print("Z-mag field: "); Serial.print(myIMU.mz);
          Serial.println(" mG");
  
          myIMU.tempCount = myIMU.readTempData();  // Read the adc values
          // Temperature in degrees Centigrade
          myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;
          // Print temperature in degrees Centigrade
          Serial.print("Temperature is ");  Serial.print(myIMU.temperature, 1);
          Serial.println(" degrees C");
        }
  
  #ifdef LCD
        display.clearDisplay();
        display.setCursor(0, 0); display.print("MPU9250/AK8963");
        display.setCursor(0, 8); display.print(" x   y   z  ");
  
        display.setCursor(0,  16); display.print((int)(1000 * myIMU.ax));
        display.setCursor(24, 16); display.print((int)(1000 * myIMU.ay));
        display.setCursor(48, 16); display.print((int)(1000 * myIMU.az));
        display.setCursor(72, 16); display.print("mg");
  
        display.setCursor(0,  24); display.print((int)(myIMU.gx));
        display.setCursor(24, 24); display.print((int)(myIMU.gy));
        display.setCursor(48, 24); display.print((int)(myIMU.gz));
        display.setCursor(66, 24); display.print("o/s");
  
        display.setCursor(0,  32); display.print((int)(myIMU.mx));
        display.setCursor(24, 32); display.print((int)(myIMU.my));
        display.setCursor(48, 32); display.print((int)(myIMU.mz));
        display.setCursor(72, 32); display.print("mG");
  
        display.setCursor(0,  40); display.print("Gyro T ");
        display.setCursor(50,  40); display.print(myIMU.temperature, 1);
        display.print(" C");
        display.display();
  #endif // LCD
  
        myIMU.count = millis();
        digitalWrite(myLed, !digitalRead(myLed));  // toggle led
      } // if (myIMU.delt_t > 500)
    } // if (!AHRS)
    else
    {
      // Serial print and/or display at 0.5 s rate independent of data rates
      myIMU.delt_t = millis() - myIMU.count;
  
      // update LCD once per half-second independent of read rate
      if (myIMU.delt_t > 500)
      {
        if(SerialDebug)
        {
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
  
        if(SerialDebug)
        {
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
  
  #ifdef LCD
        display.clearDisplay();
  
        display.setCursor(0, 0); display.print(" x   y   z  ");
  
        display.setCursor(0,  8); display.print((int)(1000 * myIMU.ax));
        display.setCursor(24, 8); display.print((int)(1000 * myIMU.ay));
        display.setCursor(48, 8); display.print((int)(1000 * myIMU.az));
        display.setCursor(72, 8); display.print("mg");
  
        display.setCursor(0,  16); display.print((int)(myIMU.gx));
        display.setCursor(24, 16); display.print((int)(myIMU.gy));
        display.setCursor(48, 16); display.print((int)(myIMU.gz));
        display.setCursor(66, 16); display.print("o/s");
  
        display.setCursor(0,  24); display.print((int)(myIMU.mx));
        display.setCursor(24, 24); display.print((int)(myIMU.my));
        display.setCursor(48, 24); display.print((int)(myIMU.mz));
        display.setCursor(72, 24); display.print("mG");
  
        display.setCursor(0,  32); display.print((int)(myIMU.yaw));
        display.setCursor(24, 32); display.print((int)(myIMU.pitch));
        display.setCursor(48, 32); display.print((int)(myIMU.roll));
        display.setCursor(66, 32); display.print("ypr");
  
      // With these settings the filter is updating at a ~145 Hz rate using the
      // Madgwick scheme and >200 Hz using the Mahony scheme even though the
      // display refreshes at only 2 Hz. The filter update rate is determined
      // mostly by the mathematical steps in the respective algorithms, the
      // processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
      // an ODR of 10 Hz for the magnetometer produce the above rates, maximum
      // magnetometer ODR of 100 Hz produces filter update rates of 36 - 145 and
      // ~38 Hz for the Madgwick and Mahony schemes, respectively. This is
      // presumably because the magnetometer read takes longer than the gyro or
      // accelerometer reads. This filter update rate should be fast enough to
      // maintain accurate platform orientation for stabilization control of a
      // fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
      // produced by the on-board Digital Motion Processor of Invensense's MPU6050
      // 6 DoF and MPU9150 9DoF sensors. The 3.3 V 8 MHz Pro Mini is doing pretty
      // well!
        display.setCursor(0, 40); display.print("rt: ");
        display.print((float) myIMU.sumCount / myIMU.sum, 2);
        display.print(" Hz");
        display.display();
  #endif // LCD
  
        myIMU.count = millis();
        myIMU.sumCount = 0;
        myIMU.sum = 0;
      } // if (myIMU.delt_t > 500)
    } // if (AHRS)
}

void imuSetup () {
   Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(38400);

  while(!Serial){};

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);

#ifdef LCD
  display.begin(); // Ini8ialize the display
  display.setContrast(58); // Set the contrast

  // Start device display with ID of sensor
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0); display.print("MPU9250");
  display.setTextSize(1);
  display.setCursor(0, 20); display.print("9-DOF 16-bit");
  display.setCursor(0, 30); display.print("motion sensor");
  display.setCursor(20,40); display.print("60 ug LSB");
  display.display();

  // Set up for data display
  display.setTextSize(1); // Set text size to normal, 2 is twice normal etc.
  display.setTextColor(BLACK); // Set pixel color; 1 on the monochrome screen
  display.clearDisplay();   // clears the screen and buffer
#endif // LCD

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print(F("MPU9250 I AM 0x"));
  Serial.print(c, HEX);
  Serial.print(F(" I should be 0x"));
  Serial.println(0x71, HEX);

#ifdef LCD
  display.setCursor(20,0); display.print("MPU9250");
  display.setCursor(0,10); display.print("I AM");
  display.setCursor(0,20); display.print(c, HEX);
  display.setCursor(0,30); display.print("I Should Be");
  display.setCursor(0,40); display.print(0x71, HEX);
  display.display();
  delay(1000);
#endif // LCD

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

#ifdef LCD
    display.clearDisplay();

    display.setCursor(0, 0); display.print("MPU9250 bias");
    display.setCursor(0, 8); display.print(" x   y   z  ");

    display.setCursor(0,  16); display.print((int)(1000*myIMU.accelBias[0]));
    display.setCursor(24, 16); display.print((int)(1000*myIMU.accelBias[1]));
    display.setCursor(48, 16); display.print((int)(1000*myIMU.accelBias[2]));
    display.setCursor(72, 16); display.print("mg");

    display.setCursor(0,  24); display.print(myIMU.gyroBias[0], 1);
    display.setCursor(24, 24); display.print(myIMU.gyroBias[1], 1);
    display.setCursor(48, 24); display.print(myIMU.gyroBias[2], 1);
    display.setCursor(66, 24); display.print("o/s");

    display.display();
    delay(1000);
#endif // LCD

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

#ifdef LCD
    display.clearDisplay();
    display.setCursor(20,0); display.print("AK8963");
    display.setCursor(0,10); display.print("I AM");
    display.setCursor(0,20); display.print(d, HEX);
    display.setCursor(0,30); display.print("I Should Be");
    display.setCursor(0,40); display.print(0x48, HEX);
    display.display();
    delay(1000);
#endif // LCD

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

#ifdef LCD
    display.clearDisplay();
    display.setCursor(20,0);  display.print("AK8963");
    display.setCursor(0,10);  display.print("ASAX ");
    display.setCursor(50,10); display.print(myIMU.factoryMagCalibration[0], 2);
    display.setCursor(0,20);  display.print("ASAY ");
    display.setCursor(50,20); display.print(myIMU.factoryMagCalibration[1], 2);
    display.setCursor(0,30);  display.print("ASAZ ");
    display.setCursor(50,30); display.print(myIMU.factoryMagCalibration[2], 2);
    display.display();
    delay(1000);
#endif // LCD

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

#ifdef LCD
    display.clearDisplay();
    display.setCursor(20,0); display.print("AK8963");
    display.setCursor(0,10); display.print("ASAX "); display.setCursor(50,10);
    display.print(myIMU.factoryMagCalibration[0], 2);
    display.setCursor(0,20); display.print("ASAY "); display.setCursor(50,20);
    display.print(myIMU.factoryMagCalibration[1], 2);
    display.setCursor(0,30); display.print("ASAZ "); display.setCursor(50,30);
    display.print(myIMU.factoryMagCalibration[2], 2);
    display.display();
    delay(1000);
#endif // LCD
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

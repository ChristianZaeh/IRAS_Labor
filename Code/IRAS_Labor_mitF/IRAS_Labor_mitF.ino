/* Einbinden von Bibliotheken */
#include "SR04.h" // Ultraschallsensor
#include<Arduino.h> // IMU
#include<ArdusatSDK.h> // IMU
#include<math.h> // IMU
#include<Adafruit_Sensor.h> // Helligkeitssensor TSL2561
#include<Adafruit_TSL2561_U.h> // Helligkeitssensor TSL2561
#include<Wire.h> // I2C Kommunikation
#include<SPI.h>
#include<SD.h> // SD-Card Reader


/* Definiere genutzte PINs */
#define TRIG_PIN 4 // Trigger Pin Ultraschallsensor
#define ECHO_PIN 3 // Echo-Pin Ultraschallsensor

/* Erzeuge Objekte der verschiedenen Sensoren */
SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN); // erzeugt ein neues Objekt "sr04" vom 
                                     // Typ SR04 (Ultraschallsensor)

Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(0x39, 12345);
// erzeugt ein neues Objekt "tsl" vom Typ Adafruit_TSL2561_Unified mit der ID 12345 und der //I2C
// Adresse 0x39 (Helligkeitssensor)

sensors_event_t event; // erzeugt ein neues Objekt "event" vom
 // Typ sensors_event_t (IMU)

File myFile; // erzeugt ein neues Objekt "myFile" vom
 // Typ File (SD-Karte)

Acceleration accel; // erzeugt ein neues Objekt "accel" vom
 // Typ Acceleration (IMU/Beschleunigungssensor)

Magnetic mag; // erzeugt ein neues Objekt "mag" vom Typ
 // Magnetic (IMU/Magnetsensor)

Orientation orient(accel,mag); // erzeugt ein neues Objekt "orient" vom
 // Typ Orientation mit zwei Inputs (IMU)

Gyro gyro; //erzeugt ein neues Objekt "gyro" vom Typ Gyro
/* Deklariere Variablen */
//float distance; // erzeugt eine Fließkommavariable "distance" vom Typ float
float accel_mag;
float accel_max = 0;


/* Starte das Setup */
void setup() {
 Serial.begin(9600); // Starte Monitorausgabe mit der Baudrate 9600
 pinMode(10, OUTPUT); // definiere Pin Nr. 10 als Output (SD-Karte)
 delay(1000); // Pause von 1000 us (= 1 Sekunde)
 Serial.print(F("Initializing SD card...")); // Monitorausgabe
 if (!SD.begin(10)) // Ueberprüft, ob Karte im SD-Kartensensor eingesteckt ist
 {
 Serial.println("initialization failed!");
 return;
 }
 Serial.println(F("SD-Card Reader initialization done."));
 if(!tsl.begin()) // Ueberprueft, ob der Helligkeitssensor verbunden ist
 {
 Serial.print("Ooops, no TSL2561 detected ...");
 while(!tsl.begin());
 }
 if(!accel.begin()) // Ueberprueft ob der Beschl.Sensor auf der IMU angeschlossen ist
 {
 Serial.println("can't init IMU!");
 }
 tsl.enableAutoRange(true); // Helligkeitssensor stellt sich automatisch auf die
 // Helligkeitsbedingungen ein
 tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);
 
 accel.begin(); // startet den Beschleunigungssensor
 mag.begin(); // startet den Magnetsensor
 orient.begin(); //startet Orientierung
 gyro.begin(); //startet Gyroskop

  Serial.print("t [ms]");
  Serial.print("\t");
  Serial.print("D [cm]");
  Serial.print("\t");
  Serial.print("L [lux]");
  Serial.print("\t");
  Serial.print("Ax [m/s^2]");
  Serial.print("\t");
  Serial.print("Ay [m/s^2]");
  Serial.print("\t");
  Serial.print("Az [m/s^2]");
  Serial.print("\t");
  Serial.print("As [m/s^2]");
  Serial.print("\t");
  Serial.print("Am [m/s^2]");
  Serial.print("\t");
  Serial.print("Rx [1/s^2]");
  Serial.print("\t");
  Serial.print("Ry [1/s^2]");
  Serial.print("\t");
  Serial.print("Rz [1/s^2]");
  Serial.print("\t");
  Serial.print("Ox [°]");
  Serial.print("\t\t");
  Serial.print("Oy [°]");
  Serial.print("\t\t");
  Serial.println("Oz [°]");

myFile = SD.open("MWERTE.csv", FILE_WRITE);
  if (myFile) {
    Serial.print("Writing to MWERTE.csv...");
    myFile.print(F("t [ms]"));
    myFile.print(" ; ");
    myFile.print(F("D [cm]"));
    myFile.print(" ; ");
    myFile.print("L [lux]");
    myFile.print(" ; ");
    myFile.print(F("Ax [m/s^2]"));
    myFile.print(" ; ");
    myFile.print(F("Ay [m/s^2]"));
    myFile.print(" ; ");
    myFile.print(F("Az [m/s^2]"));
    myFile.print(" ; ");
    myFile.print("As[m/s^2]");
    myFile.print("  ;");
    myFile.print("Am[m/s^2]");
    myFile.print("  ;");
    myFile.print(F("Rx [1/s^2]"));
    myFile.print(" ; ");
    myFile.print(F("Ry [1/s^2]"));
    myFile.print(" ; ");
    myFile.print(F("Rz [1/s^2]"));
    myFile.print(" ; ");
    myFile.print(F("Ox [°]"));
    myFile.print(" ; ");
    myFile.print(F("Oy [°]"));
    myFile.print(" ; ");
    myFile.println(F("Oz [°]"));
    myFile.close();
  }
  else {
    Serial.println(F("error opening MWERTE.csv"));
    while (1);
  }
  

}
/* Ende der Setup-Funktion */


/* Starte die Loop-Funktion */
void loop() {

/*Zeit*/
 Serial.print(millis());
 Serial.print("\t");

/* Ultraschallsensor */
 //distance=sr04.Distance(); // misst Entfernung und speichert den Wert in "distance"
 Serial.print(sr04.Distance());
 Serial.print("\t");
 delay(50);

/* Helligkeitssensor */
 tsl.getEvent(&event); // misst die Helligkeit in Lux (vorgefertigte Funktion)
 if (event.light) // falls Helligkeit gemessen wurde, dann…
 {
 Serial.print(event.light);
 }
 else
 {
 Serial.println(F("Sensor overload"));
 while(1);
 }
 Serial.print("\t");
 delay(50);

/* Beschleunigungssensor */
 accel.read(); // wertet die Daten des Beschleunigungssensors aus
 gyro.read(); //wertet die Daten des Gyroskops aus
 orient.read();
 accel_mag = sqrt(accel.x*accel.x+accel.y*accel.y+accel.z*accel.z);

 if(accel_mag > accel_max) //gibt die bis zum jetzigen Zeitpunkt maximale Geschwindigkeit aus
 {
  accel_max = accel_mag;
 }
 
//Berechnet den Betrag der Gesamtbeschleunigung
 Serial.print(accel.x);
 Serial.print("\t\t");
 Serial.print(accel.y);
 Serial.print("\t\t");
 Serial.print(accel.z);
 Serial.print("\t\t");
 Serial.print(accel_mag);
 Serial.print("\t\t");
 Serial.print(accel_max);
 Serial.print("\t\t");
 Serial.print(gyro.x);
 Serial.print("\t\t");
 Serial.print(gyro.y);
 Serial.print("\t\t");
 Serial.print(gyro.z);
 Serial.print("\t\t");
 Serial.print(orient.roll);
 Serial.print("\t\t");
 Serial.print(orient.pitch);
 Serial.print("\t\t");
 Serial.print(orient.heading);
 Serial.println("\t\t");
 delay(100);

/* SD - Kartensensor */

myFile = SD.open("MWERTE.csv", FILE_WRITE);
 if (myFile) {
   Serial.print("Writing to MWERTE.csv...");
   myFile.print(millis());
   myFile.print(" ; ");
   myFile.print(sr04.Distance());
   myFile.print(" ; ");
   myFile.print(event.light);
   myFile.print(" ; ");
   myFile.print(accel.x);
   myFile.print(" ; ");
   myFile.print(accel.y);
   myFile.print(" ; ");
   myFile.print(accel.z);
   myFile.print(" ; ");
   myFile.print(accel_mag);
   myFile.print(" ; ");
   myFile.print(accel_max);
   myFile.print(" ; ");
   myFile.print(gyro.x);
   myFile.print(" ; ");
   myFile.print(gyro.y);
   myFile.print(" ; ");
   myFile.print(gyro.z);
   myFile.print(" ; ");
   myFile.print(orient.roll);
   myFile.print(" ; ");
   myFile.print(orient.pitch);
   myFile.print(" ; ");
   myFile.println(orient.heading);
   myFile.close();
   Serial.println(F("Done"));
 }
 else {
  Serial.println(F("error opening MWERTE.csv"));
  while(1);
 }
 delay(50);
}
/* Ende Loop-Funktion */

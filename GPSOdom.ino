#include <Adafruit_GPS.h>
#include <Wire.h>
#include <SparkFun_Seven_Segment.h>

// Create GPS module object
Adafruit_GPS GPS(&Serial1);
SevenSegmentDisplay display(XXX); // Replace XXX with the I2C address of your S7S Display

#define GPSECHO false
#define GPSRATE 10

double lastLatitude = 0.0;
double lastLongitude = 0.0;
double totalDistance = 0.0;

void setup() {
  Serial.begin(115200);
  display.begin();
  display.setBrightness(90);
  
  // Initialize GPS module
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  
  delay(1000);
  GPS.sendCommand(PGCMD_ANTENNA);
}

void loop() {
  char c = GPS.read();

  if (GPS.newNMEAreceived()) {
    if (GPS.parse(GPS.lastNMEA())) {
      calculateDistance();
      updateDisplay();
    }
  }
}

void calculateDistance() {
  if (lastLatitude == 0.0 && lastLongitude == 0.0) {
    lastLatitude = GPS.latitudeDegrees;
    lastLongitude = GPS.longitudeDegrees;
  }
  else {
    double distance = calculateHaversineDistance(lastLatitude, lastLongitude, GPS.latitudeDegrees, GPS.longitudeDegrees);
    totalDistance += distance;
    totalDistance = fmod(totalDistance, 1000.0); // Keep total distance under 1000.0
    lastLatitude = GPS.latitudeDegrees;
    lastLongitude = GPS.longitudeDegrees;
  }
}

double calculateHaversineDistance(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371.0; // Radius of earth in KM
  double dLat = (lat2 - lat1) * PI / 180.0;
  double dLon = (lon2 - lon1) * PI / 180.0;
  
  lat1 = (lat1) * PI / 180.0;
  lat2 = (lat2) * PI / 180.0;
  
  double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(lat1) * cos(lat2);
  double rad = 2 * atan2(sqrt(a), sqrt(1 - a));
  double distance = R * rad;
  
  return distance * 0.621371; // Convert from KM to miles
}

void updateDisplay() {
  display.clear();
  display.print(totalDistance, 1); // Print total distance with 1 decimal place
}

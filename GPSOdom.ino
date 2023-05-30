#include <Adafruit_GPS.h>
#include <Wire.h>

// Create GPS module object
Adafruit_GPS GPS(&Serial1);

#define GPSECHO false
#define GPSRATE 10

#define DISPLAY_ADDRESS 0x71 // Replace with your actual I2C address

double lastLatitude = 0.0;
double lastLongitude = 0.0;
double totalDistance = 0.0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
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
  char displayData[5];
  sprintf(displayData, "%04.1f", totalDistance); // Convert the floating point number to a string with one decimal place

  Wire.beginTransmission(DISPLAY_ADDRESS);
  Wire.write(displayData, 4);
  Wire.endTransmission();
}
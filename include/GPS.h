// #ifndef _GPS_H_
// #define _GPS_H_
// #include <Arduino.h>
// #include <SoftwareSerial.h>
// #include <TinyGPS++.h>
// #include <Pins.h>
// #include <States.h>
// #include <DataEntries.h>

// // instance
// TinyGPSPlus gps;

// // communication
// SoftwareSerial gps_serial(Pin::GPS_TX, Pin::GPS_RX);

// /*
//    Get position

//    this function return the latitude of the current position
//    of the robot
// */
// bool gps_location_found = false;
// void update_gps_position()
// {
//   while (gps_serial.available() > 0)
//   {
//     gps.encode(gps_serial.read());
//   }

//   if (gps.location.isUpdated())
//   {
//     gps_location_found = true;
//     NavigationEntry::hasStarted = true;
//   }
// }

// double get_gps_distance(
//     const double current_lat, 
//     const double current_lng,
//     const double destination_lat,
//     const double destination_lng
// ) {
//   if (!gps_location_found) return 0;
  
//   return TinyGPSPlus::distanceBetween(
//     current_lat,
//     current_lng,
//     destination_lat,
//     destination_lng
//   );
// }

// double get_gps_course(
//     const double current_lat, 
//     const double current_lng,
//     const double destination_lat,
//     const double destination_lng
//     ) 
// {
//   if(!gps_location_found)  return 0;

//   return TinyGPSPlus::courseTo(
//     current_lat, 
//     current_lng, 
//     destination_lat, 
//     destination_lng
//   );
// }


// #endif

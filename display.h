#ifndef DISPLAY_H
#define DISPLAY_H

#include <iostream>
#include <string>
#include <iomanip>
#include <sstream>
#include "simulation.h"
#include "intersection.h"
#include "parkinglot.h"
#include "vehicle.h"

using namespace std;

inline void displayStartupBanner() {
    cout << ("================================================================================\n");
    cout << ("              TRAFFIC INTERSECTION SIMULATION SYSTEM\n");
    cout << ("                      OS Project - Fall 2025\n");
    cout << ("================================================================================\n");
    cout << ("\n");
    cout << ("  Intersections: F10 (West) <---> F11 (East)\n");
    cout << ("  Parking Lots:  10 spots each with bounded waiting queue\n");
    cout << ("  Vehicle Types: Car, Bike, Bus, Tractor, Ambulance, Firetruck\n");
    cout << ("\n");
    cout << ("  Press Ctrl+C to initiate graceful shutdown\n");
    cout << ("================================================================================\n");
    cout << ("\n");
}

inline void displayShutdownBanner() {
    pthread_mutex_lock(&console_mutex);
    
    cout << ("\n");
    cout << ("================================================================================\n");
    cout << ("                          SIMULATION COMPLETE\n");
    cout << ("================================================================================\n");
    cout << ("  Total Vehicles Processed: " + to_string(vehicles_completed) + "\n");
    cout << ("================================================================================\n");
    cout << ("\n");
    
    pthread_mutex_unlock(&console_mutex);
}

inline void logVehicleSpawn(int id, string type, string intersection, string side, string direction) {
    safePrintWithTime("[SPAWN] Vehicle " + to_string(id) + " (" + type + ") spawned at " 
       + intersection + " " + side + " going " + direction);
}

inline void logVehicleEntry(int id, string type, string intersection, string side) {
    safePrintWithTime("[ENTRY] Vehicle " + to_string(id) + " (" + type + ") entered " 
       + intersection + " from " + side);
}

inline void logVehicleExit(int id, string type, string intersection, string side) {
    safePrintWithTime("[EXIT] Vehicle " + to_string(id) + " (" + type + ") exited " 
       + intersection + " via " + side);
}

inline void logVehicleTransit(int id, string type, string from_int, string to_int) {
    safePrintWithTime("[TRANSIT] Vehicle " + to_string(id) + " (" + type + ") moving from " 
       + from_int + " to " + to_int);
}

inline void logVehicleComplete(int id, string type) {
    safePrintWithTime("[COMPLETE] Vehicle " + to_string(id) + " (" + type + ") has exited the simulation");
}

inline void logParking(int id, string type, string intersection, bool entering) {
    safePrintWithTime("[PARKING] Vehicle " + to_string(id) + " (" + type + ") " 
       + string(entering ? "parked at" : "left parking at") + " " + intersection);
}

inline void logEmergency(string direction, bool starting) {
    safePrintWithTime("[EMERGENCY] " + string(starting ? "ACTIVATING" : "DEACTIVATING") 
       + " " + direction + " corridor");
}

inline void logLightChange(string intersection, string direction, string state) {
    safePrintWithTime("[LIGHT] " + intersection + " " + direction + " -> " + state);
}

#endif // DISPLAY_H

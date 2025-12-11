#ifndef INTERSECTION_H
#define INTERSECTION_H

#include <iostream>
#include <vector>
#include <string>
#include <semaphore.h>
#include "vehicle.h"

using namespace std;

struct TrafficController {
    string side;
    string light_state;
    vector<Vehicle> queue;
    
    TrafficController() : side("NORTH"), light_state("RED") {}
};

struct Intersection {
    string id;
    TrafficController north_controller;
    TrafficController south_controller;
    TrafficController east_controller;
    TrafficController west_controller;
    sem_t access_semaphore;
    bool emergency_mode;
    string emergency_entry_side;
    string emergency_exit_side;
    
    Intersection() : id("UNKNOWN"), emergency_mode(false) {
        sem_init(&access_semaphore, 0, 1);
    }
    
    ~Intersection() {
        sem_destroy(&access_semaphore);
    }
};

inline void initIntersection(Intersection& intersection, string id) {
    intersection.id = id;
    intersection.north_controller.side = "NORTH";
    intersection.north_controller.light_state = "RED";
    intersection.south_controller.side = "SOUTH";
    intersection.south_controller.light_state = "RED";
    intersection.east_controller.side = "EAST";
    intersection.east_controller.light_state = "RED";
    intersection.west_controller.side = "WEST";
    intersection.west_controller.light_state = "RED";
    intersection.emergency_mode = false;
    sem_init(&intersection.access_semaphore, 0, 1);
}

inline TrafficController& getController(Intersection& intersection, string side) {
    if (side == "NORTH") return intersection.north_controller;
    if (side == "SOUTH") return intersection.south_controller;
    if (side == "EAST") return intersection.east_controller;
    return intersection.west_controller;
}

inline void setControllerLight(Intersection& intersection, string side, string state) {
    sem_wait(&intersection.access_semaphore);
    
    if (side == "NORTH") intersection.north_controller.light_state = state;
    else if (side == "SOUTH") intersection.south_controller.light_state = state;
    else if (side == "EAST") intersection.east_controller.light_state = state;
    else if (side == "WEST") intersection.west_controller.light_state = state;
    
    sem_post(&intersection.access_semaphore);
}

inline string getControllerLight(Intersection& intersection, string side) {
    sem_wait(&intersection.access_semaphore);
    string state;
    
    if (side == "NORTH") state = intersection.north_controller.light_state;
    else if (side == "SOUTH") state = intersection.south_controller.light_state;
    else if (side == "EAST") state = intersection.east_controller.light_state;
    else state = intersection.west_controller.light_state;
    
    sem_post(&intersection.access_semaphore);
    return state;
}

inline bool canVehicleMove(Intersection& intersection, string from_side) {
    return getControllerLight(intersection, from_side) == "GREEN";
}

inline void setAllLightsRed(Intersection& intersection) {
    sem_wait(&intersection.access_semaphore);
    intersection.north_controller.light_state = "RED";
    intersection.south_controller.light_state = "RED";
    intersection.east_controller.light_state = "RED";
    intersection.west_controller.light_state = "RED";
    sem_post(&intersection.access_semaphore);
}

inline void setEmergencyMode(Intersection& intersection, bool active, 
                              string entry_side = "", string exit_side = "") {
    sem_wait(&intersection.access_semaphore);
    intersection.emergency_mode = active;
    intersection.emergency_entry_side = entry_side;
    intersection.emergency_exit_side = exit_side;
    sem_post(&intersection.access_semaphore);
}

inline bool isEmergencyMode(Intersection& intersection) {
    sem_wait(&intersection.access_semaphore);
    bool mode = intersection.emergency_mode;
    sem_post(&intersection.access_semaphore);
    return mode;
}

inline void activateEastboundEmergencyCorridor(Intersection& f10, Intersection& f11) {
    cout << ("========================================\n");
    cout << ("[EMERGENCY] EASTBOUND CORRIDOR ACTIVATED\n");
    cout << ("[EMERGENCY] Path: F10_WEST -> F10_EAST -> F11_WEST -> F11_EAST\n");
    cout << ("========================================\n");
    
    setAllLightsRed(f10);
    setAllLightsRed(f11);
    
    setControllerLight(f10, "WEST", "GREEN");
    setControllerLight(f10, "EAST", "GREEN");
    setEmergencyMode(f10, true, "WEST", "EAST");
    
    setControllerLight(f11, "WEST", "GREEN");
    setControllerLight(f11, "EAST", "GREEN");
    setEmergencyMode(f11, true, "WEST", "EAST");
    
    cout << ("[EMERGENCY] F10: WEST=GREEN, EAST=GREEN, others=RED\n");
    cout << ("[EMERGENCY] F11: WEST=GREEN, EAST=GREEN, others=RED\n");
}

inline void activateWestboundEmergencyCorridor(Intersection& f10, Intersection& f11) {
    cout << ("========================================\n");
    cout << ("[EMERGENCY] WESTBOUND CORRIDOR ACTIVATED\n");
    cout << ("[EMERGENCY] Path: F11_EAST -> F11_WEST -> F10_EAST -> F10_WEST\n");
    cout << ("========================================\n");
    
    setAllLightsRed(f10);
    setAllLightsRed(f11);
    
    setControllerLight(f11, "EAST", "GREEN");
    setControllerLight(f11, "WEST", "GREEN");
    setEmergencyMode(f11, true, "EAST", "WEST");
    
    setControllerLight(f10, "EAST", "GREEN");
    setControllerLight(f10, "WEST", "GREEN");
    setEmergencyMode(f10, true, "EAST", "WEST");
    
    cout << ("[EMERGENCY] F11: EAST=GREEN, WEST=GREEN, others=RED\n");
    cout << ("[EMERGENCY] F10: EAST=GREEN, WEST=GREEN, others=RED\n");
}

inline void deactivateEmergencyCorridor(Intersection& f10, Intersection& f11) {
    cout << ("========================================\n");
    cout << ("[EMERGENCY] CORRIDOR DEACTIVATED\n");
    cout << ("[EMERGENCY] Resuming normal traffic operations\n");
    cout << ("========================================\n");
    
    setEmergencyMode(f10, false);
    setEmergencyMode(f11, false);
    setAllLightsRed(f10);
    setAllLightsRed(f11);
}

inline void deactivateEmergencyMode(Intersection& intersection) {
    setEmergencyMode(intersection, false);
    setAllLightsRed(intersection);
    cout << ("[EMERGENCY] " + intersection.id + ": Emergency mode deactivated, resuming normal operation\n");
}

inline void printIntersection(Intersection& intersection) {
    sem_wait(&intersection.access_semaphore);
    
    cout << ("========================================\n");
    cout << ("INTERSECTION: " + intersection.id + "\n");
    cout << ("========================================\n");
    cout << ("Emergency Mode: " + string(intersection.emergency_mode ? "ACTIVE" : "INACTIVE") + "\n");
    cout << ("----------------------------------------\n");
    cout << ("Traffic Controllers:\n");
    cout << ("  NORTH: [" + intersection.north_controller.light_state + "] - " 
         + to_string(intersection.north_controller.queue.size()) + " vehicles waiting\n");
    cout << ("  SOUTH: [" + intersection.south_controller.light_state + "] - "
         + to_string(intersection.south_controller.queue.size()) + " vehicles waiting\n");
    cout << ("  EAST:  [" + intersection.east_controller.light_state + "] - "
         + to_string(intersection.east_controller.queue.size()) + " vehicles waiting\n");
    cout << ("  WEST:  [" + intersection.west_controller.light_state + "] - "
         + to_string(intersection.west_controller.queue.size()) + " vehicles waiting\n");
    cout << ("========================================\n");
    
    sem_post(&intersection.access_semaphore);
}

inline void printLightChange(string intersection_id, string side, string new_state) {
    cout << ("[SIGNAL] " + intersection_id + "_" + side + ": " + new_state + "\n");
}

#endif // INTERSECTION_H

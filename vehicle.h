#ifndef VEHICLE_H
#define VEHICLE_H

#include <iostream>
#include <string>
#include <time.h>

using namespace std;

struct Vehicle {
    int id;
    string type;
    string spawn_intersection;
    string spawn_side;
    string direction;
    string current_intersection;
    string current_side;
    string priority;
    time_t arrival_time;
    bool wants_parking;
    bool has_exited;
    
    Vehicle(int vid, string vtype, string spawn_int, string side, string dir) {
        id = vid;
        type = vtype;
        spawn_intersection = spawn_int;
        spawn_side = side;
        direction = dir;
        current_intersection = spawn_int;
        current_side = side;
        arrival_time = time(NULL);
        wants_parking = false;
        has_exited = false;
        
        if (type == "Ambulance" || type == "Firetruck") {
            priority = "HIGH";
            wants_parking = false;
        }
        else if (type == "Bus") {
            priority = "MEDIUM";
        }
        else {
            priority = "LOW";
        }
    }
    
    Vehicle() : id(0), type("Car"), spawn_intersection("F10"), spawn_side("NORTH"),
                direction("STRAIGHT"), current_intersection("F10"), current_side("NORTH"),
                priority("LOW"), arrival_time(0), wants_parking(false), has_exited(false) {}
};

inline bool isEmergencyVehicle(string type) {
    return (type == "Ambulance" || type == "Firetruck");
}

inline bool isValidEmergencySpawn(string spawn_intersection, string spawn_side, string direction) {
    if (direction != "STRAIGHT") return false;
    if (spawn_intersection == "F10" && spawn_side == "WEST") return true;
    if (spawn_intersection == "F11" && spawn_side == "EAST") return true;
    return false;
}

inline string getEmergencyPath(string spawn_intersection, string spawn_side) {
    if (spawn_intersection == "F10" && spawn_side == "WEST") {
        return "F10_WEST -> F10_EAST -> F11_WEST -> F11_EAST";
    }
    else if (spawn_intersection == "F11" && spawn_side == "EAST") {
        return "F11_EAST -> F11_WEST -> F10_EAST -> F10_WEST";
    }
    return "INVALID";
}

inline string getEmergencyDirection(string spawn_intersection, string spawn_side) {
    if (spawn_intersection == "F10" && spawn_side == "WEST") return "EASTBOUND";
    if (spawn_intersection == "F11" && spawn_side == "EAST") return "WESTBOUND";
    return "INVALID";
}

inline string getOppositeSide(string side) {
    if (side == "NORTH") return "SOUTH";
    if (side == "SOUTH") return "NORTH";
    if (side == "EAST") return "WEST";
    if (side == "WEST") return "EAST";
    return side;
}

inline string getExitSide(string entry_side, string direction) {
    if (direction == "STRAIGHT") {
        return getOppositeSide(entry_side);
    }
    else if (direction == "LEFT") {
        if (entry_side == "NORTH") return "EAST";
        if (entry_side == "SOUTH") return "WEST";
        if (entry_side == "EAST") return "SOUTH";
        if (entry_side == "WEST") return "NORTH";
    }
    else if (direction == "RIGHT") {
        if (entry_side == "NORTH") return "WEST";
        if (entry_side == "SOUTH") return "EAST";
        if (entry_side == "EAST") return "NORTH";
        if (entry_side == "WEST") return "SOUTH";
    }
    return entry_side;
}

inline bool willTransitionToOtherIntersection(string current_int, string exit_side) {
    if (current_int == "F10" && exit_side == "EAST") return true;
    if (current_int == "F11" && exit_side == "WEST") return true;
    return false;
}

inline void getNextIntersection(string current_int, string exit_side, 
                                 string& next_int, string& entry_side) {
    if (current_int == "F10" && exit_side == "EAST") {
        next_int = "F11";
        entry_side = "WEST";
    }
    else if (current_int == "F11" && exit_side == "WEST") {
        next_int = "F10";
        entry_side = "EAST";
    }
    else {
        next_int = "EXITED";
        entry_side = "NONE";
    }
}

inline void printVehicle(const Vehicle& v) {
    cout << ("----------------------------------------\n");
    cout << ("Vehicle ID: " + to_string(v.id) + "\n");
    cout << ("  Type: " + v.type + "\n");
    cout << ("  Priority: " + v.priority + "\n");
    cout << ("  Spawn: " + v.spawn_intersection + " [" + v.spawn_side + "]\n");
    cout << ("  Direction: " + v.direction + "\n");
    cout << ("  Current Location: " + v.current_intersection + " [" + v.current_side + "]\n");
    cout << ("  Wants Parking: " + string(v.wants_parking ? "Yes" : "No") + "\n");
    cout << ("  Status: " + string(v.has_exited ? "EXITED" : "ACTIVE") + "\n");
    cout << ("----------------------------------------\n");
}

inline void printVehicleEntry(const Vehicle& v, string intersection) {
    cout << ("[ENTRY] Vehicle " + to_string(v.id) + " (" + v.type + ") entered " 
         + intersection + " from " + v.current_side + " going " + v.direction + "\n");
}

inline void printVehicleExit(const Vehicle& v, string intersection, string exit_side) {
    cout << ("[EXIT] Vehicle " + to_string(v.id) + " (" + v.type + ") exited " 
         + intersection + " via " + exit_side + "\n");
}

inline void printVehicleTransition(const Vehicle& v, string from_int, string to_int) {
    cout << ("[TRANSIT] Vehicle " + to_string(v.id) + " (" + v.type + ") moving from " 
         + from_int + " to " + to_int + "\n");
}

#endif // VEHICLE_H

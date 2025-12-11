#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <iostream>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include "simulation.h"
#include "intersection.h"

using namespace std;

extern int f10_current_phase;
extern int f11_current_phase;

inline void cycleNorthSouth(Intersection& intersection) {
    if (isEmergencyMode(intersection)) return;
    
    setControllerLight(intersection, "NORTH", "GREEN");
    setControllerLight(intersection, "SOUTH", "GREEN");
    setControllerLight(intersection, "EAST", "RED");
    setControllerLight(intersection, "WEST", "RED");
    
    safePrintWithTime("[SIGNAL] " + intersection.id + ": NORTH-SOUTH GREEN, EAST-WEST RED");
    
    usleep(GREEN_DURATION);
    
    if (isEmergencyMode(intersection) || shutdown_flag) return;
    
    setControllerLight(intersection, "NORTH", "YELLOW");
    setControllerLight(intersection, "SOUTH", "YELLOW");
    
    safePrintWithTime("[SIGNAL] " + intersection.id + ": NORTH-SOUTH YELLOW");
    
    usleep(YELLOW_DURATION);
    
    if (isEmergencyMode(intersection) || shutdown_flag) return;
    
    setControllerLight(intersection, "NORTH", "RED");
    setControllerLight(intersection, "SOUTH", "RED");
}

inline void cycleEastWest(Intersection& intersection) {
    if (isEmergencyMode(intersection)) return;
    
    setControllerLight(intersection, "EAST", "GREEN");
    setControllerLight(intersection, "WEST", "GREEN");
    setControllerLight(intersection, "NORTH", "RED");
    setControllerLight(intersection, "SOUTH", "RED");
    
    safePrintWithTime("[SIGNAL] " + intersection.id + ": EAST-WEST GREEN, NORTH-SOUTH RED");
    
    usleep(GREEN_DURATION);
    
    if (isEmergencyMode(intersection) || shutdown_flag) return;
    
    setControllerLight(intersection, "EAST", "YELLOW");
    setControllerLight(intersection, "WEST", "YELLOW");
    
    safePrintWithTime("[SIGNAL] " + intersection.id + ": EAST-WEST YELLOW");
    
    usleep(YELLOW_DURATION);
    
    if (isEmergencyMode(intersection) || shutdown_flag) return;
    
    setControllerLight(intersection, "EAST", "RED");
    setControllerLight(intersection, "WEST", "RED");
}

inline void sendToController(int write_fd, char message) {
    write(write_fd, &message, 1);
}

inline bool checkForMessage(int read_fd, char& message) {
    int flags = fcntl(read_fd, F_GETFL, 0);
    fcntl(read_fd, F_SETFL, flags | O_NONBLOCK);
    
    ssize_t result = read(read_fd, &message, 1);
    
    fcntl(read_fd, F_SETFL, flags);
    
    return (result > 0);
}

inline void* f10ControllerThread(void* arg) {
    Intersection* f10 = (Intersection*)arg;
    
    safePrintWithTime("[CONTROLLER] F10 Traffic Controller started");
    
    int phase = 0;
    
    while (!shutdown_flag) {
        char msg;
        if (checkForMessage(pipe_f11_to_f10[0], msg)) {
            if (msg == MSG_EMERGENCY_WESTBOUND) {
                safePrintWithTime("[CONTROLLER] F10 received WESTBOUND emergency alert from F11");
            }
            else if (msg == MSG_EMERGENCY_CLEAR) {
                safePrintWithTime("[CONTROLLER] F10 received emergency clear signal");
            }
            else if (msg == MSG_SHUTDOWN) {
                break;
            }
        }
        
        if (emergency_active) {
            usleep(100000);
            continue;
        }
        
        if (phase == 0) {
            cycleNorthSouth(*f10);
            phase = 1;
        } else {
            cycleEastWest(*f10);
            phase = 0;
        }
    }
    
    safePrintWithTime("[CONTROLLER] F10 Traffic Controller shutting down");
    return NULL;
}

inline void* f11ControllerThread(void* arg) {
    Intersection* f11 = (Intersection*)arg;
    
    safePrintWithTime("[CONTROLLER] F11 Traffic Controller started");
    
    int phase = 0;
    
    while (!shutdown_flag) {
        char msg;
        if (checkForMessage(pipe_f10_to_f11[0], msg)) {
            if (msg == MSG_EMERGENCY_EASTBOUND) {
                safePrintWithTime("[CONTROLLER] F11 received EASTBOUND emergency alert from F10");
            }
            else if (msg == MSG_EMERGENCY_CLEAR) {
                safePrintWithTime("[CONTROLLER] F11 received emergency clear signal");
            }
            else if (msg == MSG_SHUTDOWN) {
                break;
            }
        }
        
        if (emergency_active) {
            usleep(100000);
            continue;
        }
        
        if (phase == 0) {
            cycleEastWest(*f11);
            phase = 1;
        } else {
            cycleNorthSouth(*f11);
            phase = 0;
        }
    }
    
    safePrintWithTime("[CONTROLLER] F11 Traffic Controller shutting down");
    return NULL;
}

inline void handleEmergencyVehicle(Intersection& f10, Intersection& f11, 
                                    string spawn_intersection, string spawn_side) {
    pthread_mutex_lock(&stats_mutex);
    emergency_active = true;
    
    if (spawn_intersection == "F10" && spawn_side == "WEST") {
        emergency_direction = "EASTBOUND";
        pthread_mutex_unlock(&stats_mutex);
        activateEastboundEmergencyCorridor(f10, f11);
        sendToController(pipe_f10_to_f11[1], MSG_EMERGENCY_EASTBOUND);
    }
    else if (spawn_intersection == "F11" && spawn_side == "EAST") {
        emergency_direction = "WESTBOUND";
        pthread_mutex_unlock(&stats_mutex);
        activateWestboundEmergencyCorridor(f10, f11);
        sendToController(pipe_f11_to_f10[1], MSG_EMERGENCY_WESTBOUND);
    }
    else {
        pthread_mutex_unlock(&stats_mutex);
        safePrintWithTime("[ERROR] Invalid emergency vehicle spawn location!");
    }
}

inline void clearEmergency(Intersection& f10, Intersection& f11) {
    pthread_mutex_lock(&stats_mutex);
    emergency_active = false;
    emergency_direction = "";
    pthread_mutex_unlock(&stats_mutex);
    
    deactivateEmergencyCorridor(f10, f11);
    
    sendToController(pipe_f10_to_f11[1], MSG_EMERGENCY_CLEAR);
    sendToController(pipe_f11_to_f10[1], MSG_EMERGENCY_CLEAR);
}

inline string processVehicleCrossing(Intersection& intersection, Vehicle& vehicle) {
    string entry_side = vehicle.current_side;
    string exit_side = getExitSide(entry_side, vehicle.direction);
    
    bool is_emergency = isEmergencyVehicle(vehicle.type);
    
    if (!is_emergency) {
        while (!canVehicleMove(intersection, entry_side) && !shutdown_flag) {
            usleep(100000);
        }
    }
    
    if (shutdown_flag) return exit_side;
    
    safePrintWithTime("[CROSSING] Vehicle " + to_string(vehicle.id) + " (" + vehicle.type + 
                      ") crossing " + intersection.id + " from " + entry_side + " to " + exit_side);
    
    usleep(CROSSING_TIME);
    
    vehicle.current_side = exit_side;
    
    return exit_side;
}

#endif // CONTROLLER_H

#ifndef SIMULATION_H
#define SIMULATION_H

#include <iostream>
#include <vector>
#include <string>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <signal.h>
#include <cstdlib>
#include <ctime>

using namespace std;

// Configuration
const int DEFAULT_VEHICLE_COUNT = 15;

// Time delays (microseconds)
const int SPAWN_MIN_DELAY = 500000;
const int SPAWN_MAX_DELAY = 2000000;
const int CROSSING_TIME = 1000000;
const int GREEN_DURATION = 3000000;
const int YELLOW_DURATION = 1000000;
const int PARKING_MIN_TIME = 2000000;
const int PARKING_MAX_TIME = 5000000;

// Vehicle type distribution (out of 100)
const int PROB_CAR = 40;
const int PROB_BIKE = 20;
const int PROB_BUS = 15;
const int PROB_TRACTOR = 10;
const int PROB_AMBULANCE = 8;
const int PROB_FIRETRUCK = 7;

const int PARKING_PROBABILITY = 30;

// Direction distribution (out of 100)
const int PROB_STRAIGHT = 50;
const int PROB_LEFT = 25;
const int PROB_RIGHT = 25;

// Synchronization primitives
extern pthread_mutex_t console_mutex;
extern pthread_mutex_t vehicle_mutex;
extern pthread_mutex_t f10_mutex;
extern pthread_mutex_t f11_mutex;
extern pthread_mutex_t stats_mutex;

extern pthread_cond_t f10_north_south_cond;
extern pthread_cond_t f10_east_west_cond;
extern pthread_cond_t f11_north_south_cond;
extern pthread_cond_t f11_east_west_cond;

extern bool shutdown_flag;
extern bool emergency_active;
extern string emergency_direction;

extern int next_vehicle_id;
extern int vehicles_completed;
extern int total_vehicles_to_spawn;

// Pipes for IPC
extern int pipe_f10_to_f11[2];
extern int pipe_f11_to_f10[2];

const char MSG_EMERGENCY_EASTBOUND = 'E';
const char MSG_EMERGENCY_WESTBOUND = 'W';
const char MSG_EMERGENCY_CLEAR = 'C';
const char MSG_SHUTDOWN = 'S';

struct VehicleThreadData {
    int vehicle_id;
    string type;
    string spawn_intersection;
    string spawn_side;
    string direction;
    bool wants_parking;
};

struct ControllerData {
    string intersection_id;
    int read_pipe;
    int write_pipe;
};

const string VEHICLE_TYPES[] = {"Car", "Bike", "Bus", "Tractor", "Ambulance", "Firetruck"};
const int NUM_VEHICLE_TYPES = 6;

const string SPAWN_SIDES[] = {"NORTH", "SOUTH", "EAST", "WEST"};
const int NUM_SIDES = 4;

const string DIRECTIONS[] = {"STRAIGHT", "LEFT", "RIGHT"};
const int NUM_DIRECTIONS = 3;

inline string getRandomVehicleType() {
    int r = rand() % 100;
    
    if (r < PROB_CAR) return "Car";
    r -= PROB_CAR;
    
    if (r < PROB_BIKE) return "Bike";
    r -= PROB_BIKE;
    
    if (r < PROB_BUS) return "Bus";
    r -= PROB_BUS;
    
    if (r < PROB_TRACTOR) return "Tractor";
    r -= PROB_TRACTOR;
    
    if (r < PROB_AMBULANCE) return "Ambulance";
    
    return "Firetruck";
}

inline string getRandomSpawnSide() {
    return SPAWN_SIDES[rand() % NUM_SIDES];
}

inline string getRandomDirection() {
    int r = rand() % 100;
    
    if (r < PROB_STRAIGHT) return "STRAIGHT";
    r -= PROB_STRAIGHT;
    
    if (r < PROB_LEFT) return "LEFT";
    
    return "RIGHT";
}

inline string getRandomIntersection() {
    return (rand() % 2 == 0) ? "F10" : "F11";
}

inline bool shouldWantParking(string vehicle_type) {
    if (vehicle_type == "Ambulance" || vehicle_type == "Firetruck") {
        return false;
    }
    return (rand() % 100) < PARKING_PROBABILITY;
}

inline int getRandomDelay(int min_delay, int max_delay) {
    return min_delay + (rand() % (max_delay - min_delay + 1));
}

inline void safePrint(const string& message) {
    pthread_mutex_lock(&console_mutex);
    cout << message << endl;
    pthread_mutex_unlock(&console_mutex);
}

inline string getTimestamp() {
    time_t now = time(NULL);
    struct tm* timeinfo = localtime(&now);
    char buffer[9];
    strftime(buffer, sizeof(buffer), "%H:%M:%S", timeinfo);
    return string(buffer);
}

inline void safePrintWithTime(const string& message) {
    pthread_mutex_lock(&console_mutex);
    cout << "[" << getTimestamp() << "] " << message << endl;
    pthread_mutex_unlock(&console_mutex);
}

#endif // SIMULATION_H

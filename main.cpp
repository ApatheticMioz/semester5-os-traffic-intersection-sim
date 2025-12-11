// Traffic Intersection Simulation - OS Project
// Compile with: g++ -pthread -o traffic_sim main.cpp

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <cstring>
#include <csignal>
#include <vector>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>

#ifndef _WIN32
#include <sys/wait.h>
#include <sys/select.h>
#endif

#include "simulation.h"
#include "vehicle.h"
#include "intersection.h"
#include "parkinglot.h"
#include "controller.h"
#include "display.h"

using namespace std;

// Global variable definitions
pthread_mutex_t console_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t vehicle_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t f10_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t f11_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t stats_mutex = PTHREAD_MUTEX_INITIALIZER;

pthread_cond_t f10_north_south_cond = PTHREAD_COND_INITIALIZER;
pthread_cond_t f10_east_west_cond = PTHREAD_COND_INITIALIZER;
pthread_cond_t f11_north_south_cond = PTHREAD_COND_INITIALIZER;
pthread_cond_t f11_east_west_cond = PTHREAD_COND_INITIALIZER;

bool shutdown_flag = false;
bool emergency_active = false;
string emergency_direction = "";
int vehicles_completed = 0;
int total_vehicles_to_spawn = DEFAULT_VEHICLE_COUNT;
int next_vehicle_id = 1;

int pipe_f10_to_f11[2];
int pipe_f11_to_f10[2];
int pipe_f10_to_parent[2];
int pipe_f11_to_parent[2];

Intersection intersection_f10;
Intersection intersection_f11;
ParkingLot parking_f10;
ParkingLot parking_f11;

vector<pthread_t> vehicle_threads;

volatile sig_atomic_t child_shutdown_flag = 0;

void childSignalHandler(int signum) {
    child_shutdown_flag = 1;
}

void signalHandler(int signum) {
    safePrintWithTime("SIGNAL: Shutdown signal received. Initiating graceful shutdown...");
    shutdown_flag = true;
    
    pthread_cond_broadcast(&f10_north_south_cond);
    pthread_cond_broadcast(&f10_east_west_cond);
    pthread_cond_broadcast(&f11_north_south_cond);
    pthread_cond_broadcast(&f11_east_west_cond);
}

void* vehicleThread(void* arg) {
    Vehicle* v = (Vehicle*)arg;
    
    logVehicleSpawn(v->id, v->type, v->spawn_intersection, v->spawn_side, v->direction);
    
    Intersection* current_intersection_ptr;
    ParkingLot* current_parking;
    pthread_mutex_t* current_mutex;
    pthread_cond_t* ns_cond;
    pthread_cond_t* ew_cond;
    
    while (!v->has_exited && !shutdown_flag) {
        if (v->current_intersection == "F10") {
            current_intersection_ptr = &intersection_f10;
            current_parking = &parking_f10;
            current_mutex = &f10_mutex;
            ns_cond = &f10_north_south_cond;
            ew_cond = &f10_east_west_cond;
        } else {
            current_intersection_ptr = &intersection_f11;
            current_parking = &parking_f11;
            current_mutex = &f11_mutex;
            ns_cond = &f11_north_south_cond;
            ew_cond = &f11_east_west_cond;
        }
        
        TrafficController* controller;
        if (v->current_side == "NORTH") {
            controller = &current_intersection_ptr->north_controller;
        } else if (v->current_side == "SOUTH") {
            controller = &current_intersection_ptr->south_controller;
        } else if (v->current_side == "EAST") {
            controller = &current_intersection_ptr->east_controller;
        } else {
            controller = &current_intersection_ptr->west_controller;
        }
        
        // Emergency vehicle handling
        if (v->priority == "HIGH") {
            pthread_mutex_lock(current_mutex);
            
            if (v->spawn_intersection == "F10" && v->spawn_side == "WEST") {
                logEmergency("EASTBOUND", true);
                emergency_active = true;
                emergency_direction = "EASTBOUND";
                activateEastboundEmergencyCorridor(intersection_f10, intersection_f11);
            } else if (v->spawn_intersection == "F11" && v->spawn_side == "EAST") {
                logEmergency("WESTBOUND", true);
                emergency_active = true;
                emergency_direction = "WESTBOUND";
                activateWestboundEmergencyCorridor(intersection_f10, intersection_f11);
            }
            
            pthread_mutex_unlock(current_mutex);
            
            usleep(CROSSING_TIME);
            logVehicleEntry(v->id, v->type, v->current_intersection, v->current_side);
            
            usleep(CROSSING_TIME);
            
            string exit_side = getExitSide(v->current_side, v->direction);
            logVehicleExit(v->id, v->type, v->current_intersection, exit_side);
            
            if (willTransitionToOtherIntersection(v->current_intersection, getExitSide(v->current_side, v->direction))) {
                string next_int = (v->current_intersection == "F10") ? "F11" : "F10";
                logVehicleTransit(v->id, v->type, v->current_intersection, next_int);
                
                v->current_intersection = next_int;
                v->current_side = (next_int == "F11") ? "WEST" : "EAST";
                
                usleep(CROSSING_TIME);
                exit_side = getExitSide(v->current_side, v->direction);
                logVehicleExit(v->id, v->type, v->current_intersection, exit_side);
            }
            
            pthread_mutex_lock(&f10_mutex);
            pthread_mutex_lock(&f11_mutex);
            
            logEmergency(emergency_direction, false);
            emergency_active = false;
            emergency_direction = "";
            deactivateEmergencyCorridor(intersection_f10, intersection_f11);
            
            pthread_mutex_unlock(&f11_mutex);
            pthread_mutex_unlock(&f10_mutex);
            
            v->has_exited = true;
            
        } else {
            // Regular vehicle processing
            pthread_mutex_lock(current_mutex);
            
            controller->queue.push_back(*v);
            
            bool is_ns = (v->current_side == "NORTH" || v->current_side == "SOUTH");
            pthread_cond_t* wait_cond = is_ns ? ns_cond : ew_cond;
            
            if (controller->light_state != "GREEN") {
                pthread_mutex_unlock(current_mutex);
                safePrintWithTime("[WAITING] Vehicle " + to_string(v->id) + " (" + v->type + ") waiting at " + v->current_intersection + " " + v->current_side + " (light is " + controller->light_state + ")");
                pthread_mutex_lock(current_mutex);
            }
            
            while (controller->light_state != "GREEN" && !shutdown_flag && !emergency_active) {
                pthread_cond_wait(wait_cond, current_mutex);
            }
            
            if (shutdown_flag) {
                pthread_mutex_unlock(current_mutex);
                break;
            }
            
            while (emergency_active && !shutdown_flag) {
                pthread_cond_wait(wait_cond, current_mutex);
            }
            
            if (shutdown_flag) {
                pthread_mutex_unlock(current_mutex);
                break;
            }
            
            for (auto it = controller->queue.begin(); it != controller->queue.end(); ++it) {
                if (it->id == v->id) {
                    controller->queue.erase(it);
                    break;
                }
            }
            
            pthread_mutex_unlock(current_mutex);
            
            logVehicleEntry(v->id, v->type, v->current_intersection, v->current_side);
            usleep(CROSSING_TIME);
            
            string exit_side = getExitSide(v->current_side, v->direction);
            
            // Parking handling
            if (v->wants_parking && !v->has_exited) {
                bool parked = tryPark(*current_parking, *v);
                if (parked) {
                    logParking(v->id, v->type, v->current_intersection, true);
                    
                    int park_time = PARKING_MIN_TIME + rand() % (PARKING_MAX_TIME - PARKING_MIN_TIME);
                    usleep(park_time);
                    
                    exitParking(*current_parking, *v);
                    logParking(v->id, v->type, v->current_intersection, false);
                } else {
                    if (tryJoinWaitQueue(*current_parking, *v)) {
                        usleep(500000);
                        tryPark(*current_parking, *v);
                    }
                }
            }
            
            logVehicleExit(v->id, v->type, v->current_intersection, exit_side);
            
            if (willTransitionToOtherIntersection(v->current_intersection, exit_side)) {
                string next_int = (v->current_intersection == "F10") ? "F11" : "F10";
                logVehicleTransit(v->id, v->type, v->current_intersection, next_int);
                
                v->current_intersection = next_int;
                v->current_side = (next_int == "F11") ? "WEST" : "EAST";
            } else {
                v->has_exited = true;
            }
        }
    }
    
    if (v->has_exited) {
        logVehicleComplete(v->id, v->type);
        
        pthread_mutex_lock(&stats_mutex);
        vehicles_completed++;
        pthread_mutex_unlock(&stats_mutex);
    }
    
    delete v;
    return NULL;
}

void f10ControllerProcess() {
    signal(SIGTERM, childSignalHandler);
    signal(SIGINT, childSignalHandler);
    
    safePrintWithTime("[CONTROLLER] F10 Controller Process started (PID: " + to_string(getpid()) + ")");
    
    char msg;
    int cycle = 0;
    
    while (!child_shutdown_flag) {
        ssize_t bytes_read = read(pipe_f11_to_f10[0], &msg, 1);
        if (bytes_read > 0) {
            if (msg == MSG_SHUTDOWN) {
                break;
            }
            if (msg == MSG_EMERGENCY_EASTBOUND || msg == MSG_EMERGENCY_WESTBOUND) {
                safePrintWithTime("[PIPE] F10 received emergency message");
                usleep(100000);
                continue;
            }
        }
        
        if (child_shutdown_flag) break;
        safePrintWithTime("[LIGHT] F10: NORTH-SOUTH -> GREEN (cycle " + to_string(++cycle) + ")");
        write(pipe_f10_to_parent[1], "F10_NS_G", 8);
        usleep(GREEN_DURATION);
        
        if (child_shutdown_flag) break;
        safePrintWithTime("[LIGHT] F10: NORTH-SOUTH -> YELLOW");
        write(pipe_f10_to_parent[1], "F10_NS_Y", 8);
        usleep(YELLOW_DURATION);
        
        if (child_shutdown_flag) break;
        safePrintWithTime("[LIGHT] F10: NORTH-SOUTH -> RED");
        write(pipe_f10_to_parent[1], "F10_NS_R", 8);
        
        if (child_shutdown_flag) break;
        safePrintWithTime("[LIGHT] F10: EAST-WEST -> GREEN");
        write(pipe_f10_to_parent[1], "F10_EW_G", 8);
        usleep(GREEN_DURATION);
        
        if (child_shutdown_flag) break;
        safePrintWithTime("[LIGHT] F10: EAST-WEST -> YELLOW");
        write(pipe_f10_to_parent[1], "F10_EW_Y", 8);
        usleep(YELLOW_DURATION);
        
        if (child_shutdown_flag) break;
        safePrintWithTime("[LIGHT] F10: EAST-WEST -> RED");
        write(pipe_f10_to_parent[1], "F10_EW_R", 8);
    }
    
    safePrintWithTime("[CONTROLLER] F10 Controller Process shutting down");
}

void f11ControllerProcess() {
    signal(SIGTERM, childSignalHandler);
    signal(SIGINT, childSignalHandler);
    
    safePrintWithTime("[CONTROLLER] F11 Controller Process started (PID: " + to_string(getpid()) + ")");
    
    char msg;
    int cycle = 0;
    
    while (!child_shutdown_flag) {
        ssize_t bytes_read = read(pipe_f10_to_f11[0], &msg, 1);
        if (bytes_read > 0) {
            if (msg == MSG_SHUTDOWN) {
                break;
            }
            if (msg == MSG_EMERGENCY_EASTBOUND || msg == MSG_EMERGENCY_WESTBOUND) {
                safePrintWithTime("[PIPE] F11 received emergency message");
                usleep(100000);
                continue;
            }
        }
        
        if (child_shutdown_flag) break;
        safePrintWithTime("[LIGHT] F11: NORTH-SOUTH -> GREEN (cycle " + to_string(++cycle) + ")");
        write(pipe_f11_to_parent[1], "F11_NS_G", 8);
        usleep(GREEN_DURATION);
        
        if (child_shutdown_flag) break;
        safePrintWithTime("[LIGHT] F11: NORTH-SOUTH -> YELLOW");
        write(pipe_f11_to_parent[1], "F11_NS_Y", 8);
        usleep(YELLOW_DURATION);
        
        if (child_shutdown_flag) break;
        safePrintWithTime("[LIGHT] F11: NORTH-SOUTH -> RED");
        write(pipe_f11_to_parent[1], "F11_NS_R", 8);
        
        if (child_shutdown_flag) break;
        safePrintWithTime("[LIGHT] F11: EAST-WEST -> GREEN");
        write(pipe_f11_to_parent[1], "F11_EW_G", 8);
        usleep(GREEN_DURATION);
        
        if (child_shutdown_flag) break;
        safePrintWithTime("[LIGHT] F11: EAST-WEST -> YELLOW");
        write(pipe_f11_to_parent[1], "F11_EW_Y", 8);
        usleep(YELLOW_DURATION);
        
        if (child_shutdown_flag) break;
        safePrintWithTime("[LIGHT] F11: EAST-WEST -> RED");
        write(pipe_f11_to_parent[1], "F11_EW_R", 8);
    }
    
    safePrintWithTime("[CONTROLLER] F11 Controller Process shutting down");
}

void* lightStateListenerThread(void* arg) {
    char buffer[16];
    fd_set read_fds;
    struct timeval timeout;
    
    while (!shutdown_flag) {
        FD_ZERO(&read_fds);
        FD_SET(pipe_f10_to_parent[0], &read_fds);
        FD_SET(pipe_f11_to_parent[0], &read_fds);
        
        int max_fd = (pipe_f10_to_parent[0] > pipe_f11_to_parent[0]) ? pipe_f10_to_parent[0] : pipe_f11_to_parent[0];
        
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000;
        
        int ready = select(max_fd + 1, &read_fds, NULL, NULL, &timeout);
        
        if (ready > 0) {
            if (FD_ISSET(pipe_f10_to_parent[0], &read_fds)) {
                ssize_t n = read(pipe_f10_to_parent[0], buffer, 8);
                if (n == 8) {
                    buffer[n] = '\0';
                    string msg(buffer);
                    
                    pthread_mutex_lock(&f10_mutex);
                    if (msg == "F10_NS_G") {
                        intersection_f10.north_controller.light_state = "GREEN";
                        intersection_f10.south_controller.light_state = "GREEN";
                        intersection_f10.east_controller.light_state = "RED";
                        intersection_f10.west_controller.light_state = "RED";
                        pthread_cond_broadcast(&f10_north_south_cond);
                    } else if (msg == "F10_NS_Y") {
                        intersection_f10.north_controller.light_state = "YELLOW";
                        intersection_f10.south_controller.light_state = "YELLOW";
                    } else if (msg == "F10_NS_R") {
                        intersection_f10.north_controller.light_state = "RED";
                        intersection_f10.south_controller.light_state = "RED";
                    } else if (msg == "F10_EW_G") {
                        intersection_f10.east_controller.light_state = "GREEN";
                        intersection_f10.west_controller.light_state = "GREEN";
                        intersection_f10.north_controller.light_state = "RED";
                        intersection_f10.south_controller.light_state = "RED";
                        pthread_cond_broadcast(&f10_east_west_cond);
                    } else if (msg == "F10_EW_Y") {
                        intersection_f10.east_controller.light_state = "YELLOW";
                        intersection_f10.west_controller.light_state = "YELLOW";
                    } else if (msg == "F10_EW_R") {
                        intersection_f10.east_controller.light_state = "RED";
                        intersection_f10.west_controller.light_state = "RED";
                    }
                    pthread_mutex_unlock(&f10_mutex);
                }
            }
            
            if (FD_ISSET(pipe_f11_to_parent[0], &read_fds)) {
                ssize_t n = read(pipe_f11_to_parent[0], buffer, 8);
                if (n == 8) {
                    buffer[n] = '\0';
                    string msg(buffer);
                    
                    pthread_mutex_lock(&f11_mutex);
                    if (msg == "F11_NS_G") {
                        intersection_f11.north_controller.light_state = "GREEN";
                        intersection_f11.south_controller.light_state = "GREEN";
                        intersection_f11.east_controller.light_state = "RED";
                        intersection_f11.west_controller.light_state = "RED";
                        pthread_cond_broadcast(&f11_north_south_cond);
                    } else if (msg == "F11_NS_Y") {
                        intersection_f11.north_controller.light_state = "YELLOW";
                        intersection_f11.south_controller.light_state = "YELLOW";
                    } else if (msg == "F11_NS_R") {
                        intersection_f11.north_controller.light_state = "RED";
                        intersection_f11.south_controller.light_state = "RED";
                    } else if (msg == "F11_EW_G") {
                        intersection_f11.east_controller.light_state = "GREEN";
                        intersection_f11.west_controller.light_state = "GREEN";
                        intersection_f11.north_controller.light_state = "RED";
                        intersection_f11.south_controller.light_state = "RED";
                        pthread_cond_broadcast(&f11_east_west_cond);
                    } else if (msg == "F11_EW_Y") {
                        intersection_f11.east_controller.light_state = "YELLOW";
                        intersection_f11.west_controller.light_state = "YELLOW";
                    } else if (msg == "F11_EW_R") {
                        intersection_f11.east_controller.light_state = "RED";
                        intersection_f11.west_controller.light_state = "RED";
                    }
                    pthread_mutex_unlock(&f11_mutex);
                }
            }
        }
    }
    
    return NULL;
}

void* vehicleSpawnerThread(void* arg) {
    int spawned = 0;
    
    while (spawned < total_vehicles_to_spawn && !shutdown_flag) {
        Vehicle* v = new Vehicle();
        
        pthread_mutex_lock(&vehicle_mutex);
        v->id = next_vehicle_id++;
        pthread_mutex_unlock(&vehicle_mutex);
        
        bool is_emergency = (rand() % 10 == 0);
        
        if (is_emergency) {
            v->type = (rand() % 2 == 0) ? "Ambulance" : "Firetruck";
            v->priority = "HIGH";
            v->direction = "STRAIGHT";
            
            if (rand() % 2 == 0) {
                v->spawn_intersection = "F10";
                v->spawn_side = "WEST";
            } else {
                v->spawn_intersection = "F11";
                v->spawn_side = "EAST";
            }
            v->wants_parking = false;
        } else {
            v->type = getRandomVehicleType();
            v->priority = (v->type == "Bus") ? "MEDIUM" : "LOW";
            
            v->spawn_intersection = (rand() % 2 == 0) ? "F10" : "F11";
            
            int side = rand() % 4;
            switch (side) {
                case 0: v->spawn_side = "NORTH"; break;
                case 1: v->spawn_side = "SOUTH"; break;
                case 2: v->spawn_side = "EAST"; break;
                default: v->spawn_side = "WEST"; break;
            }
            
            int dir = rand() % 3;
            switch (dir) {
                case 0: v->direction = "LEFT"; break;
                case 1: v->direction = "RIGHT"; break;
                default: v->direction = "STRAIGHT"; break;
            }
            
            v->wants_parking = (rand() % 10 < 3);
        }
        
        v->current_intersection = v->spawn_intersection;
        v->current_side = v->spawn_side;
        v->has_exited = false;
        
        pthread_t tid;
        if (pthread_create(&tid, NULL, vehicleThread, (void*)v) == 0) {
            vehicle_threads.push_back(tid);
            spawned++;
        } else {
            delete v;
            safePrintWithTime("ERROR: Failed to create vehicle thread");
        }
        
        int delay = SPAWN_MIN_DELAY + rand() % (SPAWN_MAX_DELAY - SPAWN_MIN_DELAY);
        usleep(delay);
    }
    
    safePrintWithTime("SPAWNER: All vehicles spawned");
    return NULL;
}

void initializeIntersections() {
    intersection_f10.id = "F10";
    intersection_f10.north_controller.side = "NORTH";
    intersection_f10.north_controller.light_state = "RED";
    intersection_f10.south_controller.side = "SOUTH";
    intersection_f10.south_controller.light_state = "RED";
    intersection_f10.east_controller.side = "EAST";
    intersection_f10.east_controller.light_state = "RED";
    intersection_f10.west_controller.side = "WEST";
    intersection_f10.west_controller.light_state = "RED";
    intersection_f10.emergency_mode = false;
    
    intersection_f11.id = "F11";
    intersection_f11.north_controller.side = "NORTH";
    intersection_f11.north_controller.light_state = "RED";
    intersection_f11.south_controller.side = "SOUTH";
    intersection_f11.south_controller.light_state = "RED";
    intersection_f11.east_controller.side = "EAST";
    intersection_f11.east_controller.light_state = "RED";
    intersection_f11.west_controller.side = "WEST";
    intersection_f11.west_controller.light_state = "RED";
    intersection_f11.emergency_mode = false;
}

void initializeParkingLots() {
    initParkingLot(parking_f10, "F10_Parking");
    initParkingLot(parking_f11, "F11_Parking");
}

void initializePipes() {
    if (pipe(pipe_f10_to_f11) < 0) {
        perror("Failed to create F10->F11 pipe");
        exit(1);
    }
    if (pipe(pipe_f11_to_f10) < 0) {
        perror("Failed to create F11->F10 pipe");
        exit(1);
    }
    if (pipe(pipe_f10_to_parent) < 0) {
        perror("Failed to create F10->Parent pipe");
        exit(1);
    }
    if (pipe(pipe_f11_to_parent) < 0) {
        perror("Failed to create F11->Parent pipe");
        exit(1);
    }
    
    fcntl(pipe_f10_to_f11[0], F_SETFL, O_NONBLOCK);
    fcntl(pipe_f11_to_f10[0], F_SETFL, O_NONBLOCK);
}

void cleanup() {
    close(pipe_f10_to_f11[0]);
    close(pipe_f10_to_f11[1]);
    close(pipe_f11_to_f10[0]);
    close(pipe_f11_to_f10[1]);
    close(pipe_f10_to_parent[0]);
    close(pipe_f10_to_parent[1]);
    close(pipe_f11_to_parent[0]);
    close(pipe_f11_to_parent[1]);
    
    destroyParkingLot(parking_f10);
    destroyParkingLot(parking_f11);
    
    pthread_mutex_destroy(&console_mutex);
    pthread_mutex_destroy(&vehicle_mutex);
    pthread_mutex_destroy(&f10_mutex);
    pthread_mutex_destroy(&f11_mutex);
    pthread_mutex_destroy(&stats_mutex);
    
    pthread_cond_destroy(&f10_north_south_cond);
    pthread_cond_destroy(&f10_east_west_cond);
    pthread_cond_destroy(&f11_north_south_cond);
    pthread_cond_destroy(&f11_east_west_cond);
}

int main(int argc, char* argv[]) {
    srand(time(NULL));
    
    if (argc > 1) {
        total_vehicles_to_spawn = atoi(argv[1]);
        if (total_vehicles_to_spawn <= 0) {
            total_vehicles_to_spawn = DEFAULT_VEHICLE_COUNT;
        }
    }
    
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    displayStartupBanner();
    
    safePrintWithTime("Initializing simulation with " + to_string(total_vehicles_to_spawn) + " vehicles");
    safePrintWithTime("[PARENT] Main process PID: " + to_string(getpid()));
    
    initializeIntersections();
    initializeParkingLots();
    initializePipes();
    
    safePrintWithTime("Initialization complete. Starting simulation...");
    usleep(1000000);
    
    pid_t f10_pid = fork();
    
    if (f10_pid < 0) {
        perror("Failed to fork F10 controller process");
        exit(1);
    } else if (f10_pid == 0) {
        close(pipe_f10_to_f11[0]);
        close(pipe_f11_to_f10[1]);
        close(pipe_f10_to_parent[0]);
        close(pipe_f11_to_parent[0]);
        close(pipe_f11_to_parent[1]);
        f10ControllerProcess();
        exit(0);
    }
    
    safePrintWithTime("[PARENT] Spawned F10 controller process (PID: " + to_string(f10_pid) + ")");
    
    pid_t f11_pid = fork();
    
    if (f11_pid < 0) {
        perror("Failed to fork F11 controller process");
        kill(f10_pid, SIGTERM);
        exit(1);
    } else if (f11_pid == 0) {
        close(pipe_f11_to_f10[0]);
        close(pipe_f10_to_f11[1]);
        close(pipe_f11_to_parent[0]);
        close(pipe_f10_to_parent[0]);
        close(pipe_f10_to_parent[1]);
        f11ControllerProcess();
        exit(0);
    }
    
    safePrintWithTime("[PARENT] Spawned F11 controller process (PID: " + to_string(f11_pid) + ")");
    
    close(pipe_f10_to_parent[1]);
    close(pipe_f11_to_parent[1]);
    
    pthread_t listener_tid;
    pthread_create(&listener_tid, NULL, lightStateListenerThread, NULL);
    
    pthread_t spawner_tid;
    pthread_create(&spawner_tid, NULL, vehicleSpawnerThread, NULL);
    
    pthread_join(spawner_tid, NULL);
    
    safePrintWithTime("Waiting for all vehicles to complete...");
    
    int timeout_counter = 0;
    int max_timeout = 60;
    
    while (vehicles_completed < total_vehicles_to_spawn && timeout_counter < max_timeout && !shutdown_flag) {
        usleep(1000000);
        timeout_counter++;
    }
    
    shutdown_flag = true;
    
    pthread_cond_broadcast(&f10_north_south_cond);
    pthread_cond_broadcast(&f10_east_west_cond);
    pthread_cond_broadcast(&f11_north_south_cond);
    pthread_cond_broadcast(&f11_east_west_cond);
    
    safePrintWithTime("[PARENT] Terminating controller processes...");
    kill(f10_pid, SIGTERM);
    kill(f11_pid, SIGTERM);
    
    int status;
    waitpid(f10_pid, &status, 0);
    safePrintWithTime("[PARENT] F10 controller process terminated");
    waitpid(f11_pid, &status, 0);
    safePrintWithTime("[PARENT] F11 controller process terminated");
    
    pthread_join(listener_tid, NULL);
    
    for (size_t i = 0; i < vehicle_threads.size(); i++) {
        pthread_join(vehicle_threads[i], NULL);
    }
    
    displayShutdownBanner();
    
    safePrintWithTime("Final Statistics:");
    cout << ("  Vehicles Completed: " + to_string(vehicles_completed) + "/" + to_string(total_vehicles_to_spawn) + "\n");
    cout << ("  F10 Parking Final: " + to_string(parking_f10.parked_vehicles.size()) + " parked\n");
    cout << ("  F11 Parking Final: " + to_string(parking_f11.parked_vehicles.size()) + " parked\n");
    
    cleanup();
    
    safePrintWithTime("Simulation ended successfully.");
    
    return 0;
}

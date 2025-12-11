#ifndef PARKINGLOT_H
#define PARKINGLOT_H

#include <iostream>
#include <vector>
#include <string>
#include <semaphore.h>
#include "vehicle.h"

using namespace std;

const int MAX_PARKING_SPOTS = 10;
const int MAX_WAITING_QUEUE = 5;

struct ParkingLot {
    string intersection_id;
    sem_t parking_spots;
    sem_t waiting_queue;
    vector<Vehicle> parked_vehicles;
    vector<Vehicle> waiting_vehicles;
    sem_t access_lock;
    
    ParkingLot(string id) {
        intersection_id = id;
        sem_init(&parking_spots, 0, MAX_PARKING_SPOTS);
        sem_init(&waiting_queue, 0, MAX_WAITING_QUEUE);
        sem_init(&access_lock, 0, 1);
    }
    
    ParkingLot() : intersection_id("F10") {
        sem_init(&parking_spots, 0, MAX_PARKING_SPOTS);
        sem_init(&waiting_queue, 0, MAX_WAITING_QUEUE);
        sem_init(&access_lock, 0, 1);
    }
    
    ~ParkingLot() {
        sem_destroy(&parking_spots);
        sem_destroy(&waiting_queue);
        sem_destroy(&access_lock);
    }
};

inline void initParkingLot(ParkingLot& lot, string id) {
    lot.intersection_id = id;
    sem_init(&lot.parking_spots, 0, MAX_PARKING_SPOTS);
    sem_init(&lot.waiting_queue, 0, MAX_WAITING_QUEUE);
    sem_init(&lot.access_lock, 0, 1);
}

inline void destroyParkingLot(ParkingLot& lot) {
    sem_destroy(&lot.parking_spots);
    sem_destroy(&lot.waiting_queue);
    sem_destroy(&lot.access_lock);
}

inline int getAvailableSpots(ParkingLot& lot) {
    int value;
    sem_getvalue(&lot.parking_spots, &value);
    return value;
}

inline int getAvailableWaitSlots(ParkingLot& lot) {
    int value;
    sem_getvalue(&lot.waiting_queue, &value);
    return value;
}

inline bool tryPark(ParkingLot& lot, Vehicle v) {
    if (isEmergencyVehicle(v.type)) {
        return false;
    }
    
    if (sem_trywait(&lot.parking_spots) == 0) {
        sem_wait(&lot.access_lock);
        lot.parked_vehicles.push_back(v);
        sem_post(&lot.access_lock);
        return true;
    }
    return false;
}

inline bool enterParking(ParkingLot& lot, Vehicle v) {
    if (isEmergencyVehicle(v.type)) {
        return false;
    }
    
    sem_wait(&lot.parking_spots);
    
    sem_wait(&lot.access_lock);
    lot.parked_vehicles.push_back(v);
    sem_post(&lot.access_lock);
    return true;
}

inline bool exitParking(ParkingLot& lot, int vehicle_id) {
    sem_wait(&lot.access_lock);
    
    for (size_t i = 0; i < lot.parked_vehicles.size(); i++) {
        if (lot.parked_vehicles[i].id == vehicle_id) {
            lot.parked_vehicles.erase(lot.parked_vehicles.begin() + i);
            sem_post(&lot.access_lock);
            sem_post(&lot.parking_spots);
            return true;
        }
    }
    
    sem_post(&lot.access_lock);
    return false;
}

inline bool exitParking(ParkingLot& lot, Vehicle& v) {
    return exitParking(lot, v.id);
}

inline bool tryJoinWaitQueue(ParkingLot& lot, Vehicle v) {
    if (isEmergencyVehicle(v.type)) {
        return false;
    }
    
    if (sem_trywait(&lot.waiting_queue) == 0) {
        sem_wait(&lot.access_lock);
        lot.waiting_vehicles.push_back(v);
        sem_post(&lot.access_lock);
        return true;
    }
    return false;
}

inline bool leaveWaitQueue(ParkingLot& lot, int vehicle_id) {
    sem_wait(&lot.access_lock);
    
    for (size_t i = 0; i < lot.waiting_vehicles.size(); i++) {
        if (lot.waiting_vehicles[i].id == vehicle_id) {
            lot.waiting_vehicles.erase(lot.waiting_vehicles.begin() + i);
            sem_post(&lot.access_lock);
            sem_post(&lot.waiting_queue);
            return true;
        }
    }
    
    sem_post(&lot.access_lock);
    return false;
}

inline void printParkingLot(ParkingLot& lot) {
    sem_wait(&lot.access_lock);
    
    cout << ("========================================\n");
    cout << ("PARKING LOT: " + lot.intersection_id + "\n");
    cout << ("========================================\n");
    cout << ("Capacity: " + to_string(MAX_PARKING_SPOTS) + " spots\n");
    cout << ("Available: " + to_string(getAvailableSpots(lot)) + " spots\n");
    cout << ("Occupied: " + to_string(lot.parked_vehicles.size()) + " vehicles\n");
    cout << ("----------------------------------------\n");
    
    if (!lot.parked_vehicles.empty()) {
        cout << ("Parked Vehicles:\n");
        for (size_t i = 0; i < lot.parked_vehicles.size(); i++) {
            cout << ("  " + to_string(i + 1) + ". " + lot.parked_vehicles[i].type 
                 + " (ID: " + to_string(lot.parked_vehicles[i].id) + ")\n");
        }
    } else {
        cout << ("Parked Vehicles: None\n");
    }
    
    cout << ("----------------------------------------\n");
    cout << ("Wait Queue: " + to_string(lot.waiting_vehicles.size()) + "/" + to_string(MAX_WAITING_QUEUE) + "\n");
    
    if (!lot.waiting_vehicles.empty()) {
        cout << ("Waiting Vehicles:\n");
        for (size_t i = 0; i < lot.waiting_vehicles.size(); i++) {
            cout << ("  " + to_string(i + 1) + ". " + lot.waiting_vehicles[i].type 
                 + " (ID: " + to_string(lot.waiting_vehicles[i].id) + ")\n");
        }
    } else {
        cout << ("Waiting Vehicles: None\n");
    }
    
    cout << ("========================================\n");
    
    sem_post(&lot.access_lock);
}

inline void printParkingEntry(int vehicle_id, string vehicle_type, string intersection_id) {
    cout << ("[PARKING] Vehicle " + to_string(vehicle_id) + " (" + vehicle_type 
         + ") entered parking at " + intersection_id + "\n");
}

inline void printParkingExit(int vehicle_id, string vehicle_type, string intersection_id) {
    cout << ("[PARKING] Vehicle " + to_string(vehicle_id) + " (" + vehicle_type 
         + ") exited parking at " + intersection_id + "\n");
}

#endif // PARKINGLOT_H

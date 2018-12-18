/*
Code written by Urban Eriksson, urbane@kth.se

The implementation follows the HIWIN manual:

HIWIN CoE Drive User Guide V1.1.pdf

which can be found in the documents folder of the repository.
*/

#pragma once
#include "asp_servo_api/servo.h"
#include <tinyxml2.h>
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <atomic>
#include <thread>
#include <future>

namespace asp {

    // The states of the EtherCat state machine, see 2.3
    enum EthercatStates {
        Init, 
        PreOperational, 
        SafeOperational, 
        Operational
    };

    class ServoCollection {

        public:

        ServoCollection(std::string fullfilename);
        ~ServoCollection();
        friend std::ostream& operator<<(std::ostream& strm, const ServoCollection& sc); 

        // API methods
        bool connect();
        bool disconnect();
        bool require_ecat_state(EthercatStates newstate);
        bool require_servo_state(ServoStates newstate);
        void set_verbose(bool on) {logging_enabled_ = on;};

        // Read actual values from the servos
        uint16_t read_UINT16(std::string servo_name, std::string entity_name) {return servos_[servo_name]->read_UINT16(entity_name);};
        int16_t read_INT16(std::string servo_name, std::string entity_name) {return servos_[servo_name]->read_INT16(entity_name);};
        int read_INT32(std::string servo_name, std::string entity_name) {return servos_[servo_name]->read_INT32(entity_name);};

        // Write target values to servos
        void write(std::string servo_name, std::string entity_name, uint16_t value) {servos_[servo_name]->write(entity_name,value);};
        void write(std::string servo_name, std::string entity_name, int16_t value) {servos_[servo_name]->write(entity_name,value);};
        void write(std::string servo_name, std::string entity_name, int value) {servos_[servo_name]->write(entity_name,value);};

        // Need not be used except for debug purposes
        Servo* get_servo(std::string name){return servos_[name];};
        void set_servo(std::string name, int value) {};

        private:

        // The async loop outputting data to the servos
        void ethercat_loop();

        std::map<std::string,Servo*> servos_;
        std::map<int,Servo*> servo_by_pos_;
        std::string ethernetport_;
        std::atomic<int> cycletime_ms_;
        EthercatStates currentstate_;
        char IOmap_[2048];
        std::atomic<bool> PDO_input_enable_{false};
        std::atomic<bool> PDO_output_enable_{false};
        std::atomic<bool> is_connected_{false};
        std::future<void> async_future_;
        bool has_done_SDO_and_PDO_configuration_{false};
        std::atomic<bool> logging_enabled_{false};

    };
}


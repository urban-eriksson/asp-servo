/*
Code written by Urban Eriksson, urbane@kth.se

The implementation follows the HIWIN manual:

HIWIN CoE Drive User Guide V1.1.pdf

which can be found in the documents folder of the repository.
*/

#include "asp_servo_api/asp_servo.h"
#include "asp_servo_api/servo.h"
#include <ethercat.h>
#include <tinyxml2.h>
#include <stdexcept>
#include <sys/time.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <string>
#include <vector>

namespace asp {


    // Constructor - read settings, create the servo objects
    ServoCollection::ServoCollection(std::string fullfilename) {

        tinyxml2::XMLDocument doc;
        const char * FULLFILENAME = fullfilename.c_str();
        tinyxml2::XMLError eResult = doc.LoadFile(FULLFILENAME);
        std::cout << "XML read result (0=no error): " << eResult << std::endl;

        tinyxml2::XMLElement *pRoot = doc.FirstChildElement("SystemConfiguration");

        tinyxml2::XMLElement *pEth = pRoot->FirstChildElement("EthernetPort");
        ethernetport_ = pEth->GetText();

        tinyxml2::XMLElement *pCyc = pRoot->FirstChildElement("CycleTime_ms");
        cycletime_ms_ = std::stoi(pCyc->GetText());


        tinyxml2::XMLElement *pServoCollection = pRoot->FirstChildElement("ServoCollection");
        tinyxml2::XMLElement *pServo = pServoCollection->FirstChildElement();
        while (pServo != NULL) {
            Servo* s = Servo::deserializeXML(pServo);
            servos_.insert(std::pair<std::string,Servo*>(s->get_name(),s));
            servo_by_pos_.insert(std::pair<int,Servo*>(s->get_position(),s));
            pServo = pServo->NextSiblingElement();
        }

        currentstate_ = EthercatStates::Init;

        std::cout << "Initialized ServoCollection from " << FULLFILENAME << std::endl;
    }

    // Destructor
    ServoCollection::~ServoCollection() {
        for (auto& kvp:servos_) {
            delete(kvp.second);
        }
    }

    // Setup communication, start cyclic async loop
    bool ServoCollection::connect() {

        std::vector<char> ethernetportvec(ethernetport_.c_str(), ethernetport_.c_str() + ethernetport_.size() + 1u);
        char* ethernetportchar = &ethernetportvec[0];

        if (ec_init(ethernetportchar)){
            std::cout << "Ethernetport " << ethernetport_ << " initialized!" << std::endl;
            
            if ( ec_config_init(FALSE) > 0 ) {
                std::cout << ec_slavecount << " slave(s) found and configured." << std::endl;
            }
            else {
                std::runtime_error("No slaves were found.");
            }
        }

        is_connected_ = true;

        // Start async thread
        async_future_ = std::async(std::launch::async, &ServoCollection::ethercat_loop, this);

        return true;
    }

    // Stop communication, exit async thread, and close socket
    bool ServoCollection::disconnect() {
        require_ecat_state(EthercatStates::PreOperational);
        is_connected_ = false;
        async_future_.get();
        ec_close();        
    }

    // Move from current state to the required state. see Fig 2-2
    // This may include going through other states
    bool ServoCollection::require_ecat_state(EthercatStates requiredstate) {

        if (requiredstate == currentstate_) {
            return true;
        }

        // Transistion matrix gives the next state given the current state and the desired state
        std::vector<std::vector<EthercatStates>> transition_matrix {
            {EthercatStates::Init,           EthercatStates::PreOperational, EthercatStates::PreOperational, EthercatStates::PreOperational},
            {EthercatStates::Init,           EthercatStates::PreOperational, EthercatStates::SafeOperational,EthercatStates::SafeOperational},
            {EthercatStates::PreOperational, EthercatStates::PreOperational, EthercatStates::SafeOperational,EthercatStates::Operational},
            {EthercatStates::SafeOperational,EthercatStates::SafeOperational,EthercatStates::SafeOperational,EthercatStates::Operational}
        };

        EthercatStates nextstate = transition_matrix[currentstate_][requiredstate];

        if (currentstate_ == EthercatStates::Init && nextstate == EthercatStates::PreOperational) {
            std::cout << "Changing EtherCat state from Init to Pre-Op" << std::endl;
            ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE);
        }
        else if (currentstate_ == EthercatStates::PreOperational && nextstate == EthercatStates::SafeOperational) {
            std::cout << "Changing EtherCat state from Pre-Op to Safe-Op" << std::endl;

            if (!has_done_SDO_and_PDO_configuration_) {

                std::cout << "SDO setup:" << std::endl;

                // Send objects and values to slaves according to StartupParameters in the XML file
                for (auto& kvp: servos_) {
                    Servo * pservo = kvp.second;
                    pservo->initialize_SDO_settings();
                }

                // Setup PDO according to the PDOmapping in the XML file
                for (auto& kvp : servos_) {
                    Servo* pservo = kvp.second;
                    pservo->do_PDO_mapping();
                    pservo->SDOread_INT32(1,0x1C32,2);
                }

                ec_config_map(&IOmap_);
                ec_configdc();
                has_done_SDO_and_PDO_configuration_ = true;

            }

            ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);
            PDO_input_enable_ = true;

        }
        else if (currentstate_ == EthercatStates::SafeOperational && nextstate == EthercatStates::Operational){
            std::cout << "Changing EtherCat state from Safe-Op to Operational" << std::endl;
            for (int slave_index = 1; slave_index <= ec_slavecount; slave_index++) {
               ec_slave[slave_index].state = EC_STATE_OPERATIONAL;
               ec_writestate(slave_index);
            }
            int state_res = ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
            std::cout << "Statecheck returned (8=operational): " << state_res << std::endl;
            int64_t sync0time_ns = cycletime_ms_ * 1000;
            ec_dcsync0(1, TRUE, sync0time_ns, 0); // SYNC0 on slave 1
            if (ec_slave[0].state == EC_STATE_OPERATIONAL )
            {
               std::cout << "Operational state reached for all slaves." << std::endl;
               PDO_output_enable_ = true;
            }
            else {
                std::runtime_error("All slaves could not reach operational state");
            }
        }
        else if (currentstate_ == EthercatStates::Operational && nextstate == EthercatStates::SafeOperational) {
            std::cout << "Changing EtherCat state from Operational to Safe-Op" << std::endl;
            PDO_output_enable_ = false;  
            int64_t sync0time_ns = cycletime_ms_ * 1000;
            ec_dcsync0(1, FALSE, sync0time_ns, 0); // SYNC0 off  
            for (int slave_index = 1; slave_index <= ec_slavecount; slave_index++) {
               ec_slave[slave_index].state = EC_STATE_SAFE_OP;
               ec_writestate(slave_index);
            }
            ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);
        }
        else if (currentstate_ == EthercatStates::SafeOperational && nextstate == EthercatStates::PreOperational) {
            std::cout << "Changing EtherCat state from Safe-Op to Pre-Op" << std::endl;
            PDO_input_enable_ = false;            
            ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE);
        }
        else if (currentstate_ == EthercatStates::PreOperational && nextstate == EthercatStates::Init) {
            std::cout << "Changing EtherCat state from Pre-Op to Init" << std::endl;
            ec_statecheck(0, EC_STATE_INIT, EC_TIMEOUTSTATE);
        } 

        currentstate_ = nextstate;

        return require_ecat_state(requiredstate);
    }

    bool ServoCollection::require_servo_state(ServoStates requiredstate) {

        std::map<std::string,ServoStates> intermediatestates;
        for (auto& kvp:servos_) {
            ServoStates currentstate = kvp.second->get_servo_state();
            ServoStates nextstate = kvp.second->require_servo_state(currentstate, requiredstate);
            intermediatestates.insert(std::pair<std::string,ServoStates>(kvp.first,nextstate));
        }

        int maxiter = 40;
        int iter = 0;
        bool aok;
        while (iter < maxiter) {
            aok = true;
            for (auto& kvp:servos_) {
                Servo* s = kvp.second;
                ServoStates currentstate = s->get_servo_state();
                if (currentstate != requiredstate) {
                    aok = false;
                    if (currentstate == intermediatestates[kvp.first]) {
                       ServoStates nextstate = s->require_servo_state(currentstate, requiredstate);
                       intermediatestates[kvp.first] = nextstate;
                    }
                }
            }
            if (aok) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            iter++;
        }

        if (!aok) {
            std::runtime_error("Not all servos reached the required state in 2s");
        }

        return true;

    }


    // Adding the cycle time to the time spec
    #define NSEC_PER_SEC 1000000000
    void add_timespec(struct timespec *ts, int64 addtime)
    {
       int64 sec, nsec;

       nsec = addtime % NSEC_PER_SEC;
       sec = (addtime - nsec) / NSEC_PER_SEC;
       ts->tv_sec += sec;
       ts->tv_nsec += nsec;
       if ( ts->tv_nsec > NSEC_PER_SEC )
       {
          nsec = ts->tv_nsec % NSEC_PER_SEC;
          ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
          ts->tv_nsec = nsec;
       }
    }

    // PI control to get linux time synced to DC time
    void ec_sync(int64 reftime, int64 cycletime_ns , int64 *offsettime_ns)
    {
       int integral = 0;
       int64 delta;
       /* set linux sync point 50us later than DC sync, just as example */
       //delta = (reftime - 50000) % cycletime_ns;
       // Not sure what to use here. Zero works as well /UE
       delta = (reftime) % cycletime_ns;
       if(delta > (cycletime_ns /2)) { 
           delta = delta - cycletime_ns; 
       }
       else if(delta>0) { 
           integral++; 
       }
       else if(delta<0) { 
           integral--; 
       }
       *offsettime_ns = -(delta / 100) - (integral /20);
    }


    // Asynchrounous loop
    void ServoCollection::ethercat_loop() {

        uint16_t control_word = 0;
        struct timespec tspec;
        struct timespec tspecdummy;
        struct timespec tcheck;
        struct timespec tcheck_old;
        int64_t cycletime_ns = cycletime_ms_ * 1000;
        int64_t offset_ns = 0;
        clock_gettime(CLOCK_MONOTONIC, &tspec);

        while (is_connected_) {

            // Basically adding the cycletime to the timespec
            add_timespec(&tspec, cycletime_ns + offset_ns);
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tspec, &tspecdummy);

            // For logging only
            clock_gettime(CLOCK_MONOTONIC, &tcheck);
            long deltat_us = (tcheck.tv_nsec - tcheck_old.tv_nsec)/1000;
            if (deltat_us < 0) {
                deltat_us += 1000000;
            }
            tcheck_old = tcheck;

            // Recieve data - possible in SAFE_OP and OPERATIONAL
            if (PDO_input_enable_) {
                ec_receive_processdata(EC_TIMEOUTRET);
            }

            // Syncing to DC
            ec_sync(ec_DCtime, cycletime_ns, &offset_ns);

            // Sending data - possible in OPERATIONAL only
            if (PDO_output_enable_) {

                for (auto& kvp : servos_) {  
                    int position = kvp.second->get_position();                  
                    kvp.second->update_IOmap(ec_slave[position].outputs, ec_slave[position].inputs);
                }
                
                ec_send_processdata();

                if (logging_enabled_) {

                    for (int i=1; i<=servo_by_pos_.size(); i++) {
                        std::cout << "#" << i << ":";
                        std::string logstr = servo_by_pos_[i]->read_logstring();
                        std::cout << logstr << " ";
                    }

                    std::cout << "dt:" << deltat_us << "us" << std::endl;
                }
            }
        }
    }

    std::ostream& operator<<(std::ostream& strm, const ServoCollection& sc) {
        strm << "------------------- ServoCollection ---------------------" << std::endl;
        strm << "EthernetPort  : " << sc.ethernetport_ << std::endl;
        strm << "Cycletime (ms): " << sc.cycletime_ms_ << std::endl;
        for (auto& s:sc.servos_) {
            strm << *s.second << std::endl ;
        }
        strm << "---------------------------------------------------------" << std::endl;
        return strm;
    }

}

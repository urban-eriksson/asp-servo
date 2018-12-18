/*
Code written by Urban Eriksson, urbane@kth.se

The implementation follows the HIWIN manual:

HIWIN CoE Drive User Guide V1.1.pdf

which can be found in the documents folder of the repository.
*/

#pragma once
#include <tinyxml2.h>
#include <map>
#include <vector>
#include <string>
#include <mutex>

namespace asp {

    // The different servo states, see Table 3-1
    enum ServoStates {
        NotReadyToSwitchOn,
        SwitchOnDisabled,
        ReadyToSwitchOn,
        SwitchedOn,
        OperationEnabled,
        QuickStopActive,
        FaultReactionActive,
        Fault
    };

    // Command of state transition, see Table 3-3
    enum ServoCommands {
        Shutdown,
        SwitchOn,
        EnableOperation,
        DisableOperation,
        DisableVoltage,
        QuickStop,
        FaultReset
    };

    // Represents a mapping object as defined in chapter 4:
    // name, index, subindex and a data type
    class DeviceObject {
        public:
        DeviceObject() {};
        DeviceObject(std::string name, std::string index, std::string subindex, std::string type): 
          Name(name),Index(index),Subindex(subindex),Type(type) {};
        std::string Name;
        std::string Index;
        std::string Subindex;
        std::string Type;
        static DeviceObject* deserializeXML(tinyxml2::XMLElement *pObj);        
    };

    // Holds an object value. Could probably be simplified with a template class
    class ObjectValue {
        public:
        ObjectValue() : uint8_value(0), int8_value(0), uint16_value(0), int16_value(0), uint32_value(0), int32_value(0) {};
        uint8_t uint8_value;
        int8_t int8_value;
        uint16_t uint16_value;
        int16_t int16_value;
        uint32_t uint32_value;
        int int32_value;
        void set(std::string value_string, std::string value_type);
    };

   // Is used to control and monitor a single physical servo
   // The normal way to create the object is through 
   // the deserializeXML factory method
   class Servo  {
        public:

        Servo() {};
        Servo(std::string name,            
            int position, 
            std::string type,
            std::map<std::string,DeviceObject*> tx_objects, 
            std::map<std::string,DeviceObject*> rx_objects,
            std::vector<DeviceObject*> tx_maporder,
            std::vector<DeviceObject*> rx_maporder,
            std::vector<std::pair<DeviceObject*,ObjectValue*>> init_objects) : 
            name_(name),slave_index_(position), type_(type), 
            tx_objects_(tx_objects), rx_objects_(rx_objects),
            tx_maporder_(tx_maporder), rx_maporder_(rx_maporder),
            init_objects_(init_objects) {};
        ~Servo();

        // Factory method
        static Servo* deserializeXML(tinyxml2::XMLElement *pObj);   

        friend std::ostream& operator<<(std::ostream& strm, const Servo& s);
        std::string get_name() { return name_;};
        int get_position() {return slave_index_;};

        void initialize_SDO_settings();
        void do_PDO_mapping();

        // Read actual values from servo
        uint16_t read_UINT16(std::string entity_name);
        int16_t read_INT16(std::string entity_name);
        int read_INT32(std::string entity_name);

        // Write target values to servo
        void write(std::string entity_name, uint16_t value);
        void write(std::string entity_name, int16_t value);
        void write(std::string entity_name, int value);

        // For logging IOmap content
        std::string read_logstring();

        // Sync IOmap and internal object values
        void update_IOmap(uint8_t* outputs, uint8_t* inputs);

        ServoStates get_servo_state();
        void send_command(ServoCommands command);

        ServoStates require_servo_state(ServoStates currentstate, ServoStates newstate);

        // Communication using service data objects (SDO)
        void SDOwrite_INT8(int slave, uint16_t index, uint8_t subindex, int8_t value);
        void SDOwrite_UINT8(int slave, uint16_t index, uint8_t subindex, uint8_t value);
        void SDOwrite_INT16 (int slave, uint16_t index, uint8_t subindex, int16_t value);
        void SDOwrite_UINT16 (int slave, uint16_t index, uint8_t subindex, uint16_t value);
        void SDOwrite_INT32(int slave, uint16_t index, uint8_t subindex, int32_t value);
        void SDOwrite_UINT32(int slave, uint16_t index, uint8_t subindex, uint32_t value);
        void SDOread_INT32(int slave, uint16_t index, uint8_t subindex);

        private:

        // Internal intermediate variables for the input and output values
        std::map<std::string,ObjectValue*> rx_values_;
        std::map<std::string,ObjectValue*> tx_values_;

        std::string name_;
        int slave_index_; // i.e. the physical position as counted from the master
        std::string type_;
        std::map<std::string,DeviceObject*> rx_objects_;
        std::map<std::string,DeviceObject*> tx_objects_;
        std::vector<DeviceObject*> rx_maporder_;
        std::vector<DeviceObject*> tx_maporder_;
        std::map<std::string,int> rx_offsets_;
        std::map<std::string,int> tx_offsets_;
        std::vector<std::pair<DeviceObject*,ObjectValue*>> init_objects_;
        std::mutex mtx_;
        uint16_t controlword_{0};

    };

}
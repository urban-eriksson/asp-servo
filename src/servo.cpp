/*
Code written by Urban Eriksson, urbane@kth.se

The implementation follows the HIWIN manual:

HIWIN CoE Drive User Guide V1.1.pdf

which can be found in the documents folder of the repository.
*/

#include "asp_servo_api/servo.h"
#include <ethercat.h>
#include <tinyxml2.h>
#include <stdexcept>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <bitset>
#include <chrono>
#include <thread>

namespace asp {

    // Destructor
    Servo::~Servo() {

        for (auto& dobj:tx_maporder_){
            delete(dobj);
        }

        for (auto& dobj:rx_maporder_){
            delete(dobj);
        }

        for (auto& kvp:tx_values_) {
            delete(kvp.second);
        }

        for (auto& kvp:rx_values_) {
            delete(kvp.second);
        }

        for (auto& kvp:init_objects_) {
            delete(kvp.first);
            delete(kvp.second);
        }
    }

    // Factory function for device objects
    // Device objects are defined in the chapter 4 of the HIWIN manual
    // Name, index, subindex and a data type
    DeviceObject* DeviceObject::deserializeXML(tinyxml2::XMLElement *pDeviceObject) {

        tinyxml2::XMLElement *pName = pDeviceObject->FirstChildElement("Name");
        std::string name = pName->GetText();

        tinyxml2::XMLElement *pIndex = pDeviceObject->FirstChildElement("Index");
        std::string index = pIndex->GetText();

        tinyxml2::XMLElement *pSubindex = pDeviceObject->FirstChildElement("Subindex");
        std::string subindex = pSubindex->GetText();

        tinyxml2::XMLElement *pType = pDeviceObject->FirstChildElement("Type");
        std::string type = pType->GetText();

        return new DeviceObject(name, index, subindex, type);

    }

    // Generic container for the value of an object
    // Only one field is used at a time
    void ObjectValue::set(std::string value_string, std::string type_string) {
        if (type_string == "INT32") {
            int32_value = std::stoi(value_string);
        }
        else if (type_string == "UINT32") {
            uint32_value = std::stoul(value_string);
        }
        else if (type_string == "INT16") {
            int16_value = std::stoi(value_string);
        }
        else if (type_string == "UINT16") {
            uint16_value = std::stoul(value_string);
        }
        else if (type_string == "INT8") {
            int8_value = std::stoi(value_string);
        }
        else if (type_string == "UINT8") {
            uint8_value = std::stoul(value_string);
        }
    }


    // Factory function for the Servo class.
    // Deserializes a single Servo from settings in the XML config file
    Servo* Servo::deserializeXML(tinyxml2::XMLElement *pServo) {

        std::string name = pServo->Attribute("name");

        int position = std::stoi(pServo->Attribute("position"));

        tinyxml2::XMLElement *pType = pServo->FirstChildElement("Type");
        std::string type = pType->GetText();

        // Startup parameters
        tinyxml2::XMLElement *pInitialization = pServo->FirstChildElement("StartupParameters");
        tinyxml2::XMLElement *pItem = pInitialization->FirstChildElement("Item");
        std::vector<std::pair<DeviceObject*,ObjectValue*>> init_objects;
        while (pItem != NULL) {
            tinyxml2::XMLElement *pObject = pItem->FirstChildElement("Object");
            DeviceObject* obj = DeviceObject::deserializeXML(pObject);
            tinyxml2::XMLElement *pValue = pItem->FirstChildElement("Value");
            std::string value_string = pValue->GetText();
            ObjectValue* obj_value = new ObjectValue();
            obj_value->set(value_string,obj->Type);
            init_objects.push_back(std::pair<DeviceObject*,ObjectValue*>(obj,obj_value));
            pItem = pItem->NextSiblingElement();
        }

        // PDO mapping

        // Controlword default mapping
        DeviceObject* controlword = new DeviceObject("Controlword","0x6040","0","UINT16");
        std::map<std::string,DeviceObject*> tx_objects;
        tx_objects.insert(std::pair<std::string,DeviceObject*>(controlword->Name,controlword));
        std::vector<DeviceObject*> tx_maporder;
        tx_maporder.push_back(controlword);

        // Additional objects Tx
        tinyxml2::XMLElement *pPDOmapping = pServo->FirstChildElement("PDOmapping");
        tinyxml2::XMLElement *pTx = pPDOmapping->FirstChildElement("Tx");
        tinyxml2::XMLElement *pTxObject = pTx->FirstChildElement("Object");
        while (pTxObject != NULL) {
            DeviceObject* dobj = DeviceObject::deserializeXML(pTxObject);
            tx_objects.insert(std::pair<std::string,DeviceObject*>(dobj->Name,dobj));
            tx_maporder.push_back(dobj);
            pTxObject = pTxObject->NextSiblingElement();
        }

        // Statusword default mapping 
        DeviceObject* statusword = new DeviceObject("Statusword","0x6041","0","UINT16");
        std::map<std::string,DeviceObject*> rx_objects;
        rx_objects.insert(std::pair<std::string,DeviceObject*>(statusword->Name,statusword));
        std::vector<DeviceObject*> rx_maporder;
        rx_maporder.push_back(statusword);

        // Additional objects Rx
        tinyxml2::XMLElement *pRx = pPDOmapping->FirstChildElement("Rx");
        tinyxml2::XMLElement *pRxObject = pRx->FirstChildElement("Object");
        while (pRxObject != NULL) {
            DeviceObject* dobj = DeviceObject::deserializeXML(pRxObject);
            rx_objects.insert(std::pair<std::string,DeviceObject*>(dobj->Name,dobj));
            rx_maporder.push_back(dobj);
            pRxObject = pRxObject->NextSiblingElement();
        }

        Servo* s = new Servo(name, position, type, tx_objects, rx_objects, tx_maporder, rx_maporder, init_objects);

        return s;

    }

    // Outputs the startup parameters to the servo
    // such as e.g. operating mode and resetting the encoder
    void Servo::initialize_SDO_settings() {

        for (auto& kvp:init_objects_) {
            DeviceObject* obj = kvp.first;
            std::size_t* ptr;
            uint16_t index(std::stoul(obj->Index,nullptr,16));
            uint8_t subindex(std::stoul(obj->Subindex));
            ObjectValue* ov = kvp.second;
            if (obj->Type == "INT32") {
                SDOwrite_INT32(slave_index_, index, subindex, ov->int32_value);
            }
            else if (obj->Type == "UINT32") {
                SDOwrite_UINT32(slave_index_, index, subindex, ov->uint32_value);
            }
            else if (obj->Type == "INT16") {
                SDOwrite_INT16(slave_index_, index, subindex, ov->int16_value);
            }
            else if (obj->Type == "UINT16") {
                SDOwrite_UINT16(slave_index_, index, subindex, ov->uint16_value);
            }
            else if (obj->Type == "INT8") {
                SDOwrite_INT8(slave_index_, index, subindex, ov->int8_value);
            }
            else if (obj->Type == "UINT8") {
                SDOwrite_UINT8(slave_index_, index, subindex, ov->uint8_value);
            }
        }
    }

    // Sets up the PDO of the actual servo by sending the specified service data objects
    // Maps named variables (tx and rx values) to positions in the PDO (offsets)
    void Servo::do_PDO_mapping() {
        // Tx
        SDOwrite_UINT8 (slave_index_, 0x1C12, 0, 0);
        SDOwrite_UINT8 (slave_index_, 0x1600, 0, 0);
        int subindextx = 1;
        int offsettx = 0;
        for (DeviceObject* pobj:tx_maporder_) {
            if (pobj->Type == "INT16" || pobj->Type == "UINT16") {
                std::string hexstr = pobj->Index + "0010";
                SDOwrite_UINT32(slave_index_,0x1600,subindextx,std::stoul(hexstr,nullptr,16));
                tx_offsets_.insert(std::pair<std::string,int>(pobj->Name, offsettx));
                offsettx += 2;
            }
            else if (pobj->Type == "INT32" || pobj->Type == "UINT32") {
                std::string hexstr = pobj->Index + "0020";
                SDOwrite_UINT32(slave_index_,0x1600,subindextx,std::stoul(hexstr,nullptr,16));
                tx_offsets_.insert(std::pair<std::string,int>(pobj->Name,offsettx));
                offsettx += 4;
            }
            else {
                throw std::runtime_error("Unsupported type in config file:" + pobj->Type);
            }
            tx_values_.insert(std::pair<std::string,ObjectValue*>(pobj->Name, new ObjectValue()));
            subindextx++;
        }

        SDOwrite_UINT8 (slave_index_, 0x1600, 0, subindextx-1);
        SDOwrite_UINT16(slave_index_, 0x1C12, 1, 0x1600);
        SDOwrite_UINT8 (slave_index_, 0x1C12, 0, 1);

        // Rx
        SDOwrite_UINT8 (slave_index_, 0x1C13, 0, 0);
        SDOwrite_UINT8 (slave_index_, 0x1A00, 0, 0);
        int subindexrx = 1;
        int offsetrx = 0;
        for (DeviceObject* pobj:rx_maporder_) {
            if (pobj->Type == "INT16" || pobj->Type == "UINT16") {
                std::string hexstr = pobj->Index + "0010";
                SDOwrite_UINT32(slave_index_,0x1A00,subindexrx,std::stoul(hexstr,nullptr,16));
                rx_offsets_.insert(std::pair<std::string,int>(pobj->Name,offsetrx));
                offsetrx += 2;

            }
            else if (pobj->Type == "INT32" || pobj->Type == "UINT32") {
                std::string hexstr = pobj->Index + "0020";
                SDOwrite_UINT32(slave_index_,0x1A00,subindexrx,std::stoul(hexstr,nullptr,16));
                rx_offsets_.insert(std::pair<std::string,int>(pobj->Name,offsetrx));
                offsetrx += 4;
            }
            else {
                throw std::runtime_error("Unsupported type in config file:" + pobj->Type);
            }
            rx_values_.insert(std::pair<std::string,ObjectValue*>(pobj->Name,new ObjectValue()));
            subindexrx++;
        }
        SDOwrite_UINT8 (slave_index_, 0x1A00, 0, subindexrx-1);
        SDOwrite_UINT16(slave_index_, 0x1C13, 1, 0x1A00);
        SDOwrite_UINT8 (slave_index_, 0x1C13, 0, 1);        
    }

    // The current status of the servo is obtained from the bits of the status word.
    // See table 3.5 in the HIWIN manual
    ServoStates Servo::get_servo_state() {
        uint16_t statusword = read_UINT16("Statusword");
        std::bitset<16> statusbits(statusword);
        bool bit6 = statusbits[6]; 
        bool bit5 = statusbits[5]; 
        bool bit3 = statusbits[3]; 
        bool bit2 = statusbits[2]; 
        bool bit1 = statusbits[1]; 
        bool bit0 = statusbits[0]; 

        if (!bit6 && !bit3 && !bit2 && !bit1 && !bit0 ) {
            std::cout << "Servo " << slave_index_ << " in state NotReadyToSwitchOn" << std::endl;
            return ServoStates::NotReadyToSwitchOn;
        }
        else if ( bit6 && !bit3 && !bit2 && !bit1 && !bit0 ) {
            std::cout << "Servo " << slave_index_ << " in state SwitchOnDisabled" << std::endl;
            return ServoStates::SwitchOnDisabled;
        }
        else if ( !bit6 && bit5 && !bit3 && !bit2 && !bit1 && bit0 ) {
            std::cout << "Servo " << slave_index_ << " in state ReadyToSwitchOn" << std::endl;
            return ServoStates::ReadyToSwitchOn;
        }
        else if (!bit6 && bit5 && !bit3 && !bit2 && bit1 && bit0  ) {
            std::cout << "Servo " << slave_index_ << " in state SwitchedOn" << std::endl;
            return ServoStates::SwitchedOn;
        }
        else if ( !bit6 && bit5 && !bit3 && bit2 && bit1 && bit0  ) {
            std::cout << "Servo " << slave_index_ << " in state OperationEnabled" << std::endl;
            return ServoStates::OperationEnabled;
        }
        else if ( !bit6 && !bit5 && !bit3 && bit2 && bit1 && bit0  ) {
            std::cout << "Servo " << slave_index_ << " in state QuickStopActive" << std::endl;
            return ServoStates::QuickStopActive;
        }
        else if ( !bit6 && bit3 && bit2 && bit1 && bit0  ) {
            std::cout << "Servo " << slave_index_ << " in state FaultReactionActive" << std::endl;
            return ServoStates::FaultReactionActive;
        }
        else if ( !bit6 && bit3 && !bit2 && !bit1 && !bit0  ) {
            std::cout << "Servo " << slave_index_ << " in state Fault" << std::endl;
            return ServoStates::Fault;
        }
        else {
            std::stringstream stream;
            stream << std::setfill('0') << std::setw(4) << std::hex << statusword;
            std::cout << "Unknown state for servo " + std::to_string(slave_index_) + ": " + stream.str() + " received from statusword" << std::endl;
        }
    }

    // Log string which can be used for debugging
    std::string Servo::read_logstring() {
        std::stringstream stream;
        stream << "Out:|";
        mtx_.lock();
        for (DeviceObject* obj:tx_maporder_) {
            ObjectValue* ov = tx_values_[obj->Name];
            if (obj->Type == "UINT16") {
                stream << std::setfill('0') << std::setw(4) << std::hex << ov->uint16_value;
            }
            else if (obj->Type == "INT16") {
                stream << std::setfill('0') << std::setw(4) << std::hex << ov->int16_value;
            }
            else if (obj->Type == "INT32") {
                stream << std::setfill('0') << std::setw(8) << std::hex << ov->int32_value;
            }
            stream <<  "|";
        }
      
        stream << " In:|";

        for (DeviceObject* obj:rx_maporder_) {
            ObjectValue* ov = rx_values_[obj->Name];
            if (obj->Type == "UINT16") {
                stream << std::setfill('0') << std::setw(4) << std::hex << ov->uint16_value;
            }
            else if (obj->Type == "INT16") {
                stream << std::setfill('0') << std::setw(4) << std::hex << ov->int16_value;
            }
            else if (obj->Type == "INT32") {
                stream << std::setfill('0') << std::setw(8) << std::hex << ov->int32_value;
            }
            stream <<  "|";
        }
        mtx_.unlock();
        return stream.str();
    }

    // Reads the current value of an internal variable
    uint16_t Servo::read_UINT16(std::string entity_name) {
        uint16_t value;
        mtx_.lock();
        if (rx_objects_[entity_name]->Type == "UINT16") {
            value = rx_values_[entity_name]->uint16_value;
        }
        else {
            std::runtime_error("Wrong type when reading " + entity_name);
        }
        mtx_.unlock();
        return value;
    }

    // Reads the current value of a internal variable
    int16_t Servo::read_INT16(std::string entity_name) {
        int16_t value;
        mtx_.lock();
        if (rx_objects_[entity_name]->Type == "INT16") {
            value = rx_values_[entity_name]->int16_value;
        }
        else {
            std::runtime_error("Wrong type when reading " + entity_name);
        }
        mtx_.unlock();
        return value;
    }

    // Reads the current value of a internal variable
    int Servo::read_INT32(std::string entity_name) {
        int value;
        mtx_.lock();
        if (rx_objects_[entity_name]->Type == "INT32") {
            value = rx_values_[entity_name]->int32_value;
        }
        else {
            std::runtime_error("Wrong type when reading " + entity_name);
        }
        mtx_.unlock();
        return value;
    }

    // Tries to change the state of the servo according to the state diagram
    // See Fig 3-1
    // The method is now non-blocking so it will change try to change the state of the servo
    // one step towards the required state. The return value is the intermiediate state which the servo
    // will try to reach. By having it non-blocking several servos can be commanded to change state
    // and when intermediate states are reached the ServoCollection object can call this object once more
    // The solution is efficient when using a single thread, however if multithreading would be
    // used then the structure of the code could be done slightly differently, with each servo
    // handling its own state transitions in a seperate thread. 
    ServoStates Servo::require_servo_state(ServoStates currentstate, ServoStates requiredstate) {

        if (requiredstate == currentstate) {
            return currentstate;
        }

        // Format is transition_matrix[currentstate][requiredstate] gives next state
        // currentstate = row, requiredstate = column
        // When a transition is not valid according to the state diagram, state will remain the same.
        // This will eventually throw an error since requiring such a state should not be done. 
        // The order is: 
        //    NotReadyToSwitchOn,               SwitchOnDisabled,                    ReadyToSwitchOn,                   SwitchedOn,                      OperationEnabled,                 QuickStopActive,                  FaultReactionActive,                Fault
        std::vector<std::vector<ServoStates>> transition_matrix {
            {ServoStates::NotReadyToSwitchOn , ServoStates::SwitchOnDisabled   , ServoStates::SwitchOnDisabled   , ServoStates::SwitchOnDisabled   , ServoStates::SwitchOnDisabled   , ServoStates::NotReadyToSwitchOn , ServoStates::NotReadyToSwitchOn , ServoStates::NotReadyToSwitchOn},
            {ServoStates::SwitchOnDisabled   , ServoStates::SwitchOnDisabled   , ServoStates::ReadyToSwitchOn    , ServoStates::ReadyToSwitchOn    , ServoStates::ReadyToSwitchOn    , ServoStates::SwitchOnDisabled   , ServoStates::SwitchOnDisabled   , ServoStates::SwitchOnDisabled},
            {ServoStates::ReadyToSwitchOn    , ServoStates::SwitchOnDisabled   , ServoStates::ReadyToSwitchOn    , ServoStates::SwitchedOn         , ServoStates::SwitchedOn         , ServoStates::ReadyToSwitchOn    , ServoStates::ReadyToSwitchOn    , ServoStates::ReadyToSwitchOn}, 
            {ServoStates::SwitchedOn         , ServoStates::ReadyToSwitchOn    , ServoStates::ReadyToSwitchOn    , ServoStates::SwitchedOn         , ServoStates::OperationEnabled   , ServoStates::SwitchedOn         , ServoStates::SwitchedOn         , ServoStates::SwitchedOn},
            {ServoStates::OperationEnabled   , ServoStates::SwitchedOn         , ServoStates::SwitchedOn         , ServoStates::SwitchedOn         , ServoStates::OperationEnabled   , ServoStates::QuickStopActive    , ServoStates::OperationEnabled   , ServoStates::OperationEnabled},
            {ServoStates::QuickStopActive    , ServoStates::SwitchOnDisabled   , ServoStates::QuickStopActive    , ServoStates::QuickStopActive    , ServoStates::QuickStopActive    , ServoStates::QuickStopActive    , ServoStates::QuickStopActive    , ServoStates::QuickStopActive},
            {ServoStates::FaultReactionActive, ServoStates::FaultReactionActive, ServoStates::FaultReactionActive, ServoStates::FaultReactionActive, ServoStates::FaultReactionActive, ServoStates::FaultReactionActive, ServoStates::FaultReactionActive, ServoStates::Fault},
            {ServoStates::Fault              , ServoStates::SwitchOnDisabled   , ServoStates::Fault              , ServoStates::Fault              , ServoStates::Fault              , ServoStates::Fault              , ServoStates::Fault              , ServoStates::Fault}
        };

        ServoStates nextstate = transition_matrix[currentstate][requiredstate];

        if (currentstate == ServoStates::NotReadyToSwitchOn && nextstate == ServoStates::SwitchOnDisabled) {
            std::cout << "Waiting for servo " << slave_index_ << " to change state from NotReadyToSwitchOn to SwitchOnDisabled" << std::endl;
            //std::cout << "Sending command Quick stop to servo " << slave_index_ << std::endl;
            //send_command(ServoCommands::QuickStop); // 2
        }
        else if (currentstate == SwitchOnDisabled && nextstate == ServoStates::ReadyToSwitchOn) {
            std::cout << "Changing servo " << slave_index_ << " state from SwitchOnDisabled to ReadyToSwitchOn" << std::endl;
            std::cout << "Sending command Shutdown to servo " << slave_index_ << std::endl;
            send_command(ServoCommands::Shutdown); // 6
        }
        else if (currentstate == ReadyToSwitchOn && nextstate == ServoStates::SwitchedOn) {
            std::cout << "Changing servo " << slave_index_ << " state from ReadyToSwitchOn to SwitchedOn" << std::endl;
            std::cout << "Sending command SwitchOn to servo " << slave_index_ << std::endl;
            send_command(ServoCommands::SwitchOn); // 7
        }
        else if (currentstate == ServoStates::SwitchedOn && nextstate == ServoStates::OperationEnabled) {
            std::cout << "Changing servo " << slave_index_ << " state from SwitchedOn to OperationEnabled" << std::endl;
            std::cout << "Sending command EnableOperation to servo " << slave_index_ << std::endl;
            send_command(ServoCommands::EnableOperation); // 15
        }
        else if (currentstate == ServoStates::OperationEnabled && nextstate == ServoStates::SwitchedOn){
            std::cout << "Changing servo " << slave_index_ << " state from OperationEnabled to SwitchedOn" << std::endl;
            std::cout << "Sending command Disable operation to servo " << slave_index_ << std::endl;
            send_command(ServoCommands::DisableOperation); // 7
        }
        else if (currentstate == ServoStates::SwitchedOn && nextstate == ServoStates::ReadyToSwitchOn) {
            std::cout << "Changing servo " << slave_index_ << " state from SwitchedOn to ReadyToSwitchOn" << std::endl;
            std::cout << "Sending command Disable voltage to servo " << slave_index_ << std::endl;
            send_command(ServoCommands::DisableVoltage); // 6
        }

        return nextstate;
    }

    // Translates a command into setting certain bits of the controlword
    // See Table 3-3 in the manual
    void Servo::send_command(ServoCommands command) {

        //uint16_t controlword;
        std::bitset<16> bits(controlword_);

        switch(command) {
            case ServoCommands::Shutdown : {  
                bits[7]=0; 
                bits[2]=1; 
                bits[1]=1; 
                bits[0]=0; 
                break; }
            case ServoCommands::SwitchOn : {
                bits[7]=0; 
                bits[3]=0; 
                bits[2]=1; 
                bits[1]=1; 
                bits[0]=1; 
                break; }
            case ServoCommands::EnableOperation : {
                bits[7]=0; 
                bits[3]=1; 
                bits[2]=1; 
                bits[1]=1; 
                bits[0]=1; 
                break;} 
            case ServoCommands::DisableOperation : {
                bits[7]=0; 
                bits[3]=0; 
                bits[2]=1; 
                bits[1]=1; 
                bits[0]=1; 
                break;} 
            case ServoCommands::DisableVoltage : {
                bits[7]=0; 
                bits[1]=0; 
                break;} 
            case ServoCommands::QuickStop : {
                bits[7]=0; 
                bits[2]=0; 
                bits[1]=1; 
                break;} 
            case ServoCommands::FaultReset : {
                bits[7]=1; 
                break;} 
        }

        controlword_ = bits.to_ulong();

        write("Controlword",controlword_);

    }

    // Is performed every cycle of the async loop by the ServoCollection parent
    // Synchronizes the internal variables and the IOmap (as pointed to by ouputs and inputs)
    // Offset is counted in 8-bit bytes
    void Servo::update_IOmap(uint8_t* outputs, uint8_t* inputs) {

        mtx_.lock();

        for (DeviceObject* obj : tx_maporder_) {
            std::string name = obj->Name;
            int offset = tx_offsets_[name];
            if (obj->Type == "UINT16") {
                uint16_t value = tx_values_[name]->uint16_value;
                uint16_t *poutput  = (uint16_t*)(outputs+offset);
                *poutput = value;
            }
            else if (obj->Type == "INT16") {
                int16_t value = tx_values_[name]->int16_value;
                int16_t *poutput  = (int16_t*)(outputs+offset);
                *poutput = value;
            }
            else if (obj->Type == "INT32") {
                int32_t value = tx_values_[name]->int32_value;
                int32_t *poutput  = (int32_t*)(outputs+offset);
                *poutput = value;
            }
        }

        for (DeviceObject* obj : rx_maporder_) {
            std::string name = obj->Name;
            int offset = rx_offsets_[name];
            if (obj->Type == "UINT16") {
                uint16_t *pinput  = (uint16_t*)(inputs+offset);
                rx_values_[name]->uint16_value = *pinput;
            }
            else if (obj->Type == "INT16") {
                int16_t *pinput  = (int16_t*)(inputs+offset);
                rx_values_[name]->int16_value = *pinput;
            }
            else if (obj->Type == "INT32") {
                int32_t *pinput  = (int32_t*)(inputs+offset);
                rx_values_[name]->int32_value = *pinput;
            }

        }

        mtx_.unlock();

    }

    // Write a value to the internal variable corresponding to the entity
    void Servo::write(std::string entity_name, uint16_t value) {
        mtx_.lock();
        if (tx_objects_.count(entity_name) > 0) {
            if (tx_objects_[entity_name]->Type == "UINT16") {
                tx_values_[entity_name]->uint16_value = value;
            }
            else {
                std::runtime_error("Wrong type when writing " + entity_name);
            }
        }
        else {
            std::runtime_error("Mismatch of config file settings and program setting entity " + entity_name);
        }
        mtx_.unlock();
    }

    // Write a value to the internal variable corresponding to the entity
    void Servo::write(std::string entity_name, int16_t value) {
        mtx_.lock();
        if (tx_objects_.count(entity_name) > 0) {
            if (tx_objects_[entity_name]->Type == "INT16") {
                tx_values_[entity_name]->int16_value = value;
            }
            else {
                std::runtime_error("Wrong type when writing " + entity_name);
            }
        }
        else {
            std::runtime_error("Mismatch of config file settings and program setting entity " + entity_name);
        }
        mtx_.unlock();
    }

    // Write a value to the internal variable corresponding to the entity
    void Servo::write(std::string entity_name, int value) {
        mtx_.lock();
        std::cout << value << std::endl;
        std::cout << tx_objects_.count(entity_name) << std::endl;
        if (tx_objects_.count(entity_name) > 0) {
            if (tx_objects_[entity_name]->Type == "INT32") {
                tx_values_[entity_name]->int32_value = value;
            }
            else {
                std::runtime_error("Wrong type when writing " + entity_name);
            }
        }
        else {
            std::runtime_error("Mismatch of config file settings and program setting entity " + entity_name);
        }
        mtx_.unlock();
    }

    std::ostream& operator<<(std::ostream& strm, const Servo& s) {
        strm << "Slave name: " << s.name_ << std::endl;
        strm << "Position: " << s.slave_index_ << std::endl;
        strm << "Rx:" << std::endl;
        strm << "   Name\t\tIndex\t\tSubindex\tType" << std::endl;
        for (DeviceObject* dobj:s.rx_maporder_) {
            if (dobj->Name.length() > 5) {
                std::cout << "   " << dobj->Name << "\t" << dobj->Index << "\t\t" << dobj->Subindex << "\t\t" << dobj->Type << std::endl;
            }
            else {
                std::cout << "   " << dobj->Name << "\t\t" << dobj->Index << "\t\t" << dobj->Subindex << "\t\t" << dobj->Type << std::endl;
            }
        }
        strm << "Tx:" << std::endl;
        strm << "   Name\t\tIndex\t\tSubindex\tType" << std::endl;
        for (DeviceObject* dobj:s.tx_maporder_) {
            if (dobj->Name.length() > 5) {
               std::cout << "   " << dobj->Name << "\t" << dobj->Index << "\t\t" << dobj->Subindex << "\t\t" << dobj->Type << std::endl;
            }
            else {
               std::cout << "   " << dobj->Name << "\t\t" << dobj->Index << "\t\t" << dobj->Subindex << "\t\t" << dobj->Type << std::endl;
            }
        }
        return strm;
    }


    void Servo::SDOwrite_INT8(int slave, uint16_t index, uint8_t subindex, int8_t value)
    {
        int wck;
        wck = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
        printf("Write 8 => Slave: %u, Index: 0x%4.4x Subindex: %u, Size %lu, Value: %i , wck %i\n", slave, index, subindex, sizeof(value),value, wck);
        if (wck==0) {
            std::cout << "WARNING: wck equal to zero." << std::endl;
        }
    }

    void Servo::SDOwrite_UINT8(int slave, uint16_t index, uint8_t subindex, uint8_t value)
    {
        int wck;
        wck = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
        printf("Write 8 => Slave: %u, Index: 0x%4.4x Subindex: %u, Size %lu, Value: %u , wck %i\n", slave, index, subindex, sizeof(value),value, wck);
        if (wck==0) {
            std::cout << "WARNING: wck equal to zero." << std::endl;
        }
    }

    void Servo::SDOwrite_INT16 (int slave, uint16_t index, uint8_t subindex, int16_t value)
    {
        int wck;
        wck = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
        printf("Write16 => Slave: %u, Index: 0x%4.4x Subindex: %u, Size %lu, Value:  %i , wck %i\n", slave, index, subindex, sizeof(value),value, wck);
        if (wck==0) {
                std::cout << "WARNING: wck equal to zero." << std::endl;
        }
    }

    void Servo::SDOwrite_UINT16 (int slave, uint16_t index, uint8_t subindex, uint16_t value)
    {
        int wck;
        wck = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
        printf("Write16 => Slave: %u, Index: 0x%4.4x Subindex: %u, Size %lu, Value:  %u , wck %i\n", slave, index, subindex, sizeof(value),value, wck);
        if (wck==0) {
                std::cout << "WARNING: wck equal to zero." << std::endl;
        }
    }

    void Servo::SDOwrite_INT32(int slave, uint16_t index, uint8_t subindex, int32_t value)
    {
        int wck;
        wck = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
        printf("Write32 => Slave: %u, Index: 0x%4.4x Subindex: %u, Size %lu, Value:  %i , wck %i\n", slave, index, subindex, sizeof(value),value, wck);
        if (wck==0) {
                std::cout << "WARNING: wck equal to zero." << std::endl;
        }
    }

    void Servo::SDOwrite_UINT32(int slave, uint16_t index, uint8_t subindex, uint32_t value)
    {
        int wck;
        wck = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
        printf("Write32 => Slave: %u, Index: 0x%4.4x Subindex: %u, Size %lu, Value:  0x%4.4x , wck %i\n", slave, index, subindex, sizeof(value),value, wck);
        if (wck==0) {
                std::cout << "WARNING: wck equal to zero." << std::endl;
        }
    }

    // Can be used to read different data types
    void Servo::SDOread_INT32(int slave, uint16_t index, uint8_t subindex) {
        int32_t testsize = 8; 
        int32_t testval = 0;
        int wck = ec_SDOread(slave, index,subindex, FALSE, &testsize, &testval, EC_TIMEOUTRXM);
        printf("Read => Slave: %u, Index: 0x%4.4x Subindex: %u, Size %i, Value: %u , wck %i\n", slave, index, subindex, testsize, testval, wck);
    }

}
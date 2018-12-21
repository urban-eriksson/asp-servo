# asp-servo
EtherCat servo API utilizing the SOEM ethercat master

## Prerequisistes

`sudo apt install build-essentials cmake`

- Install SOEM library (my fork)

https://github.com/urban-eriksson/SOEM

Clone the repository

```
cd SOEM
mkdir build
cd build
cmake ..
sudo make install
```

- Install Tinyxml2 library from github source code

Clone the repository

```
cd tinyxml2
mkdir build
cd build
cmake ..
sudo make install
```

https://github.com/leethomason/tinyxml2


## Installation instructions

Clone this repository

```
cd asp-servo
mkdir build
cd build
cmake ..
sudo make install
```

## How to use

There should be an XML file describing the system configuration (see for example https://github.com/urban-eriksson/asp-servo-testapp).

```
<?xml version="1.0" encoding="utf-8"?>
<SystemConfiguration>
    <EthernetPort></EthernetPort>
    <CycleTime_ms></CycleTime_ms>
    <ServoCollection>
       <Servo position="" name="">
          <Type></Type>
          <StartupParameters>
            <!-- Items -->                        
          </StartupParameters>
          <PDOmapping>
            <Tx>
              <!-- Objects -->
            </Tx>
            <Rx>
              <!-- Objects -->
            </Rx>
          </PDOmapping>
       </Servo>
       <!-- More servos -->
    </ServoCollection>
</SystemConfiguration>
```
An object is as defined in the manual, e.g.:

```
<Object>
   <Name>Position</Name>
   <Index>0x6064</Index>
   <Subindex>0</Subindex>
   <Type>INT32</Type>
</Object>
```

The name is used when writing to and reading from the servo.

`servo_collection.write(servo_name,"Position",target_position);`

Be sure to use the correct data type for the target value.


In the StartupParameters tag an object is combined with a value to form an Item:

```
<Item>
   <Object>
      <Name>Mode of Operation</Name>
      <Index>0x6060</Index>
      <Subindex>0</Subindex>
      <Type>INT8</Type>
   </Object>
   <Value>10</Value>
</Item>  
```  

# asp-servo
EtherCat servo API utilizing the SOEM ethercat master

## Prerequisistes

'sudo apt install build-essentials cmake'

- Install SOEM library (my fork)

https://github.com/urban-eriksson/SOEM

Clone the repository

'''
cd SOEM
mkdir build
cd build
cmake ..
sudo make install
'''

- Install Tinyxml2 library from github source code

Clone the repository

'''
cd tinyxml2
mkdir build
cd build
cmake ..
sudo make install
'''



https://github.com/leethomason/tinyxml2


## Installation instructions

Clone this repository

'''
cd asp-servo
mkdir build
cd build
cmake ..
sudo make install
'''


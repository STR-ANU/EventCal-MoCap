# Real-time 'Asynchronous Multi-Object Tracking with an Event Camera' (AEMOT) with Neuromorphic Drivers

> Now in real-time!

### <ins>Built with:</ins>
[![AEMOT by Angus Apps et al](https://img.shields.io/badge/AEMOT%20by%20Angus%20Apps%20et%20al.-BD9048?style=for-the-badge)](https://github.com/angus-apps/AEMOT)  
[![Gen4 Neuromorphic Drivers](https://img.shields.io/badge/Gen4%20Neuromorphic%20Drivers-A66F38?style=for-the-badge)](https://github.com/neuromorphicsystems/gen4)

This is currently a work in progress.

This package takes the core of a reduced version of the AEMOT code (developed by Angus Apps, Ziwei Wang, Vladimir Perejogin, Timothy L. Molloy, and Robert Mahony) and configures it for real-time event data input and tracking using Neuromorphic System's Gen4 C++ Neuromorphic Drivers.

You can see the original full AEMOT code on GitHub above. The exact, reduced, code that this package here was based on was provided directly by one of the original authors, and is not publically accessible at the time of writing this.

***Currently only works with the Prophesee Gen4 EVK4 event camera!***


## How to install and build
***<ins>Enviornment note:</ins>*** This set up has been designed and tested for an Ubuntu setup (tested on Ubuntu 26.04, but a solid earlier version like Ubuntu 24.04 will likely work even better). Running on Windows is probably possible, but you will need to consult the gen4 page on how to build their drivers with Windows (not covered on this page, Ubuntu is assumed here).

This set up requires Neuromorphic System's Gen4 Neuromorphic Drivers to interact with the Gen4 camera in real-time.  
The project layout needs to look like this (so that `aemot_realtime` and `gen4` are ***sister*** folders):
```txt
working_directory
    |
    |---- /aemot_realtime
    |         |---- /configs
    |         |---- /src
    |         |---- README.md
    |
    |---- /gen4
    |       |---- /app
    |       |---- /common
    |       |---- ... etc.
```
To do this, follow these steps:

**<ins>Step 1: Install and build the gen4 content</ins>**  
See the [gen4 GitHub page](https://github.com/neuromorphicsystems/gen4) for further details, if needed...
```bash
# create a working directory and gen4 subfolder:
# note that the working_directory folder name can be what you want (just be consistent)
cd ~
mkdir -p working_directory/gen4
cd working_directory/gen4

# install prerequisites:
sudo apt install -y curl build-essential git libusb-1.0-0-dev qtbase5-dev qtdeclarative5-dev qml-module-qtquick-controls qml-module-qtquick-controls2 qml-module-qttest

# clone the gen4 repository (ensure you add the '.' at the end)
git clone https://github.com/neuromorphicsystems/gen4.git .

# build the content
cd app
curl -L https://github.com/premake/premake-core/releases/download/v5.0.0-beta2/premake-5.0.0-beta2-linux.tar.gz | tar xz
./premake5 gmake
cd build
make

# create system rules to access the cameras properly
sudo nano /etc/udev/rules.d/65-event-based-cameras.rules
# paste the following:
SUBSYSTEM=="usb", ATTR{idVendor}=="152a",ATTR{idProduct}=="84[0-1]?", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="04b4",ATTR{idProduct}=="00f[4-5]", MODE="0666"
# then press ctr+O and then ENTER to save, then ctr+X to exit.
```

**<ins>Step 2: Install this repository</ins>**  
```bash
# create the aemot_realtime subfolder:
cd ~/working_directory
mkdir aemot_realtime
cd aemot_realtime

# clone this repository (ensire you add the '.' at the end)
git clone https://github.com/TobysWorkshop/AEMOT-realtime.git .
```

**<ins>Step 3: Build the project!</ins>**  
The included CMakelists.txt should resolve the relative file paths and includes as necessary. To build, then, do the following:
```bash
# install dependencies (just do this once on project set up)
sudo apt install cmake libboost-dev libopencv-dev libyaml-cpp-dev libeigen3-dev libusb-1.0-0-dev

# navigate to aemot_realtime/ and build!
cd ~/working_directory/aemot_realtime
cmake -B build
cmake --build build -j$(nproc)
# This will create the build/ directory inside aemot_realtime/
```

Now everything is ready to use!

***<ins>Notes on different setups:</ins>*** This exact sister-folder setup is required because some of the C++ files in this project have hard-coded references to include required files from the gen4 directory. If you'd like to alter the file arrangements, or even merge this project code with the required gen4 files, then you will need to update these `#include` references.  
NOTE: the above has now been changed to rely on the includes in the CMakelists.txt, rather than hard-coded #includes in the individual .cpp files.

## How to use
This project provides two ways of running the system: realtime streaming from a Gen4 camera, and replaying from a .es file. 

Once you've built the project, you can run it using the following command (from `aemot_realtime/`):
```bash
cd ~working_directory/aemot_realtime
./build/aemot_realtime <config_name>
```
Where `<config_name>` is the name of your desired config file. This must be located inside the `aemot_realtime/configs/` directory.  
For example, if I have a config file `aemot_realtime/configs/bees.yaml`, then I would simply pass `bees` into the command (**<ins>not</ins>** `bees.yaml`!):  
`./build/aemot_realtime bees`

If no `<config_name>` is provided, it should default to the included `default.yaml` file: `aemot_realtime/configs/default.yaml`.

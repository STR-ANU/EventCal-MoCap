<a name="top"></a>

# Real-time 'Asynchronous Multi-Object Tracking with an Event Camera' (AEMOT) with Neuromorphic Drivers

> Now in real-time!

![system diagram dark mode](system_dark.png)  
*If you're using light mode, the above system diagram might be hard to see. If so, check out [this light-mode-friendly version](system_light.png) instead :)*

## Quick Links!
[![How to install and build](https://img.shields.io/badge/How%20to%20Install%20and%20Build-BD9048?style=for-the-badge)](#how-to-install-and-build)  
[![How to use the system](https://img.shields.io/badge/How%20to%20use%20the%20system-4A3A31?style=for-the-badge)](#how-to-use)  
[![How logging tracks to file works](https://img.shields.io/badge/How%20logging%20tracks%20to%20file%20works-A66F38?style=for-the-badge)](#logging-tracks-to-file)  
[![Understanding the config file](https://img.shields.io/badge/Understanding%20the%20config%20file-4A3A31?style=for-the-badge)](#understanding-the-config-file) 

## About

### ***For academic use only!***

This is currently a work in progress.

This package takes the core of a reduced version of the AEMOT code (developed by Angus Apps, Ziwei Wang, Vladimir Perejogin, Timothy L. Molloy, and Robert Mahony) and configures it for real-time event data input and tracking using Neuromorphic System's Gen4 C++ Neuromorphic Drivers.

You can see the original full AEMOT code on GitHub below. The exact, reduced, code that this package here was based on was provided directly by one of the original authors, and is not publicly accessible at the time of writing this.

***Currently only works with the Prophesee Gen4 EVK4 event camera!***

### <ins>Built with:</ins>
[![AEMOT by Angus Apps et al](https://img.shields.io/badge/AEMOT%20by%20Angus%20Apps%20et%20al.-BD9048?style=for-the-badge)](https://github.com/angus-apps/AEMOT)  
[![Gen4 Neuromorphic Drivers](https://img.shields.io/badge/Gen4%20Neuromorphic%20Drivers-A66F38?style=for-the-badge)](https://github.com/neuromorphicsystems/gen4)


<p align="right"><a href="#top">↑ go back to top</a></p>

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

<p align="right"><a href="#top">↑ go back to top</a></p>

## How to use
This project provides two ways of running the system: <ins>realtime streaming from a gen4 camera</ins>, and <ins>replaying from a .es file</ins>.

### <ins>Streaming from a gen4 camera (`aemot_realtime`)</ins>
Once you've built the project, you can run the ***aemot_realtime*** executable using the following command (from `aemot_realtime/`):
```bash
cd ~working_directory/aemot_realtime
./build/aemot_realtime <config_name>
```
Where `<config_name>` is the name of your desired config file. This must be located inside the `aemot_realtime/configs/` directory.  
For example, if I have a config file `aemot_realtime/configs/bees.yaml`, then I would simply pass `bees` into the command (**<ins>not</ins>** `bees.yaml`!):  
`./build/aemot_realtime bees`

If no `<config_name>` is provided, it should default to the included `default.yaml` file: `aemot_realtime/configs/default.yaml`.

### <ins>Replaying from a .es file (`aemot_replay`)</ins>
Once you've built the project, you can run the ***aemot_replay*** executable using the following command (from `aemot_realtime/`):
```bash
cd ~working_directory/aemot_realtime
./build/aemot_replay <config_name> <.es_file_location> <camera_name>
```
Where `<config_name>` follows the same requirements as above, and `<.es_file_location>` is a path relative to `aemot_realtime/`. There is a `/data` folder included in this repo, which is designed for this very purpose: place your .es files in here and then the command becomes:
```
./build/aemot_replay <config_name> ./data/<file_name>.es camera_1
```

<p align="right"><a href="#top">↑ go back to top</a></p>

## Logging tracks to file
The system has a built-in ability to log every validated track's kalman state at each update instance to a custom ***.bees (Binary Event Evolution Storage) file***. This is done by default while the system is running in real-time or replaying from a file.  
The system will also create a custom ***.beesum (Binary Event Evolution Summary) file***. This contains a single summary log per validated track, which holds some key metrics about that track, including why it ended, as well as the covariance at its endpoint.

Upon starting a run, a pair of .bees and .beesum files for that run will be created in the `aemot_realtime/track_logs/` folder with a timestamped file name: `<DD-MM-YYYY-hh-mm-ss>.bees` and `<DD-MM-YYYY-hh-mm-ss>.beesum`, respectively.

### <ins>.bees File Format:</ins>

| Item | Type | Value | Bytes |
| -------- | -------- | -------- | -------- |
| **<ins>Header (24 bytes)</ins>** |
| Magic bytes  | char[8] | "A E M O T L O G"  | 8  |
| version | uint32 | 2 | 4 |
| n_state | uint32 | 8, *length of each track update's state vector* | 4 |
| record_size | uint32 | 80, *the size of one kalman log record* | 4 |
| reserved | uint32 | 0 | 4 |
| **<ins>Kalman log record (80 bytes), repeated</ins>** |
| track_id | uint64 | *unique id of the track this update belongs to* | 8 |
| ts | double | time | 8 |
| Kalman filter state vector, *x_hat[8]* | double | x, y, vx, vy, lambda1, lambda2, theta, q | 64 |
| **. . .** |

### <ins>.beesum File Format:</ins>

| Item | Type | Value | Bytes |
| -------- | -------- | -------- | -------- |
| **<ins>Header (24 bytes)</ins>** |
| Magic bytes  | char[8] | "A E M O T S U M"  | 8  |
| version | uint32 | 2 | 4 |
| state_dim | uint32 | 8, *length of each track update's state vector* | 4 |
| record_size | uint32 | 624, *the size of one track summary record* | 4 |
| reserved | uint32 | 0 | 4 |
| **<ins>Track summary record (624 bytes), repeated</ins>** |
| track_id | uint64 | *unique id of the track this update belongs to* | 8 |
| t_created | double | *time this track was first created* | 8 |
| t_validated | double | *time this track was validated* | 8 |
| t_deleted | double | *time this track ended* | 8 |
| num_records | uint32 | *how many individual logs this track has in the .bees file* | 4 |
| delete_reason | uint32 | *6-option bitmask, why was the track deleted? (see table below)* | 4 |
| event_rate_at_deletion | double | *1.0 / dt_moving_average at the time the track ended* | 8 |
| x_hat_at_deletion[8] | double | *Kalman state vector when the track ended:* x, y, vx, vy, lambda1, lambda2, theta, q | 64 |
| P_at_deletion[64] | double | *row major, symmetric, flattened P matrix when the track ended* | 512 |
| **. . .** |

### <ins>Deletion Reason Bitmask *(for the relevant .beesum file parameter - see above)*:</ins>
| Reason | Bitmask |
| -------- | -------- |
| OUT_OF_FRAME | 1u << 0 |
| INACTIVE | 1u << 1 |
| LOW_ACTIVITY | 1u << 2 |
| BAD_SHAPE | 1u << 3 |
| DUPLICATE_TRACK | 1u << 4 |
| STILL_ACTIVE_AT_SHUTDOWN | 1u << 5 |

Note that multiple bits can be set, e.g. a track can be both out of frame AND inactive simultaneously. This just logs whatever checks failed when this track was deleted. The *STILL_ACTIVE_AT_SHUTDOWN* means that the track was still active when the system shut down.

<p align="right"><a href="#top">↑ go back to top</a></p>

### <ins>Optional additional file type: associated event-level logging *(.aemotevt)*</ins>
This system also has the additional capability of logging at the event-level, rather than the kalman state. This can be activated by setting the config variable `create_event_log_file` to 1 (this is set to 0 by default - see the section below on the config file).  
When activated, a separate ***.aemotevt (AEMOT event) file*** will be created alongside the standard .bees and .beesum files. The .aemotevt file contains logs of each event that was successfully associated to a track during the nearest-neighbour data association step. The file is formatted as follows:

| Item | Type | Value | Bytes |
| -------- | -------- | -------- | -------- |
| **<ins>Header (24 bytes)</ins>** |
| Magic bytes  | char[8] | "A E M O T E V T"  | 8  |
| version | uint32 | 1 | 4 |
| reserved | uint32 | 0 | 4 |
| record_size | uint32 | 24, *the size of one event log record* | 4 |
| reserved | uint32 | 0 | 4 |
| **<ins>Event log record (24 bytes), repeated</ins>** |
| t | double | *time* | 8 |
| track_id | uint32 | *unique id of the track this update belongs to* | 4 |
| x | uint16 | *pixel x-position* | 2 |
| y | uint16 | *pixel y-position* | 2 |
| p | uint8 | *polarity: 1 = ON, 0 = OFF* | 1 |
| reserved[7] | uint8 | {0, 0, 0, 0, 0, 0, 0} | 7 |
| **. . .** |

While this is a custom binary file format, a python converter script has been included (see [track_logs/aemotevt_to_es.py](track_logs/aemotevt_to_es.py)), which will convert an ***.aemotevt*** file to a ***.es*** file.  
Since the system saves the .aemotevt file in the track_logs/ folder by default, the script can be run as follows:
```bash
# from the aemot_realtime folder:
python3 ./track_logs/aemot_to_es.py <file_name>
# by default, this will save the converted file to track_logs/<file_name>.es (the same base file name as the input file)
# optional extra arguments:
python3 ./track_logs/aemot_to_es.py <file_name> --output <custom file name> --width <custom sensor width> --height <custom sensor height>
# Setting a custom output name will save the file to track_logs/<custom file name>.es
```

<p align="right"><a href="#top">↑ go back to top</a></p>

## Understanding the Config File
A .yaml config file is required to run this system. This carries a whole bunch of tunable parameters that change how the system functions - from altering the Kalman filter setup, to how track evaluation is performed, and even whether frames are rendered for user viewing.

Below is the `default.yaml` file, which is included in this repository (it can be found at [/configs/default.yaml](/configs/default.yaml).

Many of these parameters do not need to be changed. However, some notable ones include (in order of appearance):
* ***show_display*** and ***save_files***: set these to 0 and 1, respectively, for standard system performance. Having show_display on will massively reduce processing speeds, and is unfit for any meaningful realtime usage (but good for replaying from files when speed is not an issue).
* ***pool_size***: increasing this allows for more objects to be tracked at once, but will also reduce some performance.
* ***use_dt_detector***: the SAEdetector.cpp has two detector types. By default, the standard SAE detector is used. The dt detector has not yet been fully tested with this realtime setup.
* ***dist_threshold***: set this with your expected scene and objects in mind. This realtime version of AEMOT does not use an automatically adjusting distance threshold, and instead uses this as a hard gate.
* ***publish_framerate***: if using the display, set this to control how frequently the display updates. Making this larger will allow you to diagnose how tracking is performing, but will massively slow down the system (see above on show_display).
* ***accumulator_count_thresh*** and ***accumulator_time_thresh***: this realtime version of AEMOT uses batched kalman updates to speed up the system and reduce the size of the output files. Increasing these parameters will mean we perform a kalman update less often. This will speed up the system and will produce fewer logs in the .bees file (a log is written every kalman update), but may reduce the accuracy of the tracking (the cobject's centroid gets smeared more the longer we wait for a batched kalman update).
* ***corroboration_cell_size*** and ***corroboration_window***: this realtime version of AEMOT uses a corroboration grid to further gate what events get to trigger new track creation. Simply put, when an event triggers a positive SAE detection, it will only be allowed to create a new track if another positive detection has occurred recently in its grid cell. Decreasing both of these parameters makes it much harder for events to spawn new tracks, and will greatly reduce the ability for background noise to consume candidate tracks.

```yaml
## -------------------------- ##
## AEMOT_realtime config file ##
## -------------------------- ##

## General ##
# ---------------- #
width: 1281 # width (in pixels) of the camera's sensor + 1
height: 721 # height (in pixels) of the camera's sensor + 1

dt: 0.0001  # time step to be used throughout calculations

show_display : 0 # 1 = show the display, 0 = don't show the display. SWITCH TO 0 FOR REAL_TIME OPPERATION!
save_files : 1 # 1 = save .bees and .beesum files, 0 = don't save these files
create_event_log_file : 0 # 1 = save a per-event log file of just associated events, 0 = don't save this file (0 RECOMMENDED UNLESS REQUIRED FOR A SPECIFIC PURPOSE)
# ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- #


## Track Manager ##
# ---------------- #
pool_size : 20 # how many Kalman filters to set up at the start and hold in a lineup for reuse. 
use_dt_detector: 0 # 1 = use the dt detector, 0 = use the standard SAE detector.
dist_threshold: 10 # radius for region around an object where events that fall inside it are associated to that object (used in the nearest neighbour calculations in the main processing.cpp loop)
ring_buffer_len: 10

# Track evaluation
f_evaluate                    : 1       # 1 = standard track evaluation, 0 = only end tracks when they exit the frame
evaluate_ts_age               : 0.08    # how long (in seconds) to wait before allowing a young track to be evaluated (to let it get some events and initialise)
evaluate_dt_terminate         : 0.12    # time (in seconds) we allow a track to have no events before deleting 
rate_per_area                 : 1.0     # factor to scale the event rate threshold based on the area of the blob (events/area). This allows larger blobs to have a higher threshold for activity, while smaller blobs can be validated with fewer events.
event_rate_threshold          : 1000    # event rate required for validation
evaluate_low_activity_factor  : 0.15    # factor of event rate threshold to delete track
### the required event-rate that a track must meet = rate_per_area * (the blob's width * height), clamped to a min of event_rate_threshold * evaluate_low_activity_factor
# ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- #


## Renderer, high-pass, and display ##
# ---------------- #
publish_framerate: 200 # how often do we publish a frame to the display (frames/second)
contrast_threshold : 0.1 # controls how much an event darkens its pixel in the rendered frame
alpha: 50 # controls how fast rendered events decay on the frame (get lighter and eventually fade)

f_show_candidates   : 1 # 1 = display candidate tracks in blue (alongside the usual validated tracks in red) on the rendered frame, 0 = just display the validated tracks
disp_covariance_flag: 0 # 1 = display covariance regions on the rendered frame, 0 = don't display covariance
# ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- #


## Kalman Filters ##
# ---------------- #
n_state: 10 # size of the kalman state vector

# Initial size param (sets both initial lambda_1 and lambda_2)
lambda_init: 25

# Initial covariance values
var_x: 30
var_y: 30   
var_vx: 50000 #was: 8000
var_vy: 50000 #was: 8000
var_lambda_1: 5
var_lambda_2: 5
var_theta: 0.5
var_q: 2

# Diagonal process noise values
q_x: 5000
q_y: 5000
q_vx: 40000
q_vy: 40000
q_lambda_1: 0.1
q_lambda_2: 0.1
q_theta: 0.005
q_q: 0.01

# Batch Kalman Accumulators
accumulator_count_thresh : 8 # how many events do we batch before averaging to perform a single kalman update step?
accumulator_time_thresh : 0.002 # how many seconds do we wait before we automatically trigger the above kalman update step (if we don't pool enough events in time)?
# ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- #


## SAE detector ##
# ---------------- #
SAE_operation_rate            : 1   # how many events between each SAE detection. 1 is recommended for the current setup, as this produces the best tracking results

detector_dist_threshold       : 50  # distance from existing track to allow even making a detection in the first place

SAE_ksize                     : 7
SAE_alpha                     : 3
SAE_detection_threshold       : 0.9 # val = cos(theta). Check the note below on corroboration_direction_cos_threshold, as these should be loosely coupled
SAE_min_active_pixels         : 10
SAE_min_contributions         : 12

SAE_recency_window            : -1  # an event will be rejected from the SAE detector check if it has no other events in its patch that fired within this recent time window (in seconds)
### set the above recency window to a NEGATIVE NUMBER if you don't want to use this check - it will be ignored.

# for dt detection:
detector_dt_threshold         : 0.001 # maximum dt in sae patch around new event to make a detection
# ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- #


## Corroboration grid ##
# ---------------- #
corroboration_cell_size : 5 # pixels - set this to roughly your typical object size. A positive detection must find another recent detection in its own cell to create a new track
corroboration_window : 0.002 # how recent a neighbouring detection must be to allow creating a new track (in seconds)
corroboration_direction_cos_threshold : 0.6 # set this to be looser than SAE_detection_threshold in the SAE section above - a rough sanity check only
# ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- #

```

<p align="right"><a href="#top">↑ go back to top</a></p>

## Running the Included Offline Track Plotting/Verification
WIP...

<p align="right"><a href="#top">↑ go back to top</a></p>

## Other notes
System diagram at the top of this README was made using [Excalidraw](https://excalidraw.com). See their GitHub page [HERE](https://github.com/excalidraw/excalidraw) for more info.


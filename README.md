# UNDLunarRobotics22
Code for the UND robot that will compete in Nasa Lunar Robotics competition in may 2022.

# Prerequisites
 - ### Software
    - `SDL2`
      - prefer `libsdl2-dev`
    - `opencv` with `opencv_contrib`
    - CTRE's **Phoenix Library**
      - Provides API for utilizing TalonSRX/ CTRE Brand MotorControllers over CAN

 - ### Hardware
   - SocketCAN Adapter
    - `CANable` used on robot
    - Allows interfacing Linux Main Control Unit with CAN Bus
    - CTRE's **Phoenix Library** leverages SocketCAN standard on non-frc builds
  - CTRE's **TalonSRX MotorControllers**
    - This code leverages CTRE's Library so it can be easily extended to cover other options
  - Linux Based Main Control Unit
    - Nvidia Jetson Products
    - Nvidia Embedded Products
    - Raspberry Pi
    - A laptop running your flavor of Linux
    - etc.

 - ### Resources and links
   - Google


# Quick-Start
  - ## Instalation
    - `git clone` this repository
      - **ex.** `git clone git@github.com:PlavaFeniks\UNDLunarRobotics22.git`
    - navigate to installation location in terminal
      - **ex.** If `git clone` was run at `~/Documents` then run `cd ~/Documents/UNDLunarRobotics22`
    - run `sudo bash install.sh` with desired optional flags
        - `-p` will modify the sources list on raspberry pi's 
          - so that they actually can run `sudo apt-get install <packageName>` or `sudo apt-get <upgrade/update>`
        - `-o` will also run the install scripts portion dedicated to installing Opencv with Contributions
          - This will take a significant amount of time
            - `<40 min` on a **Raspberry Pi 4**
  **Congratulations** 
    - You have just installed our interface into TalonSRX MotorControllers
    - Your device now has a service called `robo.service` that allows for **plug and play** functionality
      - This allows the robot to be manually driven using an **xbox controller** and a **SocketCAN** adapter
      - ### Working with Services
        - Started manually with
            `sudo systemctl start robo`
        - This can be disabled (will not launch on startup) with 
            - `sudo systmectl disable robo.service`
        - Modified by editing
            - `etc/system/systemd/robo.service`
            - changes loaded with `sudo systemctl daemon-reload`
            - May need to modify the `execstart` line depending on where the install script was run
        - Stopped manually with 
            - `sudo systemctl stop robo`
    - Installed the CTRE API at `/usr/local/ctre`
      - Object Libraries are found at `/usr/local/ctre/lib/<your CPU architecture>`
      - Include Headers are found at `/usr/local/ctre/include`
        - Can be included into files with `<ctre.h>`, `<ctre/etc.h>`
          - No need for relative/absolute pathing with `"ctre/header.h"`, GCC does it for you!!
    - Potentially installed `opencv` with `opencv_contrib`
    - 
    
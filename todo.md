# housekeeping
        --include etc/network/interfaces file
        --include etc/system/systemd/robot.service
        --enable on pi after reflash
    include bash script to run cmake . and make
    create clean.sh script
        --reflash raspberry pi

# cmake
        --/add functionality for building multiple targets/-
        --change current cmakeLists to reflect target platforms

# example.cpp
        --Clean up code to only use talonpairs defined in movement.h
        --Clean up code and remove unncessary print statements
    
# movement.h
    Add functionality for controllmode:: PID
        --enum PID ControlMode::Velocity

S
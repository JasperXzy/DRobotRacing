#!/bin/bash

echo "Setting up udev rules for Chassis and Lidar..."

# Chassis Rule
# Matches USB-Serial with ID 1a86:55d4 and Serial "0002"
echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ATTRS{serial}=="0002", MODE:="0777", GROUP:="dialout", SYMLINK+="chassis"' | sudo tee /etc/udev/rules.d/99-chassis.rules > /dev/null
echo "Written /etc/udev/rules.d/99-chassis.rules"

# Lidar Rule
# Matches USB-Serial with ID 1a86:55d4 and Serial "5B21244110"
echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ATTRS{serial}=="5B21244110", MODE:="0777", GROUP:="dialout", SYMLINK+="lidar"' | sudo tee /etc/udev/rules.d/99-lidar.rules > /dev/null
echo "Written /etc/udev/rules.d/99-lidar.rules"

echo "Reloading udev rules..."
sudo service udev reload
sudo service udev restart

echo "Udev rules setup complete."

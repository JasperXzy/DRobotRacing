sudo sh -c 'echo "KERNEL==\"ttyACM*\", ATTRS{idVendor}==\"1a86\", ATTRS{idProduct}==\"55d4\", ATTRS{serial}==\"5B21244110\", MODE:=\"0777\", GROUP:=\"dialout\", SYMLINK+=\"lidar\"" > /etc/udev/rules.d/wheeltec_wheeltec.rules'
sudo service udev reload
sudo service udev restart
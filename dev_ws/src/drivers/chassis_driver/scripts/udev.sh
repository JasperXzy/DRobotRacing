sudo sh -c 'echo "KERNEL==\"ttyACM*\", ATTRS{idVendor}==\"1a86\", ATTRS{idProduct}==\"55d4\", ATTRS{serial}==\"0002\", MODE:=\"0777\", GROUP:=\"dialout\", SYMLINK+=\"chassis\"" > /etc/udev/rules.d/chassis.rules'
sudo service udev reload
sudo service udev restart
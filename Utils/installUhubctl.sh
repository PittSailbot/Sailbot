cd ~
sudo apt-get remove uhubctl -y
sudo apt-get install libusb-1.0-0-dev
git clone https://github.com/mvp/uhubctl
cd uhubctl
make
sudo make install
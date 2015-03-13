set -e
cd rrtplugin
make
sudo make install
clear
cd ..
python HW3.py

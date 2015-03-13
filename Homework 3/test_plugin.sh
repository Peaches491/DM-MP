set -e

cd rrtplugin
make
sudo make install
clear
python testplugin.py
cd ..

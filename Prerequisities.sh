#!/usr/bin/env bash

set -e

echo "=== Installing system dependencies ==="
sudo apt update
sudo apt install -y git wget unzip flex bison gperf cmake ninja-build ccache \
    libffi-dev libssl-dev dfu-util libusb-1.0-0 python3 python3-pip python3-venv \
    cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0 \
	git gcc g++ pkg-config cmake libssl-dev libdbus-1-dev \
     libglib2.0-dev libavahi-client-dev ninja-build python3-venv python3-dev \
     python3-pip unzip libgirepository1.0-dev libcairo2-dev libreadline-dev \
     default-jre


echo "=== Cloning ESP-IDF v5.5.2 ==="
cd ~
if [ ! -d "esp-idf" ]; then
    git clone -b v5.5.2 --recursive https://github.com/espressif/esp-idf.git
else
    echo "ESP-IDF already exists, skipping clone"
fi

echo "=== Installing ESP-IDF tools ==="
cd ~/esp-idf
./install.sh all

echo "=== Activating ESP-IDF environment ==="
source export.sh

echo "=== Cloning ESP-Matter (release/v1.5, shallow) ==="
cd ~
if [ ! -d "esp-matter" ]; then
    git clone https://github.com/espressif/esp-matter.git
    cd esp-matter
    git checkout release/v1.4.2
   git submodule update --init --depth 1
  cd ./connectedhomeip/connectedhomeip
./scripts/checkout_submodules.py --platform esp32 darwin --shallow
else
    echo "ESP-Matter already exists, skipping clone"
    cd esp-matter
fi
echo "=== Installing ESP-Matter Python dependencies ==="
cd ~/esp-matter
pip install -r requirements.txt
./install.sh
source ./export.sh

echo "=== Setup complete ==="
echo "To build ESP-Matter:"
echo "  cd ~/esp-idf && source export.sh"
echo "  cd ~/esp-matter/examples/contact-sensor"
echo "  idf.py set-target esp32h2"
echo "  idf.py build"

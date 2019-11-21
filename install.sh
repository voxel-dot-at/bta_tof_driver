#!/bin/bash

# Install BltTofAPI
echo "Install BltTofAPI Version v3.0.1"

# Download API Package
wget http://datasheets.bluetechnix.at/goto/BltTofApi/v3.0/binaries/BltTofApi_v3.0.1_Lin_noUsb_noJpg.zip

# Unzip Archive
unzip BltTofApi_v3.0.1_Lin_noUsb_noJpg.zip

# Move required folders into Repo
mv BltTofApi_v3.0.1_Lin_noUsb_noJpg/inc/ .
mkdir lib
tar -zxvf BltTofApi_v3.0.1_Lin_noUsb_noJpg/lib/Lin_x64/libbta.so.tar.gz -C ./lib/

# Clean Up
rm -rf BltTofApi_v3.0.1_Lin_noUsb_noJpg BltTofApi_v3.0.1_Lin_noUsb_noJpg.zip

# Done
echo "Done"

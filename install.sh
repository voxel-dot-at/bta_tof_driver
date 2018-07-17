#!/bin/bash

# Install BotTofAPI
echo "Install BtoTofAPI Version v2.5.2"

# Download API Package
wget http://datasheets.bluetechnix.at/goto/BltTofApi/v2.5/binaries/BltTofApi_v2.5.2.zip

# Unzip Archive
unzip BltTofApi_v2.5.2.zip

# Move required folders into Repo
mv BltTofApi_v2.5.2/inc/ .
mkdir lib
tar -zxvf BltTofApi_v2.5.2/lib/Lin_x64/libbta.so.tar.gz -C./lib/

# Clean Up
rm -rf BltTofApi_v2.5.2 BltTofApi_v2.5.2.zip

# Done
echo "Done"

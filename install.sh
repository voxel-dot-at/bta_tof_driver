#!/bin/bash

# Install BotTofAPI
echo "Install BtoTofAPI Version 2.1..."

# Download API Package
wget http://datasheets.bluetechnix.at/goto/BltTofApi/v2.1/binaries/BltTofApi_v2.1.0_Win_x86_x64_Lin_x64_Armhf.zip

# Unzip Archive
unzip BltTofApi_v2.1.0_Win_x86_x64_Lin_x64_Armhf.zip

# Move required folders into Repo
mv BltTofApi_v2.1.0_Win_x86_x64_Lin_x64_Armhf/inc/ .
mv BltTofApi_v2.1.0_Win_x86_x64_Lin_x64_Armhf/lib/ .

# Clean Up
rm -rf BltTofApi_v2.1.0_Win_x86_x64_Lin_x64_Armhf BltTofApi_v2.1.0_Win_x86_x64_Lin_x64_Armhf.zip

# Done
echo "Done"

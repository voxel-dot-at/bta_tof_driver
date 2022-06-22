#!/bin/bash


PV=3.3.4
PN=BltTofApi_v${PV}_Win_x86_x64_Lin_x64

# Install BltTofAPI
echo "Install BltTofAPI Version v${PV}"


# Download API Package
wget http://datasheets.bluetechnix.at/goto/BltTofApi/v3.3/binaries/${PN}.zip

# Unzip Archive
unzip ${PN}.zip

# Move required folders into Repo
mv ${PN}/inc/ .
mkdir lib
tar -zxvf ${PN}/lib/Lin_x64/libbta.so.tar.gz -C ./lib/

# Clean Up
rm -rf ${PN} ${PN}.zip

# Done
echo "Done"

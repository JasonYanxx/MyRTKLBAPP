#!/bin/bash

for xx in {01..30}; do
    url="https://cddis.nasa.gov/archive/gnss/data/daily/2023/brdc/BRDM00DLR_S_20230${xx}0000_01D_MN.rnx.gz"
    filename="BRDM00DLR_S_20230${xx}0000_01D_MN.rnx.gz"
    
    # Download the file using curl
    curl -O "$url"
    
    # Check if the download was successful
    if [ $? -eq 0 ]; then
        echo "Downloaded $filename"
    else
        echo "Failed to download $filename"
    fi
done
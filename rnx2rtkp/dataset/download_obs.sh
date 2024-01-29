#!/bin/bash

for xx in {01..30}; do
    url="https://noaa-cors-pds.s3.amazonaws.com/rinex/2023/0${xx}/chti/chti0${xx}0.23o.gz"
    filename="chti0${xx}0.23o.gz"
    
    # Download the file using curl
    curl -O "$url"
    
    # Check if the download was successful
    if [ $? -eq 0 ]; then
        echo "Downloaded $filename"
    else
        echo "Failed to download $filename"
    fi
done
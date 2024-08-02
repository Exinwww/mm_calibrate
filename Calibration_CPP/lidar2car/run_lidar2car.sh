#!/bin/bash
dataPath=""
outputPath=""

if ["$1" == ""]; then
    dataPath="../data/Lidar/example/"
else
    dataPath=$1
fi
if ["$2" == ""]; then
    outputPath="../output/Lidar/"
else
    outputPath=$2
fi
echo "data path: $1"
echo "output path: $2"

echo "################ get pitch roll height #################"
./bin/run_prh $dataPath $outputPath
echo "##################### get yaw ##########################"
./bin/run_yaw $dataPath $outputPath

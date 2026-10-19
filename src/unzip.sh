#!/bin/bash
savepath=$PWD
cd ${savepath}
for file in $(find . -name '*.zip')
do
    unzip "$file" -d ${savepath}/"${file%.zip}"
done

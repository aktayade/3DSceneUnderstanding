#!/bin/bash
rm *.txt
rm *.pcd
./livefeatextract final_johnny.prm
cd ../
matlab -nodesktop -nosplash -r main_office_live
cd runall
./livevisualize

Files of interest are home.mat and office.mat

Starting with *_data_nodefeats.txt converts to *_raw.mat, and after applying *_mapping using the matlab scripts map_*.m we get the final outputs home.mat and office.mat, which exclude uncommon segment types and combines a few segment types into a single class (windows and walls for home).  In the end there are 17 classes for each scene type.
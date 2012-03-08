from string import *
from numpy import *
import pickle
import numpy, time, cPickle, gzip, sys, os

datf = open('data_split5_train.txt')
lines = datf.readlines()

# Loop through each line and perform operations
OddLine = True
FeatureMat = []
LabelVec = []
for x in lines:
    words = x.split()
    
    if OddLine == True:
        Features = []
        for y in words:
            Features = append(Features, float32(atof(y)))
        if len(FeatureMat) == 0:
            FeatureMat = Features
        else:
            FeatureMat = vstack((FeatureMat, Features))
    
    else:
        Labels = []
        for y in words:
            Labels = append(Labels, int64(atoi(y)))
        if len(LabelVec)==0:
            LabelVec = Labels
        else:
            LabelVec = vstack((LabelVec, Labels))
    OddLine = not(OddLine)

FinalTupTrain = (FeatureMat, LabelVec)
print len(FinalTupTrain)
file = open( "features_dump", "wb" )
pickle.dump(FinalTupTrain,  file)
file.close()

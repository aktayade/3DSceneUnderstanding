# This file contains the code to load the Cornell feature data into a format required by the DBN code
# The format needed is a tuple of <matrix array, vector array>
from string import *
from numpy import *
import numpy, time, cPickle, gzip, sys, os
from theano import *
import theano.tensor as T

def load_cornell_data():
	datf = open('../data/cornell/data_split5_train.txt')
	lines = datf.readlines()

	# Loop through each line and perform operations
	OddLine = True
	FirstIter = True
	for x in lines:
		words = x.split()
		if OddLine == True:
			Features = []
			for y in words:            
				Features = append(Features, float32(atof(y)))
				
			if FirstIter == True:
				FeatureMat = Features
			else:
				FeatureMat = vstack((FeatureMat, Features))
		else:
			Labels = []
			for y in words:
				Labels = append(Labels, int64(atoi(y)))
				
			if FirstIter == True:
				LabelVec = Labels
				FirstIter = False
			else:
				LabelVec = append(LabelVec, Labels)
		OddLine = not(OddLine)
					
	FinalTupTrain = (FeatureMat, LabelVec)

	datf = open('../data/cornell/data_split5_test.txt')
	lines = datf.readlines()

	# Loop through each line and perform operations
	OddLine = True
	FirstIter = True
	for x in lines:
		words = x.split()
		if OddLine == True:
			Features = []
			for y in words:            
				Features = append(Features, float32(atof(y)))
				
			if FirstIter == True:
				FeatureMat = Features
			else:
				FeatureMat = vstack((FeatureMat, Features))
		else:
			Labels = []
			for y in words:
				Labels = append(Labels, int64(atoi(y)))
				
			if FirstIter == True:
				LabelVec = Labels
				FirstIter = False
			else:
				LabelVec = append(LabelVec, Labels)
		OddLine = not(OddLine)
					
	FinalTupTest = (FeatureMat, LabelVec)


	datf = open('../data/cornell/data_split5_valid.txt')
	lines = datf.readlines()

	# Loop through each line and perform operations
	OddLine = True
	FirstIter = True
	for x in lines:
		words = x.split()
		if OddLine == True:
			Features = []
			for y in words:            
				Features = append(Features, float32(atof(y)))
				
			if FirstIter == True:
				FeatureMat = Features
			else:
				FeatureMat = vstack((FeatureMat, Features))
		else:
			Labels = []
			for y in words:
				Labels = append(Labels, int64(atoi(y)))
				
			if FirstIter == True:
				LabelVec = Labels
				FirstIter = False
			else:
				LabelVec = append(LabelVec, Labels)
		OddLine = not(OddLine)
					
	FinalTupValid = (FeatureMat, LabelVec)

	def shared_dataset(data_xy):
		 data_x, data_y = data_xy
		 shared_x = theano.shared(numpy.asarray(data_x, dtype=theano.config.floatX))
		 shared_y = theano.shared(numpy.asarray(data_y, dtype=theano.config.floatX))
		 return shared_x, T.cast(shared_y, 'int32')

	test_set_x,  test_set_y  = shared_dataset(FinalTupTest)
	valid_set_x, valid_set_y = shared_dataset(FinalTupValid)
	train_set_x, train_set_y = shared_dataset(FinalTupTrain)

	rval = [(train_set_x, train_set_y), (valid_set_x,valid_set_y), (test_set_x, test_set_y)]
	return rval


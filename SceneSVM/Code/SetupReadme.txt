1. Change the paths in globalpaths.m to match your system
2. The mex files in the Classifiers folder might not be right for your system (Caen machines and win32 are compiled currently).  See the following:
	For gaussian kernel svm (libsvm): Download libsvm
		http://www.csie.ntu.edu.tw/~cjlin/libsvm/
		Extract the zip file, then open matlab and browse to the folder named matlab.  Type 'make'.
		This will compile the mex files for libsvm, move these to the Classifiers folder.
		
	For linear kernel svm (liblinear): Download liblinear
		http://www.csie.ntu.edu.tw/~cjlin/liblinear/
		Do the same thing as for libsvm (see above).
		
	For Cornell's multiclass SVM (based on StructSvm but not the same):
		http://svmlight.joachims.org/svm_multiclass.html
		Follow their instructions for compiling the binaries
		Place these binaries in the Code folder.
		Change the MultiClassPerformance.m file to match the binary file names (on linux no '.exe')
		
3.  To add additional dataset or change how it is loaded, modify Dataset/load_scene_dataset.m
	and modify Dataset.m in the code folder.
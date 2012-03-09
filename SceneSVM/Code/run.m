% This file automates running the current experiment
clear

d = Dataset('scene','office',0);
ExperimentHyperParameterSearch.run(d,'grid', 'multiclasssvm',false);
ExperimentHyperParameterSearch.run(d,'grid', 'linearsvm',false);
ExperimentHyperParameterSearch.run(d,'grid', 'gaussiansvm',false);
d = Dataset('scene','home',0);
ExperimentHyperParameterSearch.run(d,'grid', 'multiclasssvm',false);
ExperimentHyperParameterSearch.run(d,'grid', 'linearsvm',false);
ExperimentHyperParameterSearch.run(d,'grid', 'gaussiansvm',false);
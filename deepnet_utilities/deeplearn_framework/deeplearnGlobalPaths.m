p = genpath('deeplearn_framework');
if isempty(p)
    addpath('minFunc');
else
    addpath(p);
end
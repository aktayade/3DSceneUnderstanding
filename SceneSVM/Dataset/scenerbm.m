function fname_out = scenerbm(ws, num_bases, pbias, bias_mode, batch_size, num_trials)
%rbm_olshausen_v1_sparse (ws, num_bases, pbias, bias_mode, batch_size)
% ws = window size (patch sidelength)
% Note: This is a working version for cleanup

epsilon = 0.001; % 0.005 for 1000 bases

if ~exist('num_trials', 'var'),
    num_trials = 500;
end

rand('state', sum(100*clock));
randn('state', sum(100*clock));

load home.mat;
X = zscore(data(:,4:end))';

fname_prefix = sprintf('results/belief_OLSHAUSEN_V1_sparse_w%02d', ws);
[W vbias_vec hbias_vec pars error_history fname_out] = train_rbm_LB_sparse(X, fname_prefix, 'num_bases', num_bases, 'pbias', pbias, 'bias_mode', bias_mode, 'batch_size', batch_size, 'num_trials', num_trials, 'epsilon', epsilon);
[ferr dW dv dh poshidprobs poshidstates negdata]= fobj_rbm_CD_LB_sparse(X, W, vbias_vec, hbias_vec, pars, 'sample', bias_mode);

[W vbias_vec hbias_vec pars error_history fname_out] = train_rbm_LB_sparse(poshidprobs', fname_prefix, 'num_bases', num_bases, 'pbias', pbias, 'bias_mode', bias_mode, 'batch_size', batch_size, 'num_trials', num_trials, 'epsilon', epsilon);
[ferr dW dv dh poshidprobs poshidstates negdata]= fobj_rbm_CD_LB_sparse(poshidprobs', W, vbias_vec, hbias_vec, pars, 'sample', bias_mode);
data = [data(:,1:3) poshidprobs];
save('home-gaussrbm.mat', 'data');
return

%%

if 0
matlab -nodisplay
cd /afs/cs/u/hllee/visionnew/trunk/belief_nets/code_nips08

scenerbm(14, 200, 0.03, 'hgrad', 100)

end



function [W vbias_vec hbias_vec pars error_history fname_out] = train_rbm_LB_sparse(data, fname_prefix, varargin)

% Essential 
num_bases = 100;
pbias = 0.02;
sigma_start = 0.5;
sigma_stop = 0.5;

CD_mode = 'exp';
bias_mode = 'none';
pbias_lamba = 5;

% Etc parameters
K_CD = 1;
C_sigm = 1;
proj_mat = [];
unproj_mat = [];

% Initialization
W = [];
vbias_vec = [];
hbias_vec = [];
pars = [];

% learning
resample_flag = 1;
num_patches = min(100000, size(data,2)); % number of samples for each iteration
batch_size = 100;
num_trials = 1000;
epsilon = 0.01;

while(length(varargin) > 1)
    opt = varargin{1};
    val = varargin{2};
    varargin = varargin(3:end);

    switch lower(opt)
    case 'num_bases', num_bases = val;
    case 'pbias', pbias = val;
    case 'sigma_start', sigma_start = val;
    case 'sigma_stop', sigma_stop = val;
    case 'cd_mode', CD_mode = val;
    case 'bias_mode', bias_mode = val;
    case 'pbias_lamba', pbias_lamba = val;
    case 'k_cd', K_CD = val;
    case 'c_sigm', C_sigm = val;
    case 'proj_mat', proj_mat = val;
    case 'unproj_mat', unproj_mat = val;
    case 'w', W = val;
    case 'vbias_vec', vbias_vec = val;
    case 'hbias_vec', hbias_vec = val;
    case 'pars', pars = val;
    case 'resample_flag', resample_flag = val;
    case 'num_patches', num_patches = val;
    case 'batch_size', batch_size = val;
    case 'num_trials', num_trials = val;
    case 'epsilon', epsilon = val;
    otherwise,     
        error('Bad options.');
    end
end

if (num_patches > size(data,2))
    fprintf('ERROR: data does not have %d samples!\n',num_patches);
    return;
else % pick num_patches samples from data X
    X = data(:,randsample(size(data,2),num_patches));
end
num_vis = size(X,1);

% Fixed parameters
K_SAVEON = 1; % save results on every K_SAVEON epochs
K_RESAMPLEON = 1; % resample data on every K_RESAMPLEON epochs (if resample_flag = 1)

% Initialize variables
if ~exist('pars', 'var') || isempty(pars)
    pars=[];
end

if ~isfield(pars, 'num_bases'), pars.num_bases = num_bases; end
if ~isfield(pars, 'pbias'), pars.pbias = pbias; end
if ~isfield(pars, 'std_gaussian'), pars.std_gaussian = sigma_start; end
if ~isfield(pars, 'sigma_stop'), pars.sigma_stop = sigma_stop; end
if ~isfield(pars, 'CD_mode'), pars.CD_mode = CD_mode; end
if ~isfield(pars, 'bias_mode'), pars.bias_mode = bias_mode; end
if ~isfield(pars, 'pbias_lamba'), pars.pbias_lamba = pbias_lamba; end

if ~isfield(pars, 'K_CD'), pars.K_CD = K_CD; end
if ~isfield(pars, 'C_sigm'), pars.C_sigm = C_sigm; end
if ~isfield(pars, 'proj_mat'), pars.proj_mat = proj_mat; end
if ~isfield(pars, 'unproj_mat'), pars.unproj_mat = unproj_mat; end

if ~isfield(pars, 'resample_flag'), pars.resample_flag = resample_flag; end
if ~isfield(pars, 'num_patches'), pars.num_patches = num_patches; end
if ~isfield(pars, 'batch_size'), pars.batch_size = batch_size; end
if ~isfield(pars, 'num_trials'), pars.num_trials = num_trials; end
if ~isfield(pars, 'epsilon'), pars.epsilon = epsilon; end

pars.patch_size = size(data,1);
disp(pars)

%% Initialize weight matrix, vbias_vec, hbias_vec (unless given)
if ~exist('W', 'var') | isempty(W)
    W = 0.1*randn(pars.patch_size, pars.num_bases);
end

if ~exist('vbias_vec', 'var') | isempty(vbias_vec)
    vbias_vec = zeros(num_vis,1);
end

if ~exist('hbias_vec', 'var') | isempty(hbias_vec)
    hbias_vec = zeros(pars.num_bases,1);
end


% format will be
% belief_prefix_#images_numvis_numhidden_p_sigma_biasmode_decaysigma_resamplemode_decaysigmastop_kcd_timestamp.mat
fname_save = sprintf('%s_%dK_v%2d_h%02d_p%g_sigma%g_dss%g_rs%d_Csigm%g_kcd%d_%s_%s_plambda%g_%s', fname_prefix, floor(pars.num_patches/1000), num_vis, num_bases, pbias, pars.std_gaussian, pars.sigma_stop, pars.resample_flag, pars.C_sigm, pars.K_CD, CD_mode, bias_mode, pars.pbias_lamba, datestr(now, 30));
fname_mat  = sprintf('%s.mat', fname_save);
fname_out = fname_mat;
% mkdir(fileparts(fname_save));
fname_out

initialmomentum  = 0.5;
finalmomentum    = 0.9;

error_history = [];
sparsity_history = [];

Winc=0;
vbiasinc=0;
hbiasinc=0;
for t=1:pars.num_trials
    % Take a random permutation of the samples
    indperm = randperm(size(X,2));
    tic;
    ferr_current_iter = [];
    sparsity_curr_iter = [];
    for batch=1:(size(X,2)/pars.batch_size),
        % Show progress in epoch
        if (mod(batch,100) == 0)
            fprintf(1,'epoch %d batch %d\r',t,batch); 
        end
        
        % This is data to use for this step
        batch_idx = indperm((1:pars.batch_size)+pars.batch_size*(batch-1));
        Xb = X(:,batch_idx);

        % update rbm
        [ferr dW dv dh poshidprobs poshidstates negdata]= fobj_rbm_CD_LB_sparse(Xb, W, vbias_vec, hbias_vec, pars, CD_mode, bias_mode);
        ferr_current_iter = [ferr_current_iter, ferr];
        sparsity_curr_iter = [sparsity_curr_iter, mean(poshidprobs(:))];
        
        if t<5,
           momentum = initialmomentum;
        else
            momentum = finalmomentum;
        end

        % update parameters
        Winc = momentum*Winc + epsilon*dW;
        W = W + Winc;

        vbiasinc = momentum*vbiasinc + epsilon*dv;
        vbias_vec = vbias_vec + vbiasinc;

        hbiasinc = momentum*hbiasinc + epsilon*dh;
        hbias_vec = hbias_vec + hbiasinc;
    end
    toc;

    if (pars.std_gaussian > pars.sigma_stop) % stop decaying after some point
        pars.std_gaussian = pars.std_gaussian*0.99;
    end
    
    error_history(t) = mean(ferr_current_iter);
    sparsity_history(t) = mean(sparsity_curr_iter);
    
    fprintf('epoch %d error = %g \tsparsity_hid = %g\n', t, mean(ferr_current_iter), mean(sparsity_curr_iter));
    if 0 %mod(t, K_SAVEON)==0
        save(fname_mat, 'W', 'proj_mat', 'unproj_mat', 'pars', 't', 'vbias_vec', 'hbias_vec', 'error_history', 'sparsity_history');
        disp(sprintf('results saved as %s\n', fname_mat));
    end
    
    %figure(1), clf, display_network(W);

    if (pars.resample_flag && mod(t, K_RESAMPLEON) == 0)
        X = data(:,randsample(size(data,2),num_patches));
    end
end

return



function [ferr dW_total dv_total dh_total poshidprobs poshidstates negdata] = ...
	fobj_rbm_CD_LB_sparse(Xb, W, vbias, hbias, pars, opt_CD_mode, opt_adjbias_mode)

weightcost_l2  = 0.0002;   

numcases = size(Xb,2);
numdims = size(Xb,1);
numhid = size(W,2);

%%%%%%%%% START POSITIVE PHASE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
poshidprobs = 1./(1 + exp(-pars.C_sigm/(pars.std_gaussian^2).*(Xb'*W + repmat(hbias',numcases,1))));    
posprods    = Xb * poshidprobs;
poshidstates = poshidprobs > rand(numcases,numhid);
posvisact = sum(Xb');
poshidact   = sum(poshidprobs);

%%%%%%%%% START NEGATIVE PHASE  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% negdata = 1./(1 + exp(-poshidstates*vishid' - repmat(visbiases,numcases,1))); % Binary cases
neghidstates = poshidstates;
for j=1:pars.K_CD  %% pars.K_CD-step contrastive divergence
    if strcmp(opt_CD_mode, 'sample')
        negdata = pars.C_sigm*(neghidstates*W' + repmat(vbias',numcases,1)) + pars.std_gaussian*randn(numcases, numdims);
    elseif strcmp(opt_CD_mode, 'exp')
        negdata = pars.C_sigm*(neghidstates*W' + repmat(vbias',numcases,1));
    else
        error('wrong CD mode: [sample | exp]');
    end
    neghidprobs = 1./(1 + exp(-pars.C_sigm/(pars.std_gaussian^2).*(negdata*W + repmat(hbias',numcases,1))));    
    neghidstates = neghidprobs > rand(numcases,numhid);
end
negprods  = negdata'*neghidprobs;
negvisact = sum(negdata); 
neghidact = sum(neghidprobs);

ferr = mean(sum( (Xb'-negdata).^2 ));

%%%%%%%%% UPDATE WEIGHTS AND BIASES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
if strcmp(opt_adjbias_mode, 'none')
    dhbias = 0;
    dvbias = 0;
    dW = 0;
elseif strcmp(opt_adjbias_mode, 'simple')
    dhbias = mean(poshidprobs) - pars.pbias;
    dvbias = 0;
    dW = 0;
elseif strcmp(opt_adjbias_mode, 'hgrad')
    dsigm = mean(poshidprobs.*(1-poshidprobs))*2/pars.pbias;
    dhbias = (mean(poshidprobs) - pars.pbias).*dsigm;
    dvbias = 0;
    dW = 0;
elseif strcmp(opt_adjbias_mode, 'Whgrad')
    sigm_mat = poshidprobs.*(1-poshidprobs)*2/pars.pbias;
    dsigm = mean(sigm_mat);

    dhbias = (mean(poshidprobs) - pars.pbias).*dsigm;
    % dW = (repmat((mean(poshidprobs) - pars.pbias)', 1, size(W,1)).*(sigm_mat'*Xb'))./size(Xb, 2);
    dW = (repmat((mean(poshidprobs) - pars.pbias), size(W,1),1).*(Xb*sigm_mat))./size(Xb, 2);
    dvbias = 0;
else
   error('wrong adjust_bias mode!');
end

dW_total = (posprods-negprods)/numcases;
dW_total = dW_total - weightcost_l2*W;
dW_total = dW_total - pars.pbias_lamba*dW;
dv_total = (posvisact-negvisact)/numcases - pars.pbias_lamba*dvbias;
dh_total = (poshidact-neghidact)/numcases - pars.pbias_lamba*dhbias;

dv_total = dv_total';
dh_total = dh_total';

return


function X = getdata_sparsenet(IMAGES, winsize, num_patches)

num_images=size(IMAGES,3);
image_size=size(IMAGES,1);
sz= winsize;
BUFF=4;

totalsamples = 0;
% extract subimages at random from this image to make data vector X
% Step through the images
X= zeros(sz^2, num_patches);
for i=1:num_images,

    % Display progress
    fprintf('[%d/%d]',i,num_images);

    this_image=IMAGES(:,:,i);

    % Determine how many patches to take
    getsample = floor(num_patches/num_images);
    if i==num_images, getsample = num_patches-totalsamples; end

    % Extract patches at random from this image to make data vector X
    for j=1:getsample
        r=BUFF+ceil((image_size-sz-2*BUFF)*rand);
        c=BUFF+ceil((image_size-sz-2*BUFF)*rand);
        totalsamples = totalsamples + 1;
        % X(:,totalsamples)=reshape(this_image(r:r+sz-1,c:c+sz-1),sz^2,1);
        temp =reshape(this_image(r:r+sz-1,c:c+sz-1),sz^2,1);
        X(:,totalsamples) = temp - mean(temp);
    end
end  
fprintf('\n');
return

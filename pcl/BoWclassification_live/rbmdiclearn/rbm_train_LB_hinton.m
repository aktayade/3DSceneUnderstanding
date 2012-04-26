function [W, hbias, vbias, pars, stats] = rbm_train_LB_hinton(X, numhid, pbias, l2reg, pbias_lambda, maxiter, givenLabel, givenCenter, epsilon, gibbsteps, opt_GPU)
% This code tries to estimate sigma 
% The energy function used:
% E(v,h) = 1/2*(v-c).^2 - 1/sigma (v'Wh + b'h)

if ~exist('pbias', 'var')
    pbias = 0.02;
end
if ~exist('pbias_lambda', 'var') 
    pbias_lambda = 10; 
end
if ~exist('epsilon', 'var') 
    epsilon = 0.01;
end
if ~exist('l2reg', 'var') 
    l2reg = 0.001; 
end
if ~exist('gibbsteps', 'var') 
    gibbsteps = 1; 
end
if ~exist('maxiter', 'var') 
    maxiter = 250; 
end

pars.numhid = numhid;
pars.pbias =pbias;
pars.pbias_lambda = pbias_lambda;
pars.epsilon = epsilon;
pars.l2reg = l2reg;
pars.gibbnum = gibbsteps;
% Note: pars.sigma is also a parameter, but it will be "learned".

numdim = size(X,1);
batch_size = 100;

% apply kmeans to estimate the std_sigma
% Ideally, RBM should produce smaller reconstruction error than kmeans
% (otherwise, RBM would be a worse approximation of the data)
% The following kmeans path needs to be fixed..
if exist('./litekmeans/', 'dir')
    addpath ./litekmeans/
end
%opt_approx = false; %true;
%if opt_approx
%    [label,center] = litekmeans(X', min(numhid,500), true, 100);
%else
%    %[label,center] = litekmeans(X', numhid, true, 200);
%    [label,center] = kmeans(X', numhid, 'Replicates', 1, 'EmptyAction', 'singleton'); %200
%end

if exist('givenLabel', 'var') && exist('givenCenter', 'var')
    label = givenLabel;
    center = givenCenter;
else
    [label,center] = kmeans(X', numhid, 'Replicates', 1, 'EmptyAction', 'singleton'); %200
end

center = center'; % now, center= dim*centroids 
sigma_kmeans = sqrt(mean(mean((X- center(:, label)).^2,2)));
% display_network(center)
err_kmeans = sum(mean( (X- center(:, label)).^2, 2));
err_kmeans

sigma = sigma_kmeans;

if pbias*numhid <= 6
    %GMM=load('results_kmeans/sift_gmm_sigmaI_k256_iter10.mat');

    % initialize GMMs
    K = numhid;
    n = size(X,2);
    E = sparse(1:n,label,1,n,K,n)';  % transform label into indicator matrix

    % initial GMM paramters
    p0 = mean(E,2);
    mu = center;
    sigma = sqrt(mean(mean( (X - center(:, label)).^2, 2))); % Using sigma*I in this example
    
    GMM.p0 = p0;
    GMM.mu = mu;
    GMM.sigma = sigma;
    
    c = GMM.mu*GMM.p0; 
    sigmagmm = GMM.sigma;
    Wgmm = bsxfun(@minus, GMM.mu, c)./sigmagmm;

    PE = (1/sigmagmm*(Wgmm'*X));
    bgmm = zeros(length(GMM.p0),1);
    for k=1:length(GMM.p0)
        bgmm(k) = -sigmagmm*quantile(PE(k, :), 1-GMM.p0(k));
    end
end


% initialize weights
W = 0.1*randn(numdim, numhid);
hbias = zeros(numhid, 1);
vbias = zeros(numdim, 1);

Winc = zeros(size(W));
hbiasinc = zeros(size(hbias));
vbiasinc = zeros(size(vbias));


if pbias*numhid <= 6
    fprintf('Initializing the clusters with GMM...');
    W = Wgmm;
    hbias = bgmm;
    sigma = sigmagmm;
    vbias = c;
end

if ~exist('opt_GPU', 'var')
    opt_GPU = false;
end

if opt_GPU
    batch_size = 512;
    maxiter = maxiter*2;

    W = GPUsingle(W); 
    hbias = GPUsingle(hbias); 
    vbias = GPUsingle(vbias); 

    Winc = GPUsingle(Winc); 
    hbiasinc = GPUsingle(hbiasinc); 
    vbiasinc = GPUsingle(vbiasinc); 
end


initialmomentum  = 0.5;
finalmomentum    = 0.9;

error_history = [];
error_for_sigma_history = [];
sparsity_history = [];

%fname_save = sprintf('%s_LB_v%d_h%d_l2reg%g_eps%g_p%g_plambda%g_%s', fname_prefix, numdim, numhid, l2reg, epsilon, pbias, pbias_lambda, datestr(now, 30));
%fname_mat = sprintf('%s.mat', fname_save);
%fname_png = sprintf('%s.png', fname_save);




%%
% train a restricted Boltzmann machine
runningavg_prob = [];
for t=1:maxiter
    tic
    randidx = randperm(size(X,2));
    recon_err_epoch = [];
    recon_err_for_sigma_epoch = 0; %[];
    sparsity_epoch = [];
    
%     if opt_GPU
%         recon_err_for_sigma_epoch = GPUsingle(recon_err_for_sigma_epoch);
%     end
    if opt_GPU
        lfactor = floor((GPUmem/20/size(X,1)/batch_size)/100)*100; % This is sort of arbitrary hack... 
    else
        lfactor = 1;
    end
    lbatch_size = lfactor*batch_size;
    batch_count = 0;
    for k=1:ceil(size(X,2)/lbatch_size)
        if opt_GPU
            XGPU = GPUsingle(X(:, randidx((k-1)*lbatch_size+1:min(k*lbatch_size, size(X,2)))));
        else
            XGPU = X(:, randidx((k-1)*lbatch_size+1:min(k*lbatch_size, size(X,2))));
        end
        
    for b=1:ceil(size(XGPU,2)/batch_size)
        batch_count = batch_count+1;
        % batchidx = randidx((b-1)*batch_size+1:min(b*batch_size, size(XGPU,2)));

        Xb = XGPU(:, (b-1)*batch_size+1:min(b*batch_size, size(XGPU,2)) );

        % compute contrastive divergence steps
        poshidprob = sigmoid(1/sigma*(W'*Xb + repmat(hbias, 1, size(Xb,2))));
        if opt_GPU
            poshidstates = cuRand(size(poshidprob), GPUsingle)< poshidprob;
        else
            poshidstates = rand(size(poshidprob))< poshidprob;
        end

        negdata = sigma*(W*poshidstates) + repmat(vbias, 1, size(Xb,2));
        negdatarecon = sigma*(W*poshidprob) + repmat(vbias, 1, size(Xb,2));
        neghidprob = sigmoid(1/sigma*(W'*negdata + repmat(hbias, 1, size(Xb,2))));
        if gibbsteps > 1
            negdata2 = negdata;
            for n=1:(gibbsteps-1)
                % compute contrastive divergence steps
                %poshidprob = sigmoid(1/sigma*(W'*negdata2 + repmat(hbias, 1, size(Xb,2))));
                if opt_GPU
                    poshidstates = cuRand(size(neghidprob), GPUsingle)< neghidprob;
                else
                    poshidstates = rand(size(neghidprob))< neghidprob;
                end

                negdata2 = sigma*(W*poshidstates) + repmat(vbias, 1, size(Xb,2));
                neghidprob = sigmoid(1/sigma*(W'*negdata2 + repmat(hbias, 1, size(Xb,2))));
            end
        end
        % monitoring variables
        % recon_err = norm(Xb- negdata, 'fro')^2/size(Xb,2); % SE for each column (sum over entire column)
        negdata = negdatarecon;
        recon_err = mean(sum((Xb- negdata).^2)); % SE for each column (sum over entire column)
        recon_err_epoch = [recon_err_epoch recon_err];

        recon_err_for_sigma = sum((Xb- negdata).^2,2)/size(Xb,2); % SE for each coordinate
        % recon_err_for_sigma_epoch = [recon_err_for_sigma_epoch recon_err_for_sigma];
        recon_err_for_sigma_epoch = recon_err_for_sigma_epoch+ recon_err_for_sigma;

        sparsity_epoch = [sparsity_epoch mean(mean(poshidprob))]; 

        % compute contrastive gradients
        dW = (Xb*poshidprob' - negdata*neghidprob');
        dhbias = sum(poshidprob,2) - sum(neghidprob,2);
        dvbias = sum(Xb,2) - sum(negdata, 2);

        % sparsity regularization update
        if isempty(runningavg_prob)
            runningavg_prob = mean(poshidprob,2);
        else
            runningavg_prob = 0.9*runningavg_prob + 0.1*mean(poshidprob,2);
        end
            
        % dhbias_sparsity = pbias_lambda*(pbias - mean(poshidprob,2));
        dhbias_sparsity = pbias_lambda*(pbias - runningavg_prob);

        if t<5,           
            momentum = initialmomentum;
        else
            momentum = finalmomentum;
        end

        % update parameters
        Winc = momentum*Winc + epsilon*(dW/batch_size - l2reg*W);
        W = W + Winc;

        vbiasinc = momentum*vbiasinc + epsilon*dvbias/batch_size;
        vbias = vbias + vbiasinc;

        hbiasinc = momentum*hbiasinc + epsilon*(dhbias/batch_size + dhbias_sparsity);
        hbias = hbias + hbiasinc;
    end
    end

    error_history = [error_history mean(recon_err_epoch)];
    sparsity_history = [sparsity_history mean(sparsity_epoch)];
    
    % error_for_sigma_history = [error_for_sigma_history sqrt(mean(recon_err_for_sigma_epoch(:)))];
    sigma_recon = sqrt(mean(recon_err_for_sigma_epoch/batch_count));
    error_for_sigma_history = [error_for_sigma_history sigma_recon];

    eta_sigma = 0.01;
    sigma = (1-eta_sigma)*sigma + eta_sigma*sigma_recon;
    pars.sigma = sigma;
    fprintf('epoch %d:\t error=%g,\t sparsity=%g,\t error(sigma)=%g,\t sigma(param)=%g\n', t, error_history(t), sparsity_history(t), error_for_sigma_history(t), sigma);
    
    disp(sqrt(sum(sum(W.^2))))

    % figure(1), display_network_nonsquare(W);

    figure(1), hold on
    subplot(2,1,1), plot(error_history); title('reconstruction error')
    subplot(2,1,2), plot(sparsity_history); title('sparsity')

    if mod(t, 1)==0
        %save_from_cpu(fname_mat, W, hbias, vbias, pars, error_history, sparsity_history);
        %fprintf('saved as %s\n', fname_mat);
    end
    
    toc
end

%%

stats = [];
stats.error_history = error_history;
stats.sparsity_history = sparsity_history;

% For the returned weights, drop the need for sigma and transpose to format
% into same format as SparseAutoEncoder code
W = W'/sigma;
hbias = hbias/sigma;

%save_from_cpu(fname_mat, W, hbias, vbias, pars, error_history, sparsity_history);

%figure(3), display_network_nonsquare(single(W));
%saveas(gcf, fname_png);
return

function save_from_cpu(fname_mat, W, hbias, vbias, pars, error_history, sparsity_history)

W = single(W);
hbias = single(hbias);
vbias = single(vbias);
error_history = single(error_history);
sparsity_history = single(sparsity_history);

%save(fname_mat, 'W', 'hbias', 'vbias', 'pars', 'error_history', 'sparsity_history')

return


function [W, hbias, vbias, pars, stats] = rbm_train_BB_hinton(X, numhid, pbias, l2reg, pbias_lambda, maxiter, epsilon, gibbsteps)
% This code tries to estimate sigma 
% The energy function used:
% E(v,h) = - 1/sigma (v'Wh + b'h + c'v)
% sigma values can be typically set to zero
% Note that this is a binary version:
% When turning off sparsity, just use pbias=0, pbias_lambda=0;

% TODO: implement GPU version
% TODO: implement CD-k
% TODO: implement persistent CD
% TODO: Check if the implementation is consistent with CD learning?

if ~exist('pbias', 'var') 
    pbias = 0.02; end
if ~exist('pbias_lambda', 'var') 
    pbias_lambda = 10; end
if ~exist('epsilon', 'var') 
    epsilon = 0.01; end
if ~exist('l2reg', 'var') 
    l2reg = 0.001; end
if ~exist('gibbsteps', 'var') 
    gibbsteps = 1; end
if ~exist('maxiter', 'var') 
    maxiter = 250; 
end

% In case of turning sparsity off, set pbias to 0 as well. 
if pbias_lambda ==0
    pbias=0;
end

pars.numhid = numhid;
pars.pbias =pbias;
pars.pbias_lambda = pbias_lambda;
pars.epsilon = epsilon;
pars.l2reg = l2reg;
pars.gibbnum = gibbsteps;

sigma = 1;
pars.sigma = sigma;


numdim = size(X,1);
batch_size = 100;


% initialize weights
W = 0.1*randn(numdim, numhid);
hbias = zeros(numhid, 1);
vbias = zeros(numdim, 1);

Winc = zeros(size(W));
hbiasinc = zeros(size(hbias));
vbiasinc = zeros(size(vbias));

initialmomentum  = 0.5;
finalmomentum    = 0.9;

error_history = [];
sparsity_history = [];

%fname_save = sprintf('%s_BB_v%d_h%d_l2reg%g_eps%g_p%g_plambda%g_%s', fname_prefix, numdim, numhid, l2reg, epsilon, pbias, pbias_lambda, datestr(now, 30));
%fname_mat = sprintf('%s.mat', fname_save);
%fname_png = sprintf('%s.png', fname_save);

% train a restricted Boltzmann machine
for t=1:maxiter
    tic
    randidx = randperm(size(X,2));
    recon_err_epoch = [];
    sparsity_epoch = [];
    for b=1:ceil(size(X,2)/batch_size)
        batchidx = randidx((b-1)*batch_size+1:min(b*batch_size, size(X,2)));
        
        Xb = X(:, batchidx);
        
        % compute contrastive divergence steps
        poshidprob = sigmoid(1/sigma*(W'*Xb + repmat(hbias, 1, size(Xb,2))));
        poshidstates = rand(size(poshidprob))< poshidprob;

        negdata = sigmoid(1/sigma*(W*poshidstates + repmat(vbias, 1, size(Xb,2))));
        neghidprob = sigmoid(1/sigma*(W'*negdata + repmat(hbias, 1, size(Xb,2))));
        if gibbsteps > 1
            negdata2 = negdata;
            for n=1:(gibbsteps-1)
                % compute contrastive divergence steps
                %poshidprob = sigmoid(1/sigma*(W'*negdata2 + repmat(hbias, 1, size(Xb,2))));
                poshidstates = rand(size(neghidprob))< neghidprob;

                negdata2 = sigmoid(1/sigma*(W*poshidstates) + repmat(vbias, 1, size(Xb,2)));
                neghidprob = sigmoid(1/sigma*(W'*negdata2 + repmat(hbias, 1, size(Xb,2))));
            end
        end
        % monitoring variables
        recon_err = norm(Xb- negdata, 'fro')^2/size(Xb,2);
        recon_err_epoch = [recon_err_epoch recon_err];
        
        sparsity_epoch = [sparsity_epoch mean(mean(poshidprob))];
        
        % compute contrastive gradients
        dW = (Xb*poshidprob' - negdata*neghidprob');
        dhbias = sum(poshidprob,2) - sum(neghidprob,2);
        dvbias = sum(Xb,2) - sum(negdata, 2);
        
		% sparsity regularization update
        dhbias_sparsity = pbias_lambda*(pbias - mean(poshidprob,2));

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

    error_history = [error_history mean(recon_err_epoch)];
    sparsity_history = [sparsity_history mean(sparsity_epoch)];
    
    fprintf('epoch %d:\t error=%g,\t sparsity=%g\n', t, error_history(t), sparsity_history(t));

    % figure(1), display_network_nonsquare(W);

    figure(1), hold on
    subplot(2,1,1), plot(error_history); title('reconstruction error')
    subplot(2,1,2), plot(sparsity_history); title('sparsity')

    %save(fname_mat, 'W', 'hbias', 'vbias', 'pars', 'error_history', 'sparsity_history');
    %fprintf('saved as %s\n', fname_mat);
    toc
end

stats = [];
stats.error_history = error_history;
stats.sparsity_history = sparsity_history;

% For the returned weights, drop the need for sigma and transpose to format
% into same format as SparseAutoEncoder code
W = W'/sigma;
hbias = hbias/sigma;

%figure(3), display_network_nonsquare(W);
%save(fname_mat, 'W', 'hbias', 'vbias', 'pars', 'error_history', 'sparsity_history');
%saveas(gcf, fname_png);



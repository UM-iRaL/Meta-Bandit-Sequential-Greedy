function x = discretesample_nx(p, n)
% Samples from a discrete distribution
%
%   x = discretesample(p, n)
%       independently draws n samples (with replacement) from the 
%       distribution specified by p, where p is a probability array 
%       whose elements sum to 1.
%
%       Suppose the sample space comprises K distinct objects, then
%       p should be an array with K elements. In the output, x(i) = k
%       means that the k-th object is drawn at the i-th trial.
%       
%   Remarks
%   -------
%       - This function is mainly for efficient sampling in non-uniform 
%         distribution, which can be either parametric or non-parametric.         
%
%       - The function is implemented based on histc, which has been 
%         highly optimized by mathworks. The basic idea is to divide
%         the range [0, 1] into K bins, with the length of each bin 
%         proportional to the probability mass. And then, n values are
%         drawn from a uniform distribution in [0, 1], and the bins that
%         these values fall into are picked as results.
%
%       - This function can also be employed for continuous distribution
%         in 1D/2D dimensional space, where the distribution can be
%         effectively discretized.
%
%       - This function can also be useful for sampling from distributions
%         which can be considered as weighted sum of "modes". 
%         In this type of applications, you can first randomly choose 
%         a mode, and then sample from that mode. The process of choosing
%         a mode according to the weights can be accomplished with this
%         function.
%
%   Examples
%   --------
%       % sample from a uniform distribution for K objects.
%       p = ones(1, K) / K;
%       x = discretesample(p, n);
%
%       % sample from a non-uniform distribution given by user
%       x = discretesample([0.6 0.3 0.1], n);
%
%       % sample from a parametric discrete distribution with
%       % probability mass function given by f.
%       p = f(1:K);
%       x = discretesample(p, n);
%
%
%   Created by Dahua Lin, On Oct 27, 2008
%
%   Modified by Nikolay Atanasov, Jan 2014 to take matrix inputs
%
% INPUT:
%   p = num_dist x num_dim = contains num_dist distributions each with
%                            num_dim atoms
% OUTPUT:
%   x = num_dist x n
%

if nargin < 2
    n = 1;
end

num_dist = size(p,1);

if(num_dist == 1)
    if( n == 1)
        [~,x] = histc(rand, cumsum([0,p]) ); 
    else
        x = discretesample(p, n);
    end
    return;
end

% parse and verify input arguments
if( any( abs(sum(p,2) - 1) > 1e-6 ) )
    error('[discretesample_nx] Malformed probability distribution');
end


% construct the bins
edges = [zeros(num_dist,1), cumsum(p,2)];
s = edges(:,end);
toCorrect = abs(s - 1) > eps;
edges(toCorrect,:) = bsxfun(@rdivide,edges(toCorrect,:),s(toCorrect));
edges(:,end) = 1;

% draw bins
rv = rand(num_dist, n);
offsetv = 0:(num_dist-1);
rva = bsxfun(@plus,rv.',offsetv);
eda = bsxfun(@plus,edges.',offsetv);
c = histc(rva(:),eda(:));
c = transpose(reshape(c,[],num_dist));  % num_dist x size(p,2)
ce = c(:,end);
c = c(:,1:end-1);
c(:,end) = c(:,end) + ce;


% extract samples
[xv,~,cval] = find(transpose(c));
x = reshape(rude(cval,xv),n,num_dist).';


% randomly permute the sample's order
x = x(:,randperm(n));

end
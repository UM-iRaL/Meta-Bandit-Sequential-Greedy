function [match,mindist,dMat] = KNNsearch2D_nx(target,base,K)
% compute the minimum distance for each of the base points to the target
% points
%
% target = num_tpts x 2
% base = num_bpts x 2
% match = num_bpts x 1
% mindist = num_bpts x 1

if nargin < 3
    K = 1;
end

xDiff = bsxfun(@minus,target(:,1),transpose(base(:,1)));
yDiff = bsxfun(@minus,target(:,2),transpose(base(:,2)));

dMat = xDiff.^2 + yDiff.^2; % num_tpts x num_bpts

if K == 1
    [mindist,match] = min(dMat,[],1);
    match = transpose(match);
    mindist = transpose(sqrt(mindist));
else
    error('not implemented yet');
end

end

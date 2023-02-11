function [mapInd, insideMap]=sub2ind_map(siz, sub)
% siz = matrix size
% sub = num_ind x dim
%
insideMap = (sub(:,1) >= 1) & (sub(:,1) <= siz(1));

num_dim = length(siz);
for t = 2:num_dim
    insideMap = insideMap & (sub(:,t) >= 1) & (sub(:,t) <= siz(t));
end
temp = subv2ind(siz,sub(insideMap,:)); % num_valid_ind x 1

mapInd = zeros(size(sub,1),1);
mapInd(insideMap) = temp;


% [mapInd, insideMap]=sub2ind_map(x,y,sizex,sizey)
% insideMap = (x > 1) & (y > 1) & (x < sizex) & (y < sizey);
% temp = sub2ind_nx([sizex,sizey],x(insideMap),y(insideMap));
% mapInd = zeros(size(x));
% mapInd(insideMap) = temp;
end



function index = subv2ind(siz,sub)
%SUBV2IND   Linear index from subscript vector.
% SUBV2IND(SIZ,SUB) returns an equivalent single index corresponding to a
% subscript vector for an array of size SIZ.
% If SUB is a matrix, with subscript vectors as rows, then the result is a 
% column vector.
%
% This is the opposite of IND2SUBV, so that
%   SUBV2IND(SIZ,IND2SUBV(SIZ,IND)) == IND.
%
% See also IND2SUBV, SUB2IND.

% Written by Tom Minka
% Part of Tom Minka's lightspeed package.
% (c) Microsoft Corporation. All rights reserved.


prev_cum_size = [1, cumprod(siz(1:end-1))];
%index = (sub-1)*prev_cum_size' + 1;
index = sub*prev_cum_size' - sum(prev_cum_size) + 1;

end
function c = sample_class_om(y,ConfMat,ns)
%
% sample class observation model
% INPUT:
%   y = num_y x 1 = true objects classes in [1,...,num_class]
%
%   ConfMat = num_class x num_class = hypothesized x true
%
%   ns = number of samples per y
%
% OUTPUT:
%   c = num_y x ns = guessed object classes in [1,...,num_class]
% 
%
if(nargin < 3)
    ns = 1;
end

y = y(:);
% if(size(y,2) ~= 1)
%     error('[sample_class_om] size(y) should be [num_y x 1]\n');
% end

p = transpose(ConfMat(:,y));
c = discretesample_nx(p,ns);


% num_y = length(y);
% c = zeros(num_y,ns);
% for k = 1:num_y
%     c(k,:) = discretesample(model_params.Cmat(:,y(k)),ns);
% end

end
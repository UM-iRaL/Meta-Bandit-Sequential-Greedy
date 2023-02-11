function d = detectable2D(y,x,r_sense,fov,flag)
% returns the true/false chance of detecting (yx,yy) from (xx,xy,xt)
%
%   y = num_y x 2 = 2D positions
%   x = num_x x 3 = 2D poses
%
%
%   flag = [true] = if true returns the detectability of every y 
%                   versus every x 
%                   else assumes num_x = num_y (or one is scalar)
%                   and returns the the probability of detection for the 
%                   matching points
%
% OUTPUT:
%   d in {0, 1}    = num_y x num_x            = if flag == true
%                  = max(num_x,num_y) x 1     = if flag == false
%
% Author:
%       Nikolay A. Atanasov
%       University of Pennsylvania
%

if(nargin < 5)
    flag = false;
end

num_y = size(y,1);
num_x = size(x,1);

if(flag)
    d = false(num_y,num_x);
    if( num_y == 1 && num_x == 1)
        r = norm(y(1:2)-x(1:2));
        if( r <= r_sense )
            b = bearing_nx(x(1),x(2),y(1),y(2));
            d = abs(restrict_angle(b-x(3))) <= fov/2;
        end
    elseif( num_x < num_y )
        idx = custom_rangesearch_(y(:,1:2),x(:,1:2),r_sense);
        %idx = rangesearch(y(:,1:2),x(:,1:2),r_sense);
        %idx is a num_x x 1 cell giving for each x which y's should be
        %considered
        
        for k = 1:num_x
            b = bearing_nx(x(k,1),x(k,2),y(idx{k},1),y(idx{k},2));
            
            valid = abs(restrict_angle(b-x(k,3))) <= fov/2;
            valIdx = idx{k}(valid);
            d(valIdx,k) = true;
        end
    else
        idx = custom_rangesearch_(x(:,1:2),y(:,1:2),r_sense);
        %idx = rangesearch(x(:,1:2),y(:,1:2),r_sense);
        %idx is a num_y x 1 cell giving for each y which x's should be
        %considered
        
        for k = 1:num_y
            b = bearing_nx(x(idx{k},1),x(idx{k},2),y(k,1),y(k,2));
            valid = abs(restrict_angle(b-x(idx{k},3))) <= fov/2;
            valIdx = idx{k}(valid);
            d(k,valIdx) = true;
        end
    end
else
    d = false(max(num_y,num_x),1);
    if( num_x == num_y)
        r = sqrt(sum((y(:,1:2)-x(:,1:2)).^2,2));
        toCompute = r <= r_sense;
        
        if(any(toCompute))
            b = bearing_nx(x(toCompute,1),x(toCompute,2),y(toCompute,1),y(toCompute,2));
            valid = abs(restrict_angle(b-x(toCompute,3))) <= fov/2;
            
            toComputeIdx = find(toCompute);
            d(toComputeIdx(valid)) = true;
        end
    elseif( num_x == 1 )
        r = sqrt(sum((bsxfun(@minus,y(:,1:2),x(1:2))).^2,2));
        toCompute = r <= r_sense;
        
        if(any(toCompute))
            b = bearing_nx(x(1),x(2),y(toCompute,1),y(toCompute,2));
            valid = abs(restrict_angle(b-x(3))) <= fov/2;
            toComputeIdx = find(toCompute);
            
            d(toComputeIdx(valid)) = true;
        end
    elseif( num_y == 1)
        r = sqrt(sum((bsxfun(@minus,y(1:2),x(:,1:2))).^2,2));
        toCompute = r <= r_sense;
        
        if( any(toCompute) )
            b = bearing_nx(x(toCompute,1),x(toCompute,2),y(1),y(2));
            valid = abs(restrict_angle(b-x(toCompute,3))) <= fov/2;
            toComputeIdx = find(toCompute);
            d(toComputeIdx(valid)) = true;
        end
    else
        error('[detectable2D] Dimension mismatch!');
    end
end


end


function idx = custom_rangesearch_(y,x,rad)
%
% @INPUT:
%   y = num_y x num_dim
%   x = num_x x num_dim
%   rad = radius of range search
%
% @COMMENT: For speed provide num_x < num_y
%
% @OUTPUT:
%   idx = num_x x 1 cell = giving for each x which y's should be considered
%
[num_x,num_dim] = size(x);

y = reshape(y,[],1,num_dim);
x = reshape(x,1,[],num_dim);

dist = sqrt( sum(  bsxfun(@minus,y,x).^2, 3 ) ); % num_y x num_x
valid = dist <= rad; % num_y x num_x
idx = cell(num_x,1);
for k = 1:num_x
    idx{k} = find(valid(:,k));
end

end




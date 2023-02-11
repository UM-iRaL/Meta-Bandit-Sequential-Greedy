function pd = pd_gauss2D(y,x,xt,param,flag)
% returns the probability of detecting (yx,yy,yc) from (xx,xy,xt)
%
%   y = num_y x num_dim = position
%   x = num_x x num_dim = position
%   xt = num_x x 1 = orientation
%
%   param.r_sense;
%   param.fov;
%   param.fpd = probability of detection FUNCTION HANDLE
%       INPUT: dist in [0,r_sense] and class in {1,...,num_class}
%       OUTPUT: probability of detection at distance dist
%
%       e.g. param.fpd = @(d,c) p0(c) .* exp(-abs(m0(c)-d)./v0(c));
%
%   flag = [true] = if true returns the probability of detecting every y 
%                   versus every x 
%                   else assumes num_x = num_y (or one is scalar)
%                   and returns the the probability of detection for the 
%                   matching points
%
% OUTPUT:
%   pd in [0, 1]    = num_y x num_x            = if flag == true
%                   = max(num_x,num_y) x 1     = if flag == false
%
if(nargin < 5)
    flag = false;
end

if(nargin<4)
    error('[pd_gauss2D]: param.r_sense, param.fov, param.m0, param.v0, param.p0 are required!'); 
    %m0 = params.m0;
    %v0 = params.v0; % VARIANCE = std^2
    %p0 = params.p0;
end

num_y = size(y,1);
num_x = size(x,1);
% xt = zeros(num_x,1);



if(flag)
    pd = zeros(num_y,num_x);
    if( num_y == 1 && num_x == 1)
        r = norm(y(1:2)-x(1:2));
        
        if( r <= param.r_sense )
            b = bearing_nx(x(1),x(2),y(1),y(2));
            valid = abs(restrict_angle(b-xt)) <= param.fov/2;
            if(valid)
                pd = param.fpd(r,y);
                
                %if(numel(p0) == 1)
                %    pd = p0*exp(-abs(m0-r)/v0);
                %else
                %    pd = p0(y(3))*exp(-abs(m0-r)/v0);
                %end
            end
        end
    elseif( num_x < num_y )
        [idx, dist] = rangesearch(y(:,1:2),x(:,1:2),param.r_sense);
        %idx is a num_x x 1 cell giving for each x which y's should be
        %considered
        
        for k = 1:num_x
            b = bearing_nx(x(k,1),x(k,2),y(idx{k},1),y(idx{k},2));
            valid = abs(restrict_angle(b-xt(k))) <= param.fov/2;
            
            valIdx = idx{k}(valid);
            if(~isempty(valIdx))
                dist_vals = dist{k}(valid);
                pd(valIdx,k) = param.fpd(dist_vals(:), y(valIdx,3));
                
                %{
                if(numel(p0) == 1)
                    pd(valIdx,k) = p0*exp(-abs(m0-dist_vals(:))/v0);
                else
                    pd(valIdx,k) = p0(y(valIdx,3)).*exp(-abs(m0-dist_vals(:))/v0);
                end
                %}
            end
        end
        
    else
        [idx, dist] = rangesearch(x(:,1:2),y(:,1:2),param.r_sense);
        %idx is a num_y x 1 cell giving for each y which x's should be
        %considered
        
        for k = 1:num_y
            b = bearing_nx(x(idx{k},1),x(idx{k},2),y(k,1),y(k,2));
            valid = abs(restrict_angle(b-xt(idx{k}))) <= param.fov/2;
            
            if( any(valid) )
                pd(k,idx{k}(valid)) = param.fpd(dist{k}(valid), y(k,3));
            end
            
            %{
            if(numel(p0) == 1)
                pd(k,idx{k}(valid)) = p0*exp(-abs(m0-dist{k}(valid))/v0);
            else
                pd(k,idx{k}(valid)) = p0(y(k,3))*exp(-abs(m0-dist{k}(valid))/v0);
            end
            %}
        end
        
    end
else
    
    if( num_x == num_y)
        r = sqrt(sum((y(:,1:2)-x(:,1:2)).^2,2));
        pd = zeros(size(r));
        toCompute = r <= param.r_sense;
        
        if(any(toCompute))
            b = bearing_nx(x(toCompute,1),x(toCompute,2),y(toCompute,1),y(toCompute,2));
            valid = abs(restrict_angle(b-xt(toCompute))) <= param.fov/2;
            toComputeIdx = find(toCompute);
            
            valIdx = toComputeIdx(valid);
            if(~isempty(valIdx))
                pd(valIdx) = param.fpd(r(valIdx), y(valIdx,3));
                
                %{
                if(numel(p0) == 1)
                    pd(valIdx) = p0*exp(-abs(m0-r(valIdx))/v0);
                else
                    pd(valIdx) = p0(y(valIdx,3)).*exp(-abs(m0-r(valIdx))/v0);
                end
                %}
            end
        end
    elseif( num_x == 1 )
        r = sqrt(sum((bsxfun(@minus,y(:,1:2),x(1:2))).^2,2));
        pd = zeros(num_y,1);
        toCompute = r <= param.r_sense;
        
        if(any(toCompute))
            b = bearing_nx(x(1),x(2),y(toCompute,1),y(toCompute,2));
            valid = abs(restrict_angle(b-xt)) <= param.fov/2;
            toComputeIdx = find(toCompute);
            
            valIdx = toComputeIdx(valid);
            if(~isempty(valIdx))
                pd(valIdx) = param.fpd(r(valIdx), y(valIdx,3));
                
                %{
                if(numel(p0) == 1)
                    pd(valIdx) = p0*exp(-abs(m0-r(valIdx))/v0);
                else
                    pd(valIdx) = p0(y(valIdx,3)).*exp(-abs(m0-r(valIdx))/v0);
                end
                %}
            end
        end
    elseif( num_y == 1)
        r = sqrt(sum((bsxfun(@minus,y(1:2),x(:,1:2))).^2,2));
        pd = zeros(num_x,1);
        toCompute = r <= param.r_sense;
        
        if( any(toCompute) )
            b = bearing_nx(x(toCompute,1),x(toCompute,2),y(1),y(2));
            valid = abs(restrict_angle(b-xt(toCompute))) <= param.fov/2;
            toComputeIdx = find(toCompute);
            valIdx = toComputeIdx(valid);
            if(~isempty(valIdx))
                pd(valIdx) = param.fpd(r(valIdx), y(3));
                
                %{
                if(numel(p0) == 1)
                    pd(valIdx) = p0*exp(-abs(m0-r(valIdx))/v0);
                else
                    pd(valIdx) = p0(y(3))*exp(-abs(m0-r(valIdx))/v0);
                end
                %}
            end
        end
    else
        error('[pd_gauss2D] Dimension mismatch!');
    end
end


end

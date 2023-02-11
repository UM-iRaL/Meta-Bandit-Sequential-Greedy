function Z = generate_msrmnt(y,x,params,MAP)
%
%  y = num_y x 3 = (x,y,id)
%  x = (x,y,t)
%
%  params.miss_det = true/false = miss detections
%  params.clutter_rate = mean of poisson clutter distribution
%  params.da_known = true/false = known data association
%  params.r_sense
%  params.fov;
%  params.b_sigma;
%  params.r_sigma;
%  params.p0
%  params.v0
%  params.m0
%  params.Cmat
%  params.pc
%  params.ps
%  params.pscymat
%
%  z = is a RFS of (range, bearing, class, score, data association) measurements
%    = in the ROBOT FRAME!
%    = data association is returned only if it is known 
%

if(~exist('params','var'))
    error('p_d_gauss2D:missingInput', ['The following are required\n'...
          'params.miss_det\n'...
          'params.clutter_rate\n'...
          'params.da_known\n'...
          'params.r_sense\n'...
          'params.fov\n'...
          'params.b_sigma\n'...
          'params.r_sigma\n'...
          'params.p0\n'...
          'params.v0\n'...
          'params.m0\n'...
          'params.Cmat\n'...
          'params.pc\n'...
          'params.ps\n'...
          'params.pscymat']); 
end

% determine which landmarks Y are within the fov of x
pd = pd_gauss2D(y,[x(1),x(2)],x(3),params);  % num_y x 1

detectable = pd > 0;

if( nargin > 3 )   % usually we have only 3 inputs
    num_detectable = sum(detectable);
    
    if( num_detectable > 0 )
        y_detectable = y(detectable,:);
        [~,v] = line_of_sight_mex(x(1),x(2),y_detectable(:,1),y_detectable(:,2),...
            MAP.map, MAP.min, MAP.res, MAP.obst_thresh);
        tmp = pd(detectable);
        tmp(~v) = 0;
        pd(detectable) = tmp;
        detectable(detectable) = v;
    end
end

num_detectable = sum(detectable);
y_detectable = y(detectable,:);
pd_detectable = pd(detectable);

if(params.miss_det)
    detected = false(num_detectable,1);
    for k = 1:num_detectable
        detected(k) = logical(discretesample([1-pd_detectable(k),pd_detectable(k)],1)-1);
        %detected(k) = true;
    end
else
    detected = true(num_detectable,1);
end

y_detected = y_detectable(detected,:);

if(isempty(y_detected))
    Z = zeros(0,4);
else
    zr = sample_range_om(y_detected,[x(1),x(2),x(3)],params.r_sigma);
    zb = sample_bearing_om(y_detected,[x(1),x(2),x(3)],params.b_sigma);
    zb = restrict_angle(zb - x(3));     % convert to robot frame
    %zc = sample_class_om(y_detected(:,3),params.Cmat);
    %zs = sample_score_om(zc,y_detected(:,3),params.pscymat);
    
    Z = [zr(:), zb(:)];
    %Z = [zr(:),zb(:)];
end

% Add clutter
if( params.clutter_rate > 0 )
    num_clut = poissrnd(params.clutter_rate);
    
    if (num_clut > 0)
        z_clut = sample_clutter_om(num_clut,params.r_sense,params.fov,params.pc,params.ps);
        % Convert clutter to world frame
        %z_clut(:,2) = restrict_angle(z_clut(:,2)+x(3));
        Z = [Z;z_clut];
    end
else
    num_clut = 0;
end

% add data association to measurement (if known)
if(params.da_known)
    y_idx = find(detectable);
    Z = [Z,[y_idx(detected);zeros(num_clut,1)]];
else
    %randomize
    numz = num_clut + sum(detected);
    Z = Z(randperm(numz),:);
end

end



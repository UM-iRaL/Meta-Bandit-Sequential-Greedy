function H = rb_jacobian_y(xx,xy,yx,yy,flag)
% H = rb_jacobian_y(xx,xy,yx,yy,flag)
%
% returns the range-bearing jacobian of (yx,yy) wrt to (xx,xy)
%
%   xx = num_x x 1
%   xy = num_x x 1
%   yx = num_y x 1
%   yy = num_y x 1
%   flag = if true returns the bearing of every y versus every x
%          else assumes num_x = num_y and returns the bearings for the
%          matching points
%
% OUTPUT:
%   H = 2 x 2 x num_y x num_x = if flag == true
%     = 2 x 2 x max(num_x,num_y)         = if flag == false
%

if(nargin < 5)
    flag = false;
end


min_range = 0.0001;
num_x = size(xx,1);
num_y = size(yx,1);



if(flag)
    r1 = bsxfun(@minus,yx,xx.');        %num_y x num_x
    r2 = bsxfun(@minus,yy,xy.');    
else
    r1 = yx - xx;
    r2 = yy - xy;    
end


% adjust for target being too close!
small_idx = (abs(r1) < min_range & abs(r2) < min_range);
r1(small_idx) = sign(r1(small_idx))*min_range;
r2(small_idx) = sign(r2(small_idx))*min_range;

zero_idx1 = r1 == 0;
zero_idx2 = r2 == 0;
r1(zero_idx1) = min_range;
r2(zero_idx2) = min_range;

r_up_2 = r1.^2 + r2.^2;
r = sqrt(r_up_2);
    

if(flag)
    H = zeros(2,2,num_y,num_x);
    H(1,1,:,:) = r1./r;
    H(1,2,:,:) = r2./r;
    H(2,1,:,:) = -r2./r_up_2;
    H(2,2,:,:) = r1./r_up_2;
else
    if( num_x == 1 && num_y == 1)
        H = [[r1, r2]/r; [-r2 r1]/r_up_2];
    else
        H = zeros(2,2,max(num_y,num_x));
        H(1,1,:) = r1./r;
        H(1,2,:) = r2./r;
        H(2,1,:) = -r2./r_up_2;
        H(2,2,:) = r1./r_up_2;
    end
end




end
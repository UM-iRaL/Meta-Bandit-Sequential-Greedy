function b = bearing_nx(xx,xy,yx,yy,flag)
% b = bearing(xx,xy,yx,yy,flag)
%
% returns the bearing of (yx,yy) wrt to (xx,xy)
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
%   b in [-pi, pi] = num_y x num_x = if flag == true
%                  = num_x x 1 = if flag == false
%

if(nargin < 5)
    flag = false;
end

if(flag)
    b = atan2(bsxfun(@minus,yy,xy.'), bsxfun(@minus,yx,xx.'));
else
    b = atan2(yy-xy,yx-xx);
end

end

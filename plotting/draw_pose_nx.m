function h = draw_pose_nx(h,x,clr,size)
% h = draw_pose_nx(h,x,clr,size)
%
% x = [x,y,yaw] x num_x
% 

if(nargin < 4)
    size = 1;
end
if(nargin < 3)
    clr = 'g';
end
if(nargin < 2)
    h = [];
end

xvals = [x(1,:) + size*cos(x(3,:));x(1,:) + size*cos(x(3,:)+2.7);x(1,:) + size*cos(x(3,:)-2.7)];
yvals = [x(2,:) + size*sin(x(3,:));x(2,:) + size*sin(x(3,:)+2.7);x(2,:) + size*sin(x(3,:)-2.7)];

if isempty(h)
    h = patch(xvals,yvals,clr);
else
    %h = patch(xvals,yvals,clr);
    set(h,'xdata',xvals,'ydata',yvals);
end
end
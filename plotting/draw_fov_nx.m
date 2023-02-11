function h = draw_fov_nx(h,x,fov,range, clr)
%
% x = 3 x 1
% h = draw_fov_nx(h,x,fov,range)
% 
%

dx = 0.01;
da = 0.05;
a = -fov/2:da:fov/2;
b = 0:dx:1;


lin1_x = x(1) + b*range*cos(x(3)+a(1));
lin1_y = x(2) + b*range*sin(x(3)+a(1));
cir_x = x(1) + range*cos(x(3)+a);
cir_y = x(2) + range*sin(x(3)+a);
lin2_x = x(1) + b*range*cos(x(3)+a(end));
lin2_y = x(2) + b*range*sin(x(3)+a(end));

pts = [lin1_x(:), lin1_y(:);
       cir_x(:),  cir_y(:);
       lin2_x(:), lin2_y(:)];

if(isempty(h))
    if nargin > 4
        h = plot(pts(:,1),pts(:,2),strcat(clr, '.'), 'LineWidth',0.1);
    else
        h = plot(pts(:,1),pts(:,2),'r.','LineWidth',0.1);
    end
else
    set(h,'xdata',pts(:,1),'ydata',pts(:,2),'LineWidth',0.02);
end
end
function h = drawEnv(y,siz_obj)
% 
% Plots the landmarks contained in y
%
% y = num_y x 3 = (x,y,obj_class)
%
%   obj_class in [1,2,3,4,5]
%   1 = square
%   2 = circle
%   3 = triangle
%   4 = diamond
%   5 = hexagram
%
% h = array of handles to the objects
%

if(nargin<2)
    siz_obj = 0.5;
end

num_obj = size(y,1);
h = zeros(num_obj,1);


hold_state = ishold(gca);
if(~hold_state)
    hold on;
end

for k = 1:num_obj
    
    switch( y(k,3) )
        case 1  % square
%             h(k) = rectangle('Position',[y(k,1)+3.8*siz_obj*cos(deg2rad(-120)),...
%                                   y(k,2)+3.8*siz_obj*sin(deg2rad(-120)),...
%                                   3.8*siz_obj,3.8*siz_obj],'FaceColor','k');
            
            
            h(k) = rectangle('Position',[y(k,1)+siz_obj/2*cos(deg2rad(-120)),...
                                  y(k,2)+siz_obj/2*sin(deg2rad(-120)),...
                                  siz_obj/2,siz_obj/2],'FaceColor','y');
        case 2  % circle
            h(k) = rectangle('Curvature', [1 1], ...
                'Position',[y(k,1)+siz_obj*1.5*cos(deg2rad(-120)),...
                            y(k,2)+siz_obj*1.5*sin(deg2rad(-120)),...
                            siz_obj*1.5,siz_obj*1.5], 'FaceColor','c');
        case 3  % triangle
            h(k) = rectangle('Position',[y(k,1)+siz_obj/2*cos(deg2rad(-120)),...
                                  y(k,2)+siz_obj/2*sin(deg2rad(-120)),...
                                  siz_obj/2,siz_obj/2],'FaceColor','b');           
%             h(k) = patch([y(k,1)+siz_obj*cos(deg2rad(-60));
%                    y(k,1)+siz_obj*cos(deg2rad(-120));
%                    y(k,1)+siz_obj*cos(deg2rad(90))], ...
%                   [y(k,2)+siz_obj*sin(deg2rad(-60));
%                    y(k,2)+siz_obj*sin(deg2rad(-120));
%                    y(k,2)+siz_obj*sin(deg2rad(90))],'b');
        case 4
            h(k) = plot(y(k,1),y(k,2),'kd','MarkerSize',8,'LineWidth',3);
        case 5
            h(k) = plot(y(k,1),y(k,2),'mh','MarkerSize',7,'LineWidth',3);
        case 6
            h(k) = rectangle('Position',[y(k,1)+siz_obj/2*cos(deg2rad(-120)),...
                                  y(k,2)+siz_obj/2*sin(deg2rad(-120)),...
                                  siz_obj/2,siz_obj/2],'FaceColor','k');
        otherwise
            h(k) = plot(y(k,1),y(k,2),'mh','MarkerSize',7,'LineWidth',3);
    end
    
end

if(~hold_state)
    hold off;
end

end

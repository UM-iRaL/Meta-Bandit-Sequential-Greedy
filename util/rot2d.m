function R = rot2d(t)
% a rotation of t about the Z axis in 2D
%
% R = 2 x 2 x num_t
t = t(:);
num_t = length(t);

R = zeros(2,2,num_t);

R(1,1,:) = cos(t);
R(1,2,:) = -sin(t);
R(2,1,:) = sin(t);
R(2,2,:) = cos(t);

%R = [cos(t) -sin(t)
%    sin(t) cos(t)];

    
end
function x = xyQ_to_xyab(x)
% 
% x = 3 x num_sta = (x,y,Q) states
%
% x = 4 x num_sta = (x,y,a,b) states such that
%
% a+i*b is the complex number representation of Q
%

[a,b] = size(x);

if a == 3
    Q = x(3,:);
    
    x = [x(1:2,:); cos(Q); sin(Q)];
elseif b == 3
    Q = x(:,3);
    x = [x(:,1:2), cos(Q), sin(Q)];
else
    error('[xyQ_to_xyab] One dimension of x should be 3!');
end

end
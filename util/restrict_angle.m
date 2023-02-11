function phi = restrict_angle(phi,minrange,maxrange)
% phi = restrict_angle(phi,minrange,maxrange)
%
%   restricts phi to the range [minrange,maxrange]
%

if nargin < 2
    minrange = -pi;
    maxrange = pi;
end

%phi = minrange + mod(phi - minrange, maxrange - minrange);

% Faster than using mod
x = phi - minrange;
y = maxrange - minrange;
phi = minrange + x - y.*floor(x./y);

end

function P = ellipse_plot_nx(A,c,n)
%
%  ellipse_plot_nx(A,c,n) plots a 2D ellipse or a 3D ellipsoid 
%  represented in the "center" form:  
%               
%                   (x-c)' A^{-1} (x-c) <= 1
% 
%  The eigenvectors of A define the principal directions of the ellipsoid 
%  and the eigenvalues of A are the squares of the semi-axes
%
%  Inputs: 
%  A: a 2x2 or 3x3 positive-definite matrix.
%  c: a 2D or a 3D vector which represents the center of the ellipsoid.
%  n: the number of grid points for plotting the ellipse; Default: n = 20. 
%
%  Output:
%   P = dim x 127 = points representing the ellipsoid
%   
%   2D plot:
%       plot(P(1,:),P(2,:));
%   3D plot:
%       mesh(reshape(P(1,:),(n+1),[]),
%            reshape(P(2,:),(n+1),[]),
%            reshape(P(3,:),(n+1),[]));
%
%  Nikolay Atanasov
%  atanasov@seas.upenn.edu
%  University of Pennsylvania
%  15 July 2013
%

if nargin < 3
    n = 20;
end

[a,b] = size(A);
if a ~= b
    error('Only a square matrix can represent an ellipsoid...');
elseif a > 3
    error('Cannot plot an ellipse with more than 3 dimensions...');
elseif a < 2
    error('The matrix A is degenerate...');
elseif a == 3
    Type = '3D';
else
    Type = '2D';
end


if(nargin < 2)
    c = zeros(a,1);
end


[U, D, ~] = svd(A);
%[V,D] = eig(A);

if strcmp(Type, '2D')
    % get the major and minor axes
    %------------------------------------
    a1 = sqrt(D(1,1));
    a2 = sqrt(D(2,2));

    theta = 0:1/n:2*pi+1/n;

    % Parametric equation of the ellipse
    % Coordinate transform 
    %----------------------------------------
    %P = bsxfun(@plus,V * [a1*cos(theta); a2*sin(theta)],c(:));
    P = bsxfun(@plus,U \ [a1*cos(theta); a2*sin(theta)],c(:));
    
else
    % generate the ellipsoid at (0,0,0)
    %----------------------------------
    a1 = sqrt(D(1,1));
    a2 = sqrt(D(2,2));
    a3 = sqrt(D(3,3));
    [X,Y,Z] = ellipsoid(0,0,0,a1,a2,a3,n);

    %  rotate and center the ellipsoid to the actual center point
    %------------------------------------------------------------
    P = bsxfun(@plus,U \ [X(:)';Y(:)';Z(:)'],c(:));
    
end


end
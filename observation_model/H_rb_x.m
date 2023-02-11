function H = H_rb_x(x,y,r_sense,fov)
%
%  y = num_y x 2
%  x = num_x x 3
% 
%  r_sense = sensing radius
%  fov = field of view in radians
%
% OUTPUT:
%   H = sparse(2*num_y,3*num_x) = jacobian matrices
%

num_y = size(y,1);
num_x = size(x,1);

% determine which landmarks y are detectable from x
detectable = detectable2D(y,x,r_sense,fov,true); % num_y x num_x
%num_detectable = sum(detectable,1); % 1 x num_x

[idx_mat_y,idx_mat_x] = ndgrid(1:num_y,1:num_x);
idx_list_y = idx_mat_y(detectable);
idx_list_y = transpose(idx_list_y(:));
idx_list_x = idx_mat_x(detectable);
idx_list_x = transpose(idx_list_x(:));

if( isempty(idx_list_y) )
    H = sparse(2*num_y,3*num_x);
else
    jac = rb_jacobian_x(x(idx_list_x,1),x(idx_list_x,2),...
        y(idx_list_y,1),y(idx_list_y,2)); % 2 x 3 x num_detectable_pairs
    
    idx_y = [2*idx_list_y-1;2*idx_list_y];

    ii = reshape(idx_y,2,1,[]);
    ii = ii(:,ones(3,1),:);
    ii = ii(:);
    
    idx_x = [3*idx_list_x-2; 3*idx_list_x-1; 3*idx_list_x];
    jj = reshape(idx_x,1,3,[]);
    jj = jj(ones(2,1),:,:);
    jj = jj(:);
    
    H = sparse(ii,jj,jac(:),2*num_y,3*num_x);
end

end

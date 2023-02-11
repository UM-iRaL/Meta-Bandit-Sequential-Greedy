function H = H_rb_xy(x,y,r_sense,fov,sep)
%
%  x = num_x x 3
%  y = num_y x 2
%  
% 
%  r_sense = sensing radius
%  fov = field of view in radians
%  sep = separate between H_x and H_y, i.e.: H = [H_x 0_sep H_y] 
% OUTPUT:
%   H = sparse(2*num_y,(3+sep+2*num_y)*num_x) = jacobian matrix
%

if nargin < 5
    sep = 0;
end

num_y = size(y,1);
num_x = size(x,1);

% determine which landmarks y are detectable from x
detectable = detectable2D(y,x,r_sense,fov,true); % num_y x num_x
%num_detectable = sum(detectable,1); % 1 x num_x

[idx_mat_y,idx_mat_x] = ndgrid(1:num_y,1:num_x); % num_y x num_x
idx_list_y = idx_mat_y(detectable);
idx_list_y = transpose(idx_list_y(:));
idx_list_x = idx_mat_x(detectable);   % num_x x num_y
idx_list_x = transpose(idx_list_x(:));

if( isempty(idx_list_y) )
    H = sparse(2*num_y,(3+sep+2*num_y)*num_x);
else
    jac_x = rb_jacobian_x(x(idx_list_x,1),x(idx_list_x,2),...
        y(idx_list_y,1),y(idx_list_y,2)); % 2 x 3 x num_detectable_pairs
    jac_y = rb_jacobian_y(x(idx_list_x,1),x(idx_list_x,2),...
        y(idx_list_y,1),y(idx_list_y,2)); % 2 x 2 x num_detectable_pairs
        
    idx_y = [2*idx_list_y-1;2*idx_list_y];      % 2*num_x x num_y
    idx_y = reshape(idx_y,2,1,[]);
    ii_x = idx_y(:,ones(3,1),:);
    ii_x = ii_x(:);
    ii_y = idx_y(:,ones(2,1),:);
    ii_y = ii_y(:);

    
    idx_x = [(3+2*num_y)*(idx_list_x-1)+1; (3+2*num_y)*(idx_list_x-1)+2; (3+2*num_y)*(idx_list_x-1)+3];
    jj_x = reshape(idx_x,1,3,[]);
    jj_x = jj_x(ones(2,1),:,:);
    jj_x = jj_x(:);   

    jj_y = bsxfun(@plus,(idx_list_x-1)*(3+2*num_y), 3+[2*idx_list_y-1; 2*idx_list_y-1; 2*idx_list_y; 2*idx_list_y]);
    jj_y = jj_y(:);
    
    H = sparse([ii_x;ii_y],[jj_x;jj_y+sep],[jac_x(:);jac_y(:)],2*num_y,(3+sep+2*num_y)*num_x);
end

end

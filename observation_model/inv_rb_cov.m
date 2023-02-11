function  y_cov = inv_rb_cov(x, z, cov_x, cov_z, cur_y_cov, cur_y)
%   INPUT:
%   x: 3*1, cov_x: 3*3
%   z: range-bearing measurment, num_det*2, cov_z:2*2
%   OUTPUT:
%   y_cov: the covariance matrix of landmark given robot location and range
%   bearing mearsurement.
    if nargin < 5
        num_det = size(z, 1);
        y_cov = zeros([num_det,2,2]);
        for k = 1:num_det
            jac_x = [1 0 -z(k,1)*sin(x(3)+z(k,2));
                0 1  z(k,1)*cos(x(3)+z(k,2))];
            jac_z = [cos(x(3)+z(k,2)) -z(1)*sin(x(3)+z(k,2));
                sin(x(3)+z(k,2))  z(1)*cos(x(3)+z(k,2))];
            y_cov(k,:,:) = jac_x*cov_x*jac_x' + jac_z*cov_z*jac_z';
            
        end
    else
    end
end


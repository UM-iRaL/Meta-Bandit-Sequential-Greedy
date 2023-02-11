function F = dd_motion_model_jacobian_u(x,U,samp,flag)
% jacobian of motion model wrt u
%
% @input:
%   x = 3 x num_x = starting states (x,y,Q)
%   U = 2 x num_u = [v(1), ..., v(n);
%                    w(1), ..., w(n)];
%   samp = time discretization step
%
%   flag = [false] = if true returns the new state of every x versus every u
%                   else assumes num_sta = num_controls (or one is scalar)
%                   and returns the corresponding new states
%
% @output:
%   F = (3 x 2 x num_u x num_x)   = if flag == true
%     = (3 x 2 x max(num_u,num_x)) = if flag == false
%

if( nargin < 4 )
    flag = false;
end

% discretization step
if( nargin < 3)
    samp = 0.5;
end

[u_dim, num_ctrl] = size(U);
[x_dim, num_sta] = size(x);


if( flag )
    F = zeros(x_dim,u_dim, num_ctrl,num_sta);
    F(3,2,:,:) = samp;
    
    dyaw = transpose(samp*U(2,:));  % num_ctrl x 1
    sin_old_x_3 = sin(x(3,:));     % 1 x num_sta
    cos_old_x_3 = cos(x(3,:));     % 1 x num_sta

    isSmall_w = (abs(dyaw) <= 0.001);
    isLarge_w = ~isSmall_w;

    if(any(isLarge_w))
        new_x_3_large = bsxfun(@plus,x(3,:),dyaw(isLarge_w));     % num_large x num_sta
        cos_new_x_3_large = cos(new_x_3_large);         % num_large x num_sta
        sin_new_x_3_large = sin(new_x_3_large);         % num_large x num_sta
        
        F11_large = bsxfun(@rdivide,bsxfun(@minus,sin_new_x_3_large,sin_old_x_3),U(2,isLarge_w).');
        F21_large = bsxfun(@rdivide,bsxfun(@minus,cos_old_x_3,cos_new_x_3_large),U(2,isLarge_w).');
        
        
        R_large = transpose(U(1,isLarge_w)./U(2,isLarge_w)); % num_large x 1 
        F12_large = bsxfun(@times,cos_new_x_3_large,samp*R_large)-bsxfun(@times,F11_large, R_large);
        F22_large = bsxfun(@times,sin_new_x_3_large,samp*R_large)-bsxfun(@times,F21_large, R_large);
        
        F(1,1,isLarge_w,:) = F11_large;
        F(2,1,isLarge_w,:) = F21_large;
        F(1,2,isLarge_w,:) = F12_large;
        F(2,2,isLarge_w,:) = F22_large;
    end
    

    if(any(isSmall_w))
        new_x_3_small = bsxfun(@plus,x(3,:),dyaw(isSmall_w)/2);   % num_small x num_sta
        cos_new_x_3_small = cos(new_x_3_small);         % num_small x num_sta
        sin_new_x_3_small = sin(new_x_3_small);         % num_small x num_sta
        
        F11_small = samp*cos_new_x_3_small;
        F21_small = samp*sin_new_x_3_small;
        
        F12_small = -samp/2 * bsxfun(@times,F21_small,U(1,isSmall_w).');
        F22_small = samp/2 * bsxfun(@times,F11_small,U(1,isSmall_w).');
        
        F(1,1,isSmall_w,:) = F11_small;
        F(2,1,isSmall_w,:) = F21_small;
        F(1,2,isSmall_w,:) = F12_small;
        F(2,2,isSmall_w,:) = F22_small;        
    end
    
else
    F = zeros(3,2,max(num_ctrl,num_sta));
    F(3,2,:) = samp;
    
    dyaw = transpose(samp*U(2,:));  % num_ctrl x 1
    sin_old_x_3 = sin(x(3,:));     % 1 x num_sta
    cos_old_x_3 = cos(x(3,:));     % 1 x num_sta
    
    isSmall_w = (abs(dyaw) <= 0.001);
    isLarge_w = ~isSmall_w;
    
    if(any(isLarge_w))
        
        if( num_sta == 1)
            new_x_3_large = x(3) + dyaw(isLarge_w);
            sin_old_x_3_large = sin_old_x_3;
            cos_old_x_3_large = cos_old_x_3;
        else
            new_x_3_large = transpose(x(3,isLarge_w))+dyaw(isLarge_w);     % max(num_large,num_sta) x 1
            sin_old_x_3_large = transpose(sin_old_x_3(isLarge_w));
            cos_old_x_3_large = transpose(cos_old_x_3(isLarge_w));
        end
        
        cos_new_x_3_large = cos(new_x_3_large);
        sin_new_x_3_large = sin(new_x_3_large);
        
        F11_large = (sin_new_x_3_large - sin_old_x_3_large)./transpose(U(2,isLarge_w));
        F21_large = (cos_old_x_3_large - cos_new_x_3_large)./transpose(U(2,isLarge_w));
        

        R_large = transpose(U(1,isLarge_w)./U(2,isLarge_w)); % num_large x 1
        
        F12_large = cos_new_x_3_large.*(samp*R_large) - F11_large.*R_large;
        F22_large = sin_new_x_3_large.*(samp*R_large) - F21_large.*R_large;
        
        if( num_ctrl > 1)
            F(1,1,isLarge_w) = F11_large;
            F(2,1,isLarge_w) = F21_large;
            F(1,2,isLarge_w) = F12_large;
            F(2,2,isLarge_w) = F22_large;
        else
            F(1,1,:) = F11_large;
            F(2,1,:) = F21_large;
            F(1,2,:) = F12_large;
            F(2,2,:) = F22_large;            
        end
    end
    
    if(any(isSmall_w))
        
        if( num_sta == 1)
            new_x_3_small = x(3)+dyaw(isSmall_w)/2;   % max(num_small,num_sta) x 1
        else
            new_x_3_small = x(3,isSmall_w)+dyaw(isSmall_w)/2;
        end
            
        cos_new_x_3_small = cos(new_x_3_small);
        sin_new_x_3_small = sin(new_x_3_small);
        
        F11_small = samp*cos_new_x_3_small;
        F21_small = samp*sin_new_x_3_small;
        
        F12_small = -samp/2 * F21_small.*U(1,isSmall_w).';
        F22_small = samp/2 * F11_small.*U(1,isSmall_w).';

        if( num_ctrl > 1)
            F(1,1,isSmall_w) = F11_small;
            F(2,1,isSmall_w) = F21_small;
            F(1,2,isSmall_w) = F12_small;
            F(2,2,isSmall_w) = F22_small;
        else
            F(1,1,:) = F11_small;
            F(2,1,:) = F21_small;
            F(1,2,:) = F12_small;
            F(2,2,:) = F22_small;
        end
        
       
    end
    
end


end



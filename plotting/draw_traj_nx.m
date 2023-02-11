function h = draw_traj_nx(h,traj,varargin)
% h = draw_traj_nx(h,traj,style)
%
%   traj = num_pts x num_dim x num_traj
%   style = 'b-'
%
% Author:
%   Nikolay Atanasov
%

if(nargin < 3)
    varargin={'b-'};
end

num_traj = size(traj,3);
traj = permute(traj,[1 3 2]);   % num_pts x num_traj x num_dim

if(size(traj,1) == 1)
    traj = cat(1,traj,traj);
end

if(isempty(h))
    
    if( size(traj,3) > 2 )
        h = plot3(traj(:,:,1),traj(:,:,2),traj(:,:,3),varargin{:}, 'LineWidth', 1.5);
    else
        h = plot(traj(:,:,1),traj(:,:,2),varargin{:}, 'LineWidth', 1.5);
    end

    %for k = 1:num_traj
    %    h(k) = plot(traj(:,1,k),traj(:,2,k),style);
    %end
else
    if( size(traj,3) > 2 )
        set(h,{'xdata'},mat2cell(traj(:,:,1).',ones(num_traj,1)),...
            {'ydata'},mat2cell(traj(:,:,2).',ones(num_traj,1)),...
            {'zdata'},mat2cell(traj(:,:,3).',ones(num_traj,1)));
    else
        set(h,{'xdata'},mat2cell(traj(:,:,1).',ones(num_traj,1)),...
            {'ydata'},mat2cell(traj(:,:,2).',ones(num_traj,1)));
    end
    
    %for k = 1:num_traj
    %    set(h(k),'xdata',traj(:,1,k),'ydata',traj(:,2,k));
    %end
end

end

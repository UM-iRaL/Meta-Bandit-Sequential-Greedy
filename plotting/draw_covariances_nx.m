function h = draw_covariances_nx(h,m,C,clr)
% m = 2 x num_pts = means
% C = 2 x 2 x num_pts = covariances
% clr = color for the plot

if nargin < 4
    clr = 'r';
end

num_pts = size(m,2);
C_pts = zeros(127,num_pts,2);
if num_pts == 1
    if( any(any(isnan(C(:,:)))) )
        C_pts(:,1,:) = nan;
    else
        C_pts(:,1,:) = transpose(ellipse_plot_nx(C(:,:),m(:,1)));
    end
else
    for k = 1:num_pts
        if( any(any(isnan(C(:,:,k)))) )
            C_pts(:,k,:) = nan;
        else
            C_pts(:,k,:) = transpose(ellipse_plot_nx(C(:,:,k),m(:,k)));
        end
    end
end


if ~isempty(h)
    delete(h)
end

h = plot(C_pts(:,:,1), C_pts(:,:,2), [clr '-'],'LineWidth',2);

% Even though the below is GREAT it does not work when the number of points
% is changing
%
%set(h,'xdata',C_pts(:,:,1),'ydata',C_pts(:,:,2));
%set(h,{'xdata'},mat2cell(C_pts(:,:,1).',ones(num_pts,1)),...
%    {'ydata'},mat2cell(C_pts(:,:,2).',ones(num_pts,1)));



end
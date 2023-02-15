function h = draw_pred_nx(h, pred, clr)
%
% pred : 2 x num_pred
% h = draw_pred_nx(h,pred, clr)
% 
%
% unit = normalize(pred(:, 2) - pred(:, 1), 'norm'); 

if(isempty(h))
    if nargin > 2
        h = plot(pred(1, :),pred(2, :), strcat(clr, '.'), 'LineWidth',2);
    else
        h = plot(pred(1, :), pred(2, :),'Color',"#7E2F8E", 'LineStyle','-', 'LineWidth', 2);
    end
else
%     h = plot(pred(1, :), pred(2, :),'Color',"r", 'LineStyle','-', 'LineWidth', 2);
    set(h,'xdata',pred(1, :),'ydata',pred(2, :), 'Color',"r", 'LineStyle','-', 'LineWidth', 2);
end
end
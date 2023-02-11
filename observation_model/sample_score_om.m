function s = sample_score_om(c,y,pscymat,flag)
%
% INPUT:
%   c = num_c x 1 = detected class
%   y = num_y x 1 = correct class
%   
%   pscymat = num_score x num_class x num_class = p(s | c, y) 
%
%   flag = true = returns s for every combination of c,y
%        = false = assumes num_c = num_y (or some are scalar) and 
%                  returns a vector s
% OUTPUT:
%   s = detection score = num_c x num_y         if flag true
%                       = max(num_c,num_y) x 1  if flag false
%
%

if(nargin < 4)
    flag = false;
end
num_c = length(c);
num_y = length(y);
num_score = size(pscymat,1);

if(flag)
    

    [Sm, Cm, Ym] = ndgrid(1:num_score,c,y);
    linidx = sub2ind(size(pscymat), Sm(:), Cm(:), Ym(:));
    p = transpose(reshape(pscymat(linidx),num_score,[]));
    s = discretesample_nx(p,1);
    s = reshape(s,num_c,num_y);
    
    
    % OLD, SLOW and WRONG
    %[Cm,Ym] = ndgrid(c,y);
    %Cm = Cm(:); Ym = Ym(:);
    %s = zeros(num_c,num_y);
    %for k = 1:(num_c*num_y)
    %    s(k) = discretesample(pscymat(:,Cm(k),Ym(k)),1);
    %end

else

    if(num_c == 1 || num_y == 1)
        p = transpose(squeeze(pscymat(:,c,y)));
        s = discretesample_nx(p,1);
    elseif(num_c == num_y)
        Sm = 1:num_score;
        Sm = transpose(Sm(ones(num_c,1),:));
        Cm = transpose(c(:,ones(num_score,1)));
        Ym = transpose(y(:,ones(num_score,1)));
        linidx = sub2ind(size(pscymat), Sm(:), Cm(:), Ym(:));
        p = transpose(reshape(pscymat(linidx),num_score,[]));
        s = discretesample_nx(p,1);
    else
        error('[sample_score_om] Dimension mismatch!');
    end
        
        
    % if(num_c == num_y)
    %     s = zeros(num_c,1);
    %     for k = 1:num_c
    %         s(k) = discretesample(pscymat(:,c(k),y(k)),1);
    %     end
    % else
    %     s = zeros(num_y,1);
    %     for k = 1:num_y
    %         s(k) = discretesample(pscymat(:,c,y(k)),1);
    %     end        
    % elseif(num_y == 1)
    %     s = zeros(num_c,1);
    %     for k = 1:num_c
    %         s(k) = discretesample(pscymat(:,c(k),y),1);
    %     end        
    % else
    %     error('[sample_scoreid_model] Dimension mismatch!');
    % end
end

end
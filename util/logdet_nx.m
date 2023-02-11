function y = logdet_nx(A)
% Computes the logdet of the positive semidefinite matrix A
%
% author: Nikolay Atanasov
    [U,p] = chol(A);
    if(p == 0)
        y = 2*sum(log(diag(U)));
    else
        y = -inf;
    end
end

function d = logdetchol_nx(R)
% computes the logdet of the matrix whose cholseky factor is R
d = 2*sum(log(abs(diag(R))));
end

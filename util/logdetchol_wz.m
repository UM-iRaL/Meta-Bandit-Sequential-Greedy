function d = logdetchol_wz(R)
% computes the logdet of the matrix whose cholseky factor is R
S = diag(R);
S(S==0) = 1e-10;
d = 2*sum(log(abs(S)));
end
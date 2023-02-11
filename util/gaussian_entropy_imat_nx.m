function h = gaussian_entropy_imat_nx(imat)
% Cov = inv( iChol.' * iChol )
n = size(imat,1);
ld = -logdet_nx(imat);
h = (n*log(2*pi*exp(1)) + ld)/2;
end
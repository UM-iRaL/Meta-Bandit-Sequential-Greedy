function h = gaussian_entropy_imatchol_nx(iChol)
% Cov = inv( iChol.' * iChol )
n = size(iChol,2);
ld = -logdetchol_nx(iChol);
h = (n*log(2*pi*exp(1)) + ld)/2;
end
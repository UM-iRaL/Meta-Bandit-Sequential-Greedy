function h = gaussian_entropy_nx(Cov)
n = size(Cov,1);
ld = logdet_nx(Cov);
h = (n*log(2*pi*exp(1)) + ld)/2;
end
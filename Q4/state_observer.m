fsf()
assert(rank(obsv(A,C))==12) % Assert observability of the system

eigenvalues = [
    0.8
    0.75
    0.76
    0.74
    0.62
    0.63
    0.44
    0.71
    0.73
    0.72
    0.81
    0.7];
% L = place(A.',C.',eigenvalues);
% 
% xhat = A*xhat + B*u+L
% e=(A-L*C)*e;
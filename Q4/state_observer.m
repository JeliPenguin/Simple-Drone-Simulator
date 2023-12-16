assert(rank(obsv(A,C))==12) % Assert observability of the system

eigenvalues = [
    0.1
    0.12
    0.13
    0.15
    0.14
    0.2
    0.18
    0.17
    0.16
    0.2
    0.11
    0.22];
LMat = place(A.',C.',eigenvalues).';
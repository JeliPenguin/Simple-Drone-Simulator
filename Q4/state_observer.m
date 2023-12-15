assert(rank(obsv(A,C))==12) % Assert observability of the system

eigenvalues = [
    0.19
    0.32
    0.23
    0.25
    0.24
    0.3
    0.28
    0.27
    0.26
    0.2
    0.31
    0.22];
LMat = place(A.',C.',eigenvalues).';
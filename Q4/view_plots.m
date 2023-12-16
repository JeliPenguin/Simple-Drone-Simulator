titles = ["X","Y","Z","X dot","Y dot","Z dot","Omega X","Omega Y","Omega Z","Phi","Theta","Psi"];

for index = 1:length(titles) % columns 1 to c
    openfig("plots/"+titles(index)+'.fig','new','visible')
end

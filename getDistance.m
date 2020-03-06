function distance = getDistance(porteeCapteur,angleCapteur, xMurA, yMurA, xMurB, yMurB, matrice)
coordonneeMurA = matrice \ [xMurA; yMurA; 1] % coordonnee du point A du mur
coordonneeMurB = matrice \ [xMurB; yMurB; 1] % coordonnee du point B du mur
matriceDistance = [[porteeCapteur*cos(angleCapteur), coordonneeMurA(1)-coordonneeMurB(1)];...
                   [porteeCapteur*sin(angleCapteur), coordonneeMurA(2)-coordonneeMurB(2)]];
if (abs(det(matriceDistance)) < 0.001)
    distance = [10000; 10000]
else
    st = matriceDistance \ [coordonneeMurA(1); coordonneeMurA(2)]
    % s est represente l'intersection pour le laser
    % t l'intersection pour l'obstacle
    distance = st
end
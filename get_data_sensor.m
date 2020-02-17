function st = get_data_sensor(array_obstacle_line, size_array, portee, matrice, capteur)
st = getDistance(portee, capteur.angle, array_obstacle_line(1).xA, array_obstacle_line(1).yA,...
                  array_obstacle_line(1).xB, array_obstacle_line(1).yB, matrice)

for i = 2: 1 : size_array;
    % Recupere la valeur st pour le capteur 1 pour l'obstacle i
    newSt = getDistance(portee, capteur.angle, array_obstacle_line(i).xA, array_obstacle_line(i).yA,...
                  array_obstacle_line(i).xB, array_obstacle_line(i).yB, matrice)
    % Si l'obstacle touche le laser et que c'est l'obstacle le plus proche
    % on change de valeur
    if (newSt(2) > 0 && newSt(2) < 1 && newSt(1) > 0 && newSt(1) < st(1))
        st = newSt
    end
    
end
end
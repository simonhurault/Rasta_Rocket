function st = get_data_sensor(array_obstacle_line, size_array, portee, matrice, capteur)
st = [1, 1]

ctr = 1;
while( ctr <= size_array)
    newSt = getDistance(portee, capteur.angle, array_obstacle_line(ctr).xA, array_obstacle_line(ctr).yA,...
                  array_obstacle_line(ctr).xB, array_obstacle_line(ctr).yB, matrice)
    % Si l'obstacle touche le laser et que c'est l'obstacle le plus proche
    % on change de valeur
    if (newSt(2) > 0 && newSt(2) < 1)
        if(newSt(1) > 0 && newSt(1) < 1) 
            if(newSt(1) < st(1))
                st(1) = newSt(1)
                st(2) = newSt(2)
            end
        end
    end
    ctr = ctr + 1
end
end
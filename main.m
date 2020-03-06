function [] = main()
% init environnement matlab %
clc
close all
clear all

% init figures %
figure(1)
clf
hold on
set(1,'position',[0 0 1000 1000])
axis([-1 10 -1 10])

% init systeme robot %
rasta.R = 0.14; % rayon roue
rasta.L = 0.48; % distance entre roues
rasta.H = 0.8; % longueur robot
tau = 0.1;
capteur.Portee = 3; % portee capteur
capteur1.angle = 0; %Capteur milieu
capteur2.angle = pi/6; % capteur gauche
capteur3.angle = -pi/6; % capteur droit

% Obstacle 1 %
obstacle1.xA = 7
obstacle1.yA = 3
obstacle1.xB = 4
obstacle1.yB = -1
obstacle1.ptr = line([obstacle1.xA, obstacle1.xB],[obstacle1.yA,obstacle1.yB],'color','r','linewidth',3);

% Obstacle 2 %
obstacle2.xA = 0
obstacle2.yA = 9
obstacle2.xB = 3
obstacle2.yB = 6
obstacle2.ptr = line([obstacle2.xA, obstacle2.xB],[obstacle2.yA,obstacle2.yB],'color','r','linewidth',3);

% Obstacle 3 %
obstacle3.xA = 18
obstacle3.yA = 12
obstacle3.xB = 9
obstacle3.yB = 3
obstacle3.ptr = line([obstacle3.xA, obstacle3.xB],[obstacle3.yA,obstacle3.yB],'color','r','linewidth',3);


% Bordure %

% gauche
bordureg.xA = -1
bordureg.yA = -1
bordureg.xB = -1
bordureg.yB = 10
bordureg.ptr = line([bordureg.xA, bordureg.xB],[bordureg.yA,bordureg.yB],'color','r','linewidth',3);

% Haute
bordureh.xA = -1
bordureh.yA = 10
bordureh.xB = 10
bordureh.yB = 10
bordureh.ptr = line([bordureh.xA, bordureh.xB],[bordureh.yA,bordureh.yB],'color','r','linewidth',3);

% Droite
bordured.xA = 10
bordured.yA = 10
bordured.xB = 10
bordured.yB = -1
bordured.ptr = line([bordured.xA, bordured.xB],[bordured.yA,bordured.yB],'color','r','linewidth',3);

% Bas
bordureb.xA = 10
bordureb.yA = -1
bordureb.xB = -1
bordureb.yB = -1
bordureb.ptr = line([bordureb.xA, bordureb.xB],[bordureb.yA,bordureb.yB],'color','r','linewidth',3);


array_obstacle_line =[bordureb,bordureg,bordureh,bordured,obstacle1,obstacle2,obstacle3];
%array_obstacle_line(2) = bordureg
%array_obstacle_line(3) = bordureh
%array_obstacle_line(4) = bordured
%array_obstacle_line(5) = bordureb

size_array_line = 7

%coord robot
[rasta.X,rasta.Y]= ginput(1);
rasta.theta = pi/4;
rasta.omd = 0
rasta.omg = 0


capteur1.ptr = line([rasta.X,rasta.X + capteur.Portee * cos(rasta.theta)],...
     [rasta.Y,rasta.Y + capteur.Portee * sin(rasta.theta)],'color','g','linewidth',3);
capteur2.ptr = line([rasta.X,rasta.X + capteur.Portee * cos(rasta.theta + capteur2.angle)],...
    [rasta.Y,rasta.Y + capteur.Portee * sin(rasta.theta + capteur2.angle)],'color','g','linewidth',3);
capteur3.ptr = line([rasta.X,rasta.X + capteur.Portee * cos(rasta.theta + capteur3.angle)],...
    [rasta.Y,rasta.Y + capteur.Portee * sin(rasta.theta + capteur3.angle)],'color','g','linewidth',3);






xt = [rasta.X + rasta.L / 2 * sin(rasta.theta); rasta.X - rasta.L / 2 * sin(rasta.theta);...
    rasta.X + rasta.H*cos(rasta.theta)]
yt = [rasta.Y - rasta.L / 2 * cos(rasta.theta); rasta.Y + rasta.L / 2 * cos(rasta.theta);...
    rasta.Y + rasta.H * sin(rasta.theta)]
rasta.ptr1 = patch(xt,yt,[0;4;7])



% init environnement exec %
Tmax = 500;
dt = 0.1;

x = [rasta.X rasta.Y rasta.theta rasta.omd rasta.omg] % init etat robot
u = [0 0] % vitesse consigne roue droite puis gauche
vitesse = [0 0] % vitesse de consigne engulaire et lineaire


for t = 0 : dt : Tmax;
    matrice = [[cos(rasta.theta),-sin(rasta.theta),rasta.X];...
    [sin(rasta.theta),cos(rasta.theta),rasta.Y];...
    [0,0,1]]
    
    % maj etat robot
    rasta.X = x(1)
    rasta.Y = x(2)
    rasta.theta = x(3)
    rasta.omd = x(4)
    rasta.omg = x(5)
    
    % maj dessin
    xt = [rasta.X + rasta.L / 2 * sin(rasta.theta); rasta.X - rasta.L / 2 * sin(rasta.theta);...
        rasta.X + rasta.H*cos(rasta.theta)]
    yt = [rasta.Y - rasta.L / 2 * cos(rasta.theta); rasta.Y + rasta.L / 2 * cos(rasta.theta);...
        rasta.Y + rasta.H * sin(rasta.theta)]
    set(rasta.ptr1,'Xdata',xt, 'Ydata', yt)
    set(capteur1.ptr,'Xdata',[rasta.X,rasta.X + capteur.Portee * cos(rasta.theta + capteur2.angle)],...
    'Ydata',[rasta.Y,rasta.Y + capteur.Portee * sin(rasta.theta + capteur2.angle)])
    set(capteur2.ptr,'Xdata',[rasta.X,rasta.X + capteur.Portee * cos(rasta.theta)],...
    'Ydata',[rasta.Y,rasta.Y + capteur.Portee * sin(rasta.theta)])
    set(capteur3.ptr,'Xdata',[rasta.X,rasta.X + capteur.Portee * cos(rasta.theta + capteur3.angle)],...
    'Ydata',[rasta.Y,rasta.Y + capteur.Portee * sin(rasta.theta + capteur3.angle)])

    % calcul commande
    st1 = get_data_sensor(array_obstacle_line, size_array_line, capteur.Portee, matrice, capteur1)
    st2 = get_data_sensor(array_obstacle_line, size_array_line, capteur.Portee, matrice, capteur2)
    st3 = get_data_sensor(array_obstacle_line, size_array_line, capteur.Portee, matrice, capteur3)
    

    % Cas tout droit
    if(st1(1) >= 1 || (st1(1) <= 1 && st1(2) > 1))
        if(st2(1) > rasta.H / capteur.Portee || (st2(1) <= 1 && st2(2) > 1))
            if(st3(1) > rasta.H / capteur.Portee || (st3(1) <= 1 && st3(2) > 1))
                vitesse(1) = 1
                vitesse(2) = 0
            end
        end
        
    elseif(st2(1) <= rasta.H / capteur.Portee && st2(2) < 1 && st2(1) > 0 && st2(2) > 0)
            vitesse(1) = 0
            vitesse(2) = -0.2
        
    
    elseif(st3(1) <= rasta.H / capteur.Portee && st3(2) < 1 && st3(1) > 0 && st3(2) > 0)
            vitesse(1) = 0
            vitesse(2) = 0.2
 
        
    elseif(st2(1) < 1 && st2(2) < 1 && st2(1) > 0 && st2(2) > 0)
       % if(st3(1) < 1 && st3(2) < 1 && st3(1) > 0 && st3(2) > 0 && st2(1) <= st3(1))
            vitesse(1) = 1 * (st2(1) - rasta.H / capteur.Portee)
            vitesse(2) = -0.2  / (st2(1) - rasta.H / capteur.Portee)
       % end
        
    elseif (st3(1) < 1 && st3(2) < 1 && st3(1) > 0 && st3(2) > 0)
        vitesse(1) = 1 * (st3(1) - rasta.H / capteur.Portee)
        vitesse(2) = 0.2 / (st3(1) - rasta.H / capteur.Portee)
    else 
        vitesse(1) = 1
        vitesse(2) = 0     
    end
    
    % simu du robot
    u(1) = (vitesse(2) * rasta.L)/(2*rasta.R) + vitesse(1)/rasta.R
    u(2) = -(vitesse(2) * rasta.L)/(2*rasta.R) + vitesse(1)/rasta.R
    [t45,x45] = ode45(@(t,x)Modele(t,x,u,rasta, tau),[0 dt],x);
    L45 = length(t45);
    x = x45(L45,:);
    drawnow
    


end


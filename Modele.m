
function xdot = Modele( t,x,u,rasta )
xdot = [0;0;0]
xdot(1) = u(1)*cos(x(3));
xdot(2) = u(1)*sin(x(3));
xdot(3) = u(2);

end


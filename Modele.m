
function xdot = Modele(t,x,u,rasta, tau)
xdot = [0;0;0]
% xdot(1) = u(1)*cos(x(3));
% xdot(2) = u(1)*sin(x(3));
% xdot(3) = u(2); previous one
xdot(1) = (((x(4)+x(5))*rasta.R)/2)*cos(x(3));
xdot(2) = (((x(4)+x(5))*rasta.R)/2)*sin(x(3));
xdot(3) = (x(4) - x(5)) * rasta.R / rasta.L;
xdot(4) = -x(4)/tau + u(1)/tau
xdot(5) = -x(5)/tau + u(2)/tau


end


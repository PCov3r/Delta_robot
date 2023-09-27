%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INVERSE KINEMATICS IMPLEMENTATION %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% This function implements the inverse kinematics algorithm, as presented in the report.
% param: kinematic lengths
% (x,y,z): platform's position

function [th1,th2,th3] = ikm(param,x,y,z)
% kinematic parameters
R = param(1);
L1 = param(2);
L2 = param(3);
r = param(4);

% express e1,e2,e3 as in e1.cos(th)+e2.sin(th)+e3=0
i = 1:1:3;
e1 = 2*L1*((R-r)+y*cos(2*(i-1)*pi/3)-x*sin(2*(i-1)*pi/3));
e2 = -2*z*L1;
e3 = 2*y*(R-r)*cos(2*(i-1)*pi/3)-2*x*(R-r)*sin(2*(i-1)*pi/3)+R^2-2*R*r+r^2+x^2+y^2+z^2+L1^2-L2^2;

% get solutions for th1,2,3
thp = 2*atan((-e2+sqrt(e1.^2+e2.^2-e3.^2))./(e3-e1));
thm = 2*atan((-e2-sqrt(e1.^2+e2.^2-e3.^2))./(e3-e1));

% arrange solutions
th1 = [thp(1),thm(1)];
th2 = [thp(2),thm(2)];
th3 = [thp(3),thm(3)];

end

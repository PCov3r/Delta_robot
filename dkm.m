%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FORWARD KINEMATICS IMPLEMENTATION %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% This function implements the forward kinematics algorithm, as presented in the report.
% param: kinematic lengths
% (theta1,2,3): active joints angles

function [sol_x,sol_y,sol_z] = dkm(param,theta1,theta2,theta3)
    syms R L1 L2 r x y z th1 th2 th3

    end_pose = [x;y;z];

    % express OB'1
    OB1 = [0;-R-L1*cos(th1)+r;L1*sin(th1)];

    % obtain OB'2 and 3 by rotation
    R21 = [-1/2       -sqrt(3)/2  0;
           sqrt(3)/2  -1/2        0;
           0          0           1];
    OB2 = R21*[0;-R-L1*cos(th2)+r;L1*sin(th2)];

    R31 = [-1/2        sqrt(3)/2   0;
           -sqrt(3)/2  -1/2        0;
           0           0           1];
    OB3 = R31*[0;-R-L1*cos(th3)+r;L1*sin(th3)];

    % equations of sphere centred in OB' and of radius L2
    leg1 = sum((end_pose-OB1).^2) == L2^2;
    leg2 = sum((end_pose-OB2).^2) == L2^2;
    leg3 = sum((end_pose-OB3).^2) == L2^2;

    % rearrange into 3 other equations
    a = leg1 - leg2;
    b = leg1 - leg3;
    c = leg2 - leg3;

    % express x and y in terms of z
    eqx = isolate(expand((OB3(2,:)-OB1(2,:))*a - (OB2(2,:)-OB1(2,:))*b),x);
    eqy = isolate(expand((OB3(1,:)-OB1(1,:))*a - (OB2(1,:)-OB1(1,:))*b),y);
    % and substitute to get one equation depending only on z
    eqz = subs(leg1,[x,y],[rhs(eqx),rhs(eqy)]);

    % substitute known values and solve for z
    real_eqz = subs(eqz,[R,L1,L2,r,th1,th2,th3],[param,theta1,theta2,theta3]);
    sol_z = vpasolve(real_eqz,z);

    % with z known, solve for x
    real_eqx1 = subs(eqx,[R,L1,L2,r,th1,th2,th3,z],[param,theta1,theta2,theta3,sol_z(1)]);
    real_eqx2 = subs(eqx,[R,L1,L2,r,th1,th2,th3,z],[param,theta1,theta2,theta3,sol_z(2)]);
    sol_x = [vpasolve(real_eqx1,x);vpasolve(real_eqx2,x)];

    % with z known, solve for y
    real_eqy1 = subs(eqy,[R,L1,L2,r,th1,th2,th3,z],[param,theta1,theta2,theta3,sol_z(1)]);
    real_eqy2 = subs(eqy,[R,L1,L2,r,th1,th2,th3,z],[param,theta1,theta2,theta3,sol_z(2)]);
    sol_y = [vpasolve(real_eqy1,y);vpasolve(real_eqy2,y)];

end

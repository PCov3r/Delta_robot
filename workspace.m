%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% WORKSPACE PLOT IMPLEMENTATION %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% This program is used to plot the workspace of the Delta robot.
% It is based on a numerical approach, where the forward kinematics
% is used for various joint angles to obtain the total workspace.

% Delta robot constants
K=116;
L1=90;
L2=250;
R=50;

% Setup input joint angles
angle = linspace(-pi/2,pi/2,25);
% ndgrid is used to obtain all possible combinations
[th1,th2,th3] = ndgrid(angle,angle,angle);

% initialize workspace variable
wrkspace = [];

% sweep through all possible angles
for i=1:length(TH)
    % solve using forward kinematics
    [x,y,z] = dkm([K,L1,L2,R],th1(i),th2(i),th3(i));
    % if a solution exists, the point is part of the workspace
    if(isreal(x) && isreal(y) && isreal(z))
        sola = [x(1),y(1),z(1)];
        solb = [x(2),y(2),z(2)];
        % two solutions are possible but only one can be achieved 
        % in our case, z is supposed to be <0
        if(sola(3) < 0)
            wrkspace = cat(1,wrkspace, sola);
        else
            wrkspace = cat(1,wrkspace, solb);
        end
    end
end

%% PLOT WORKSPACE %%

% plot rough workspace as scatter
figure
scatter3(wrkspace(:,1),wrkspace(:,2),wrkspace(:,3))

% plot smooth workspace from alphaShape
figure
x = double(wrkspace(:,1));
y = double(wrkspace(:,2));
z = double(wrkspace(:,3));
shp = alphaShape(x,y,z);
plot(shp)
title("Delta robot workspace)")
xlabel('x (mm)')
ylabel('y (mm)')
zlabel('z (mm)')
pbaspect([1 1 1])

% obtain workspace volume
V = volume(shp);

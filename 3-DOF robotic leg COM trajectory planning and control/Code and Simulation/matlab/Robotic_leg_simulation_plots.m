%% Run this script after running the Robotic_leg.slx simulink model.

%% save the data to local variables.

% q is joint angles
q = [out.q.Data(:,1), out.q.Data(:,2), out.q.Data(:,3)];

% qd are joint angular velocities
qd = out.qd.Data;

%% Compute the COM position and COM velocity using q and qd.

[Ycom, Zcom] = COMPosition(q);
[Ydcom, Zdcom] = COMVelocity(q, qd);


%% Plotting the results of simulation

% COM path from simulation
figure(1);
plot(Ycom, Zcom);
grid on;
title('COM path from simulation');
xlabel('Y position of COM (units: centimeters)');
ylabel('Z position of COM (units: centimeters)');
legend('COM path');

%% COM path in x xdot plane with stability boundaries

foot_l = 11.7;
w = (9.81/0.4338)^0.5;
x = 0:0.2:10;
xd = -w*x;

figure(2);
plot(Ycom, Ydcom, '-g', x,xd,'-r', x,xd+w*foot_l,'-r');
grid on;
title('COM path in x xdot plane from simulation data');
xlabel('Y position of COM (units: centimeters)');
ylabel('Y velocity of COM (units: centimeters/second)');
legend('Y Ydot plot of COM','Boundary conditions');

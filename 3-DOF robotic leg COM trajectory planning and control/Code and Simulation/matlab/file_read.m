clearvars
% change the path of text file as needed. 

%% Setup the Import Options and import the data

opts = delimitedTextImportOptions("NumVariables", 7);

% Specify range and delimiter
opts.DataLines = [4, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["q1", "q2", "q3", "w1", "w2", "w3", "t"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "skip";

%% Import the data

% Specify the absolute path of the text file on your PC to successfully run
% the script.
JointAngles = readtable("D:\Arduino\BioRobotics\Sin_position.txt", opts);
j = table2array(JointAngles);

%% Clear temporary variables
clear opts JointAngles

%% Interpret the data

% Read the joint angles into q and time into t. initialize the home
% configuration

qh = [180,180,180];
q = [j(2:end,1)-qh(1), j(2:end,2)-qh(2), j(2:end,3)-qh(3)] .* pi/180;
t = j(1:end-1,7)./1000;

% Generate the reference sinusoidal input.
for i=1:length(q)
    qr(i,:) = [-30*abs(sin(pi*i/64)), 60*abs(sin(pi*i/64)), -30*abs(sin(pi*i/64))] .* pi/180;
end

%% Plot Joint angles reference vs actual 
figure(1);
plot(t, q(:,1),'-r',t,qr(:,1),'-g');
grid on;
title('Ankle joint Refence angle vs Actual angle WRT frame according to DH conventions ');
xlabel('time (seconds)');
ylabel('Joint angle (units: radians)');
legend('Actual angle','Reference angle');

figure(2);
plot(t, q(:,2),'-r',t,qr(:,2),'-g');
grid on;
title('Knee joint Refence angle vs Actual angle WRT frame according to DH conventions ');
xlabel('time (seconds)');
ylabel('Joint angle (units: radians)');
legend('Actual angle','Reference angle');

figure(3);
plot(t, q(:,3),'-r',t,qr(:,3),'-g');
grid on;
title('Hip joint Refence angle vs Actual angle WRT frame according to DH conventions ');
xlabel('time (seconds)');
ylabel('Joint angle (units: radians)');
legend('Actual angle','Reference angle');


%% Compute COM position

% compute COM posirion for actual and reference joint angles given
[Yref, Zref] = COMPosition(qr);
[Yact, Zact] = COMPosition(q);

% plot the COM path of both actual and reference angles
figure(4);
plot(Yref, Zref,'-g',Yact, Zact, '-r');
grid on;
title('COM path Actual vs Reference WRT frame {F}');
legend('Reference path','Actual path');
xlabel('y-axis (units: centimeters)');
ylabel('z-axis (units: centimeters)');

%% Compute the veecity of joints (both reference and actual)

for i=1:length(t)-1
    qd(i,:) = [(q(i+1,1)-q(i,1))/(t(i+1)-t(i)), (q(i+1,2)-q(i,2))/(t(i+1)-t(i)), (q(i+1,3)-q(i,3))/(t(i+1)-t(i))];
    qdr(i,:) = [(qr(i+1,1)-qr(i,1))/(t(i+1)-t(i)), (qr(i+1,2)-qr(i,2))/(t(i+1)-t(i)), (qr(i+1,3)-qr(i,3))/(t(i+1)-t(i))];
end

% compute the COM velocity of both actual and reference joint velocities
% ans positions
[Ydact, Zdact] = COMVelocity(q(2:end,:),qd);
[Ydref, Zdref] = COMVelocity(qr(2:end,:),qdr);

%% plot COM velocity vs position with the stability boundaries

foot_l = 11.7;
ww = (9.81/0.4338)^0.5;
xx = 0:0.2:10;
xxd = -ww*xx;

figure(5);
plot(Yref(2:end), Ydref, '-g',Yact(2:end), Ydact, '-b');
grid on;
title('COM x xdot plane plot WRT frame {F}');
xlabel('COM position along y-axis (units: centimeters)');
ylabel('COM velocity along y-axis (units: centimeters/second)');
legend('Reference trajectory in x xdot plane', 'Actual trajectory in x xdot plane');

figure(6);
plot(Yref(2:end), Ydref, '-g', Yact(2:end), Ydact, '-b', xx,xxd,'-r', xx,xxd+ww*foot_l,'-r');
grid on;
title('COM x xdot plane plot WRT frame {F}');
xlabel('COM position along y-axis (units: centimeters)');
ylabel('COM velocity along y-axis (units: centimeters/second)');
legend('Reference trajectory in x xdot plane', 'Actual trajectory in x xdot plane', 'Boundary conditions');

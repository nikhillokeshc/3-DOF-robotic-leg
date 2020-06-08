function [x,y] = COMPosition(q)

%dimensions in cm.
foot_l = 11.7;
foot_w = 5;
foot_h = 2;
support_w = 4;
support_l = 2.5;
support_h = 5.5;
motor_w = 3.3;
motor_l = 3;
motor_h = 5.4;
link_r = 0.6;
link1_l = 19.4;
link2_l = 15;
hip_l = 9;
hip_w = 6;
hip_h = 4;

%mass and inertia
%damping and stiffness
surface_damping = 1e2;
surface_stiffness = 1e4;

foot_m = 20; 
support_m = 20;
motor_m = 57.2;
link_m = 20;
hip_m = 30;

q_gain = 1e2;
V = 12;

%%
l0 = foot_h+support_h;
l1 = motor_h+link1_l+support_h;
l2 = motor_h+link2_l+support_h;
l3 = motor_h+hip_h;

a1 = (motor_h/2*motor_m + (motor_h+link1_l/2)*link_m + (motor_h+link1_l+support_h/2)*support_m)/(motor_m+link_m+support_m);
a2 = (motor_h/2*motor_m + (motor_h+link2_l/2)*link_m + (motor_h+link2_l+support_h/2)*support_m)/(motor_m+link_m+support_m);
a3 = (motor_h/2*motor_m+(motor_h+hip_h/2)*hip_m)/(motor_m+hip_m);

m0 = foot_m+support_m;
m1 = motor_m+link_m+support_m;
m2 = m1;
m3 = motor_m+hip_m;

x0 = (foot_m*(foot_h/2)+support_m*(foot_h+support_h/2))/(support_m+foot_m);
y0 = (foot_m*(-foot_l/2)+support_m*(-support_l/2))/(foot_m+support_m);
x1 = a1*cos(q(:,1))+l0;
y1 = a1*sin(q(:,1))-support_l/2;
x2 = l0+l1*cos(q(:,1))+a2*cos(q(:,1)+q(:,2));
y2 = -support_l/2+l1*sin(q(:,1))+a2*sin(q(:,1)+q(:,2));
x3 = l0+l1*cos(q(:,1))+l2*cos(q(:,1)+q(:,2))+a3*cos(q(:,1)+q(:,2)+q(:,3));
y3 = -support_l/2+l1*sin(q(:,1))+l2*sin(q(:,1)+q(:,2))+a3*sin(q(:,1)+q(:,2)+q(:,3));

Xcom = (m0*x0+m1*x1+m2*x2+m3*x3)/(m0+m1+m2+m3);
Ycom = (m0*y0+m1*y1+m2*y2+m3*y3)/(m0+m1+m2+m3);

x = -Ycom;
y = Xcom;
end
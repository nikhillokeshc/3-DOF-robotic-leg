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

%%
L1 = Link('d',0,'a',0,'alpha',0);
L2 = Link('d',0,'a',0,'alpha',pi/2);
L3 = Link('d',0,'a',0,'alpha',-pi/2);
L4 = Link('d',0,'a',foot_h+support_h,'alpha',0);

L5 = Link('d',0,'a',support_h+motor_h+link1_l,'alpha',0);
L6 = Link('d',0,'a',support_h+motor_h+link2_l,'alpha',0);
L7 = Link('d',0,'a',motor_h+hip_h,'alpha',0);

Leg = SerialLink([L1 L2 L3 L4 L5 L6 L7],'name', 'Robotic leg');
qh = [0 0 pi/2 0 0 0 0];

%%
qt = qh + [0 0 0 0 pi/6 -pi/3 pi/6];
[q1,q1d,q1dd] = mtraj(@lspb, qh, qt, 6);
[q2,q2d,q2dd] = mtraj(@lspb, qt, qh, 6);

p = [q1;q2]; pd = [q1d;q2d]; pdd = [q1dd;q2dd];

%%
q = -[p(:,5), p(:,6), p(:,7)];
qd = -[pd(:,5), pd(:,6), pd(:,7)];
qdd = -[pdd(:,5), pdd(:,6), pdd(:,7)];
%%
figure(1);
subplot(311);
plot(q(:,1));
subplot(312);
plot(qd(:,2));
subplot(313);
plot(qdd(:,3)); 
grid on;

figure(2);
[y,z] = COMPosition(q);
[yd,zd] = COMVelocity(q,qd);
plot(y,yd);
hold on;
foot_l = 11.7;
ww = (9.81/0.4338)^0.5;
xx = 0:0.2:10;
xxd = -ww*xx;
plot(xx,xxd,'-r');
plot(xx,xxd+ww*foot_l,'-r');

%%
T = Leg.fkine(p);
about T
figure(3);
Leg.plot(p);
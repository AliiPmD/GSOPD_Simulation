% The Main Program
% Robot Parameters
p1 = 8.77;
p2 = 0.51;
p3 = 0.76;
p4 = 0.62;
p5 = 74.48;
p6 = 6.174;

kp1 = 80;
kp2= 100;
kd1= 33;
kd2= 10;
t=0;
K0 = 45;
K1 = 40;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initial Conditions
q_10 = 3;
q_20 = -2;
dq_10 = 0;
dq_20 = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q_d1 = sin(pi*t); % desired set point for joint 1
q_d2 = sin(pi*t); % desired set point for joint 2

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation Parameters
T_f = 10; % Simulation Interval
AT = 1e-6; % Absolute Tolerance
RT = 1e-6; % Relative Tolerance
RF = 4; % Refine Factor

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Start Simualtion
q_1 = [];
q_2 = [];
dq_1 = [];
dq_2 = [];
u_1 = [];
u_2 = [];
q_tilde1=[];
q_tilde2 =[];
out = sim('controller');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot results
figure
title('Position Tracking Error')
subplot(211)
plot(out.tout,out.q_tilde1,'black','LineWidth',1.5)
hold on
plot(out.tout,q_d1*ones(length(out.tout),1),'r--')
ylabel(' Position errors (rad) ')
legend('q tilde_1','')
ylim([-2 3])
grid on
subplot(212)
plot(out.tout,out.q_tilde2,'black','LineWidth',1.5) 
hold on
plot(out.tout,q_d2*ones(length(out.tout),1),'r--')
ylabel(' Position errors (rad) ')
legend('q tilde_2','')
xlabel(' Time (s) ')
ylim([-3 1])
grid on

%--------------------------------
figure
title('Requested inputs of GSOPD control')
subplot(211)
plot(out.tout,out.u_1,'black','LineWidth',1.5)
ylabel(' Input torque (Nm) ')
legend('Joint 1')
ylim([-100 100])
grid on

subplot(212)
plot(out.tout,out.u_2,'black','LineWidth',1.5)
ylabel(' Input torque (Nm) ')
legend('Joint 2')
ylim([-50 150])
xlabel(' Time (s) ')
grid on





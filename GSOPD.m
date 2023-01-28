function [f]=GSOPD(x)
q_1 = x(1);
q_2 = x(2);
dq_1 = x(3);
dq_2 = x(4);

p1 = x(5);
p2 = x(6);
p3 = x(7);
p4 = x(8);
p5 = x(9);
p6 = x(10);

kp1 = x(11);
kp2 = x(12);
kd1 = x(13);
kd2 = x(14);
K0 = x(15);
K1 = x(16);

qd1 = x(17);
qd2 = x(18);
dqd1 = x(19);
dqd2 = x(20);
ddqd1 = x(21);
ddqd2 = x(22);

qc0 = x(23);
qc1 = x(24);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robot_parameters = [];
robot_parameters(1) = p1;
robot_parameters(2) = p2;
robot_parameters(3) = p3;
robot_parameters(4) = p4;
robot_parameters(5) = p5;
robot_parameters(6) = p6;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Position, Velovity and Desired Position Vectors
q = [ q_1; q_2];
dq = [dq_1; dq_2];
q_d = [ qd1; qd2];
dqd = [dqd1;dqd2];
ddqd = [ddqd1;ddqd2];

K_P = diag([kp1 kp2]);
K_D = diag([kd1 kd2]);
K0 = diag([K0  K0]);
K1 = diag([K1  K1]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Error Vector
q_tilde = q-q_d; %% Same as 'e' in paper

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
alpha = 0.03;
qc = [qc0;qc1];
r = qc - K0* q_tilde;
dqc = -1*alpha *K0 *tanh(q_tilde) - (alpha*K0 + K1) * r;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GSOPD
[M,C,G] = dynamic_terms(q,dq,robot_parameters);
u = M*ddqd + C*dqd + G + K_D * r - K_P* tanh(q_tilde);

f = [u;dqc;q_tilde];
end
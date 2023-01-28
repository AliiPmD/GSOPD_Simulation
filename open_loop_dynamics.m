function dx = open_loop_dynamics(x)
% Joint Positions
q_1 = x(1);
q_2 = x(2);
% Joint Velocities
dq_1 = x(3);
dq_2 = x(4);
% Joint Torques
u_1 = x(5);
u_2 = x(6);
% Robot Parameters
p1 = x(7);
p2 = x(8);
p3 = x(9);
p4 = x(10);
p5 = x(11);
p6 = x(12);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Joint Position, Velocity and Torque Vectors
q = [ q_1; q_2];
dq = [dq_1; dq_2];
u = [ u_1; u_2];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robot_parameters = [];
robot_parameters(1) = p1;
robot_parameters(2) = p2;
robot_parameters(3) = p3;
robot_parameters(4) = p4;
robot_parameters(5) = p5;
robot_parameters(6) = p6;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Open-Loop Dynamics of the Manipulator
[M,C,G] = dynamic_terms(q,dq,robot_parameters);
f = [dq; -inv(M)*(C*dq+G)];
g = [zeros(2,2); inv(M)];
dx = f+g*u;
end
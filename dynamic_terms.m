function [D,C,G]=dynamic_terms(q,dq,robot_parameters)
p1 = robot_parameters(1);
p2 = robot_parameters(2);
p3 = robot_parameters(3);
p4 = robot_parameters(4);
p5 = robot_parameters(5);
p6 = robot_parameters(6);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q1 = q(1);
q2 = q(2);
dq1 = dq(1);
dq2 = dq(2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Mass-Inertia Matrix
D = zeros(2,2);
D(1,1) = p1+(2*p2*cos(q2));
D(1,2) = p3+(p2*cos(q2));
D(2,1) = p3+(p2*cos(q2));
D(2,2) = p4;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Corilois and Centrifugal Matrix
C = zeros(2,2);
C(1,1) = -1*p2*sin(q2)*dq2;
C(1,2) = -1*p2*sin(q2)*(dq1+dq2);
C(2,1) = p2*sin(q2)*dq1;
C(2,2) = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gravity Vector
G = zeros(2,1);
G(1) =  p5*sin(q1) + p6*sin(q1+q2);
G(2) =  p6*sin(q1+q2);

end
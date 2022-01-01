addpath ./Robot_Arms_Simulation/

close;
clear;

syms theta1 theta2 theta3 theta4 theta5 theta6 theta7 theta8 halfpi H LX LA LB LC LD;

assume(cos(halfpi)==0);
assume(theta4==0);

% right arm
% A1 = simplify(Tlink(theta1+halfpi, 0, H, -halfpi));
% A1a = simplify(Tlink(-halfpi, 0, LX, -halfpi));
% A2 = simplify(Tlink(theta2 + halfpi, 0, LA, halfpi));
% A3 = simplify(Tlink(theta3, 0, 0, -halfpi));
% A4 = simplify(Tlink(theta4, 0, LB, halfpi));
% A5 = simplify(Tlink(theta5+halfpi, LC, 0, 0));
% A5a = simplify(Tlink(-halfpi, 0, 0, -halfpi));
% A6 = simplify(Tlink(theta6, 0, 0, halfpi));
% A7 = simplify(Tlink(theta7, 0, 0, -halfpi));
% A8 = simplify(Tlink(theta8, 0, LD, 0));

% left arm
A1 = simplify(Tlink(theta1+halfpi, 0, H, -halfpi));
A1a = simplify(Tlink(-halfpi, 0, LX, halfpi));
A2 = simplify(Tlink(theta2 - halfpi, 0, LA, -halfpi));
A3 = simplify(Tlink(theta3, 0, 0, halfpi));
A4 = simplify(Tlink(theta4, 0, LB, -halfpi));
A5 = simplify(Tlink(theta5-halfpi, LC, 0, 0));
A5a = simplify(Tlink(halfpi, 0, 0, halfpi));
A6 = simplify(Tlink(theta6, 0, 0, -halfpi));
A7 = simplify(Tlink(theta7, 0, 0, halfpi));
A8 = simplify(Tlink(theta8, 0, LD, 0));

%%
A14 = simplify(A1*A1a*A2*A3*A4);
A58 = simplify(A5*A5a*A6*A7*A8);

%%
assume(sin(2*halfpi)==0);
A18 = simplify(A14*A58);

assume(sin(halfpi)==1);
A18 = simplify(A18);

assume(cos(theta1+halfpi)==0);
A18 = simplify(A18);

%%
assume(sin(theta1+halfpi)==cos(theta1));
A18 = simplify(A18);

assume(sin(theta2+halfpi)==cos(theta2));
A18 = simplify(A18);

assume(sin(theta3+halfpi)==cos(theta3));
A18 = simplify(A18);

assume(sin(theta5+halfpi)==cos(theta5));
A18 = simplify(A18);

assume(sin(theta6+halfpi)==cos(theta6));
A18 = simplify(A18);

assume(sin(theta7+halfpi)==cos(theta7));
A18 = simplify(A18);

assume(sin(theta8+halfpi)==cos(theta8));
A18 = simplify(A18);

assume(cos(theta2+halfpi)==-sin(theta2));
A18 = simplify(A18);

assume(cos(theta3+halfpi)==-sin(theta3));
A18 = simplify(A18);

assume(cos(theta5+halfpi)==-sin(theta5));
A18 = simplify(A18);

assume(cos(theta6+halfpi)==-sin(theta6));
A18 = simplify(A18);

assume(cos(theta7+halfpi)==-sin(theta7));
A18 = simplify(A18);

assume(cos(theta8+halfpi)==-sin(theta8));
A18 = simplify(A18);

%%
assume(cos(theta1+3*halfpi)==0);
A18 = simplify(A18);

assume(sin(-theta1+halfpi)==cos(theta1));
A18 = simplify(A18);

assume(sin(-theta2+halfpi)==cos(theta2));
A18 = simplify(A18);

assume(sin(-theta3+halfpi)==cos(theta3));
A18 = simplify(A18);

assume(sin(-theta5+halfpi)==cos(theta5));
A18 = simplify(A18);

assume(sin(-theta6+halfpi)==cos(theta6));
A18 = simplify(A18);

assume(sin(-theta7+halfpi)==cos(theta7));
A18 = simplify(A18);

assume(sin(-theta8+halfpi)==cos(theta8));
A18 = simplify(A18);

assume(cos(-theta2+halfpi)==sin(theta2));
A18 = simplify(A18);

assume(cos(-theta3+halfpi)==sin(theta3));
A18 = simplify(A18);

assume(cos(-theta5+halfpi)==sin(theta5));
A18 = simplify(A18);

assume(cos(-theta6+halfpi)==sin(theta6));
A18 = simplify(A18);

assume(cos(-theta7+halfpi)==sin(theta7));
A18 = simplify(A18);

assume(cos(-theta8+halfpi)==sin(theta8));
A18 = simplify(A18);

%%
assume(sin(3*halfpi + theta1)==-cos(theta1));
A18 = simplify(A18);

%%
x = A18(1,4);
y = A18(2,4);
z = A18(3,4);

phi = simplify(atan(A18(2,1)/A18(1,1)));
psi = simplify(atan(A18(3,2)/A18(3,3)));
theta = simplify(atan(-A18(3,1)/sqrt(A18(1,1)^2+A18(2,1)^2)));    

%%
J=jacobian([x, y, z, phi, theta, psi],[theta2 theta3 theta5 theta6 theta7 theta8])
addpath ./Robot_Arms_Simulation/

close;
clear;

syms theta1 theta2 theta3 theta4 theta5 theta6 theta7 theta8 halfpi H LX LA LB LC LD;

assume(cos(halfpi)==0);
assume(theta1==0);
assume(theta4==0);

% right arm
A1 = simplify(Tlink(theta1+halfpi, 0, H, -halfpi));
A1a = simplify(Tlink(-halfpi, 0, LX, -halfpi));
A2 = simplify(Tlink(theta2 + halfpi, 0, LA, halfpi));
A3 = simplify(Tlink(theta3, 0, 0, -halfpi));
A4 = simplify(Tlink(theta4, 0, LB, halfpi));
A5 = simplify(Tlink(theta5+halfpi, LC, 0, 0));
A5a = simplify(Tlink(-halfpi, 0, 0, -halfpi));
A6 = simplify(Tlink(theta6, 0, 0, halfpi));
A7 = simplify(Tlink(theta7, 0, 0, -halfpi));
A8 = simplify(Tlink(theta8, 0, LD, 0));

% left arm
% A1 = simplify(Tlink(theta1+halfpi, 0, H, -halfpi));
% A1a = simplify(Tlink(-halfpi, 0, LX, halfpi));
% A2 = simplify(Tlink(theta2 - halfpi, 0, LA, -halfpi));
% A3 = simplify(Tlink(theta3, 0, 0, halfpi));
% A4 = simplify(Tlink(theta4, 0, LB, -halfpi));
% A5 = simplify(Tlink(theta5-halfpi, LC, 0, 0));
% A5a = simplify(Tlink(halfpi, 0, 0, halfpi));
% A6 = simplify(Tlink(theta6, 0, 0, -halfpi));
% A7 = simplify(Tlink(theta7, 0, 0, halfpi));
% A8 = simplify(Tlink(theta8, 0, LD, 0));

A16 = simplify(A1*A1a*A2*A3*A4*A5*A5a);

%pwx = -LC*cos(theta2)*cos(theta5)*sin(theta3) - LB*cos(theta2)*sin(theta3) - LC*sin(theta5)*cos(theta2)*cos(theta3) - LX;
pwx = LB*cos(theta2)*sin(theta3) - LX + LC*cos(theta2)*cos(theta3)*sin(theta5) + LC*cos(theta2)*cos(theta5)*sin(theta3);

%pwy = LA + LB*cos(theta3) - LC*sin(theta5)*sin(theta3) + LC*cos(theta5)*cos(theta3);
pwy = - LA - LB*cos(theta3) + LC*sin(theta5)*sin(theta3) - LC*cos(theta5)*cos(theta3);

%pwz = H + LB*sin(theta2)*sin(theta3) + LC*sin(theta5)*sin(theta2)*cos(theta3) + LC*cos(theta5)*sin(theta2)*sin(theta3);
pwz = H + LB*sin(theta2)*sin(theta3) + LC*sin(theta5)*sin(theta2)*cos(theta3) + LC*cos(theta5)*sin(theta2)*sin(theta3);

A68 = simplify(A6*A7*A8)
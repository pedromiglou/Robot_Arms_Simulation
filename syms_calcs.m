addpath ./lib/
addpath ./Robot_Arms_Simulation/

close;
clear;

syms theta3 d3 theta4 theta5 d5 theta6 d7 halfpi sx sy sz cx cy cz a b c;

%halfpi=pi/2;
%A3 = rotz(theta3 + halfpi) * trans(0, 0, d3) * rotx(halfpi);
A3 = Tlink(theta3 + halfpi, 0, d3, halfpi);
A3 = simplify(A3)

A4 = Tlink(theta4, 0, 0, -halfpi);
A4 = simplify(A4)

A5 = Tlink(0, 0, d5, halfpi);
A5 = simplify(A5)

A6 = Tlink(theta6 + halfpi, d7, 0, 0);
A6 = simplify(A6)

simplify(A3*A4*A5*A6)

pwz = d3 + d5 + d5*cos(theta4) + d7*cos(theta4 + theta6);

pwx = d7*sin(theta6)*sin(theta3)*cos(theta4) + d7*cos(theta6)*sin(theta3)*sin(theta4) + d5*sin(theta3)*sin(theta4);

pwy = - d7*sin(theta6)*cos(theta3)*cos(theta4) - d5*cos(theta3)*sin(theta4) - d7*cos(theta6)*cos(theta3)*sin(theta4);

pwx = - d7*sin(theta6)*cos(theta4)*cos(theta5) - d7*cos(theta6)*sin(theta4) - d5*sin(theta4);

pwy = d5*cos(theta4) + d7*cos(theta6)*cos(theta4) - d7*sin(theta6)*cos(theta5)*sin(theta4);

pwz = d7*sin(theta6)*sin(theta5);

%solve(a == - d7*sz*cx*cy - d7*cz*sx - d5*sx, b == d5*cx + d7*cz*cx - d7*sz*cy*sx, c == d7*sz*sy, sx^2+cx^2==1, sy^2+cy^2==1, sz^2+cz^2==1, sx, sy, sz, cx, cy, cz, 'IgnoreAnalyticConstraints', true, 'ReturnConditions', true)

%3,3
%sin(theta4)*sin(theta5)

%3,4
%r34 = d3 + d5*cos(theta4) + d7*sin(halfpi + theta6)*cos(theta4) + d7*cos(halfpi + theta6)*cos(theta5)*sin(theta4);
%r34 = simplify(r34)

%2,4
%r24 = d7*cos(halfpi + theta6)*(cos(theta5)*(sin(halfpi + theta3)*cos(theta4)) + sin(theta5)*cos(halfpi + theta3) - d5*(sin(halfpi + theta3)*sin(theta4)) - d7*sin(halfpi + theta6)*sin(halfpi + theta3)*sin(theta4) );
%r24 = simplify(r24)

%1,4
%r14 = d7*cos(halfpi + theta6)*(cos(theta5)*(cos(halfpi + theta3)*cos(theta4)) - sin(theta5)*sin(halfpi + theta3)) - d5*(cos(halfpi + theta3)*sin(theta4)) - d7*sin(halfpi + theta6)*(cos(halfpi + theta3)*sin(theta4));
%r14 = simplify(r14)

%simplify(r24-r14)

syms theta7 theta8 theta9 d9
A7 = Tlink(theta7, 0, 0, halfpi);
A8 = Tlink(theta8, 0, 0, -halfpi);
A9 = Tlink(theta9, 0, d9, 0);

A7*A8*A9
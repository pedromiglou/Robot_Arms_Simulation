addpath ./lib/
addpath ./Robot_Arms_Simulation/

close;
clear;

axis equal; axis([-2000 4000 -2000 2000 0 4000]); hold on; view(3);
grid on;

xlabel("X"); ylabel("Y"); zlabel("Z");

[HTA, HTB, STF, LTF, DTF, DTT, LTT, WTS, HTC, LBL, WBL, HBL, H, LD, LC, LB, LA, LX, LZ] = ReadParameters("tp2.txt");

xmin=0; xmax=LTF; ymin=0; ymax=WTS; zmin=0; zmax=HTB;
plot3([xmin,xmin],[ymin,ymin],[zmin,zmax],'-r')
plot3([xmax,xmax],[ymin,ymin],[zmin,zmax],'-r')
plot3([xmax,xmax],[ymax,ymax],[zmin,zmax],'-r')
plot3([xmin,xmin],[ymax,ymax],[zmin,zmax],'-r')
fill3([xmin xmax xmax xmin], [ymin ymin ymax ymax], [zmax zmax zmax zmax], 'r');

xmin=0; xmax=LTF; ymin=WTS+STF; ymax=2*WTS+STF; zmin=0; zmax=HTA;
plot3([xmin,xmin],[ymin,ymin],[zmin,zmax],'-r')
plot3([xmax,xmax],[ymin,ymin],[zmin,zmax],'-r')
plot3([xmax,xmax],[ymax,ymax],[zmin,zmax],'-r')
plot3([xmin,xmin],[ymax,ymax],[zmin,zmax],'-r')
fill3([xmin xmax xmax xmin], [ymin ymin ymax ymax], [zmax zmax zmax zmax], 'r');

xmin=LTF+DTF+DTT; xmax=LTF+DTF+DTT+LTT; ymin=(WTS+STF)/2; ymax=ymin+WTS; zmin=0; zmax=HTC;
plot3([xmin,xmin],[ymin,ymin],[zmin,zmax],'-r')
plot3([xmax,xmax],[ymin,ymin],[zmin,zmax],'-r')
plot3([xmax,xmax],[ymax,ymax],[zmin,zmax],'-r')
plot3([xmin,xmin],[ymax,ymax],[zmin,zmax],'-r')
fill3([xmin xmax xmax xmin], [ymin ymin ymax ymax], [zmax zmax zmax zmax], 'r');

% draw robot
% DH = [ 0 0 H pi/2
%     pi/2 0 LX pi/2
%     pi/2 0 LA pi/2
%     0 0 0 -pi/2
%     0 0 LB pi/2
%     0 0 0 -pi/2
%     0 0 LC pi/2
%     0 0 0 -pi/2
%     0 0 LD 0
%     ];
DH = [ 0 0 H pi/2
    pi/2 0 LX pi/2 %extra
    pi/2 0 LA pi/2
    0 0 0 -pi/2
    0 0 LB pi/2
    pi/2 LC 0 0
    -pi/2 0 0 -pi/2 %extra
    0 0 0 pi/2
    0 0 0 -pi/2
    0 0 LD 0
];

Q = invkin(100,100,700, H, LX, LA,LB,LC,LD);

Qi = [0 0 0 0 0 0 0 0 0 0]';
Qf = invkin(100,100,700, H, LX, LA,LB,LC,LD);
QQ=[Qi(:, 1) Q(:, 1)];
%         DH = [ 0 0 H pi/2
%             pi/2 0 LX pi/2 %extra
%             Q(3,1)+pi/2 0 LA pi/2
%             Q(4,1) 0 0 -pi/2
%             0 0 LB pi/2
%             Q(6,1)+pi/2 LC 0 0
%             -pi/2 0 0 -pi/2 %extra
%             Q(7,1) 0 0 pi/2
%             Q(8,1) 0 0 -pi/2
%             Q(9,1) 0 LD 0
%         ];
% res = Tlinks(DH);
% res(:,:,10) * res(:,:,9) * res(:,:,8) * res(:,:,7) * res(:,:,6) * res(:,:,5) * res(:,:,4) * res(:,:,3) * res(:,:,2) * res(:,:,1)
[H, h, P, AAA] = InitRobot(QQ,50,DH);
while 1
    AnimateRobot(H,AAA,P,h,0.05, true)
end
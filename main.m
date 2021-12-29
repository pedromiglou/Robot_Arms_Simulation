addpath ./Robot_Arms_Simulation/

close;
clear;

axis equal; axis([-3000 3000 -2000 2000 0 4000]); hold on; view(3);
grid on;

xlabel("X"); ylabel("Y"); zlabel("Z");

[HTA, HTB, STF, LTF, DTF, DTT, LTT, WTS, HTC, LBL, WBL, HBL, H, LD, LC, LB, LA, LX, LZ] = ReadParameters("tp2.txt");

xmin=-DTF-LTF; xmax=-DTF; ymin=-WTS-STF/2; ymax=-STF/2; zmin=0; zmax=HTB;
plot3([xmin,xmin],[ymin,ymin],[zmin,zmax],'-r')
plot3([xmax,xmax],[ymin,ymin],[zmin,zmax],'-r')
plot3([xmax,xmax],[ymax,ymax],[zmin,zmax],'-r')
plot3([xmin,xmin],[ymax,ymax],[zmin,zmax],'-r')
fill3([xmin xmax xmax xmin], [ymin ymin ymax ymax], [zmax zmax zmax zmax], 'r');

xmin=-DTF-LTF; xmax=-DTF; ymin=STF/2; ymax=STF/2+WTS; zmin=0; zmax=HTA;
plot3([xmin,xmin],[ymin,ymin],[zmin,zmax],'-r')
plot3([xmax,xmax],[ymin,ymin],[zmin,zmax],'-r')
plot3([xmax,xmax],[ymax,ymax],[zmin,zmax],'-r')
plot3([xmin,xmin],[ymax,ymax],[zmin,zmax],'-r')
fill3([xmin xmax xmax xmin], [ymin ymin ymax ymax], [zmax zmax zmax zmax], 'r');

xmin=DTT; xmax=DTT+LTT; ymin=-WTS/2; ymax=WTS/2; zmin=0; zmax=HTC;
plot3([xmin,xmin],[ymin,ymin],[zmin,zmax],'-r')
plot3([xmax,xmax],[ymin,ymin],[zmin,zmax],'-r')
plot3([xmax,xmax],[ymax,ymax],[zmin,zmax],'-r')
plot3([xmin,xmin],[ymax,ymax],[zmin,zmax],'-r')
fill3([xmin xmax xmax xmin], [ymin ymin ymax ymax], [zmax zmax zmax zmax], 'r');

block= Block(trans(-DTF-LTF+WBL/2, -STF/2-WTS/2, HTB+HBL), LBL, WBL, HBL, 'b');

NN=50;

% right arm
% rightDH = [ pi/2 0 H -pi/2
%     -pi/2 0 LX -pi/2 %extra
%     pi/2 0 LA pi/2
%     0 0 0 -pi/2
%     0 0 LB pi/2
%     pi/2 LC 0 0
%     -pi/2 0 0 -pi/2 %extra
%     0 0 0 pi/2
%     0 0 0 -pi/2
%     0 0 LD 0
% ];
% 
% Qi = [0 0 0 0 0 0 0 0 0 0]';
% Qf = invkinR(100,100,700, H, LX, LA,LB,LC,LD);
% QQ=[Qi(:, 1) Qf(:, 1)];
% 
% [H2, h, P, AAA] = InitRobot(QQ,NN,leftDH);

% left arm
leftDH = [ pi/2 0 H -pi/2
    -pi/2 0 LX pi/2 %extra
    -pi/2 0 LA -pi/2
    0 0 0 pi/2
    0 0 LB -pi/2
    -pi/2 LC 0 0
    +pi/2 0 0 pi/2 %extra
    0 0 0 -pi/2
    0 0 0 pi/2
    0 0 LD 0
];

Qi = [0 0 0 0 0 0 0 0 0 0]';
Qf = invkinL(-DTF-WBL/2,-STF/2-WTS/2,HTB+HBL, H, LX, LA,LB,LC,LD);
QQ=[Qi(:, 1) Qf(:, 1)];

[H2, h, P, AAA] = InitRobot(QQ,NN,leftDH);

for i=1:50
    Org = LinkOrigins(AAA(:,:,:,i));
    h.XData=Org(1,:);
    h.YData=Org(2,:);
    h.ZData=Org(3,:);

    block = block.update(trans(-DTF-LTF+WBL/2+i/50*(LTF-WBL), -STF/2-WTS/2, HTB+HBL));
    pause(0.05);
end

Qi = QQ(:,2);
Qf = invkinL(-DTF/2,-LBL/2,H-LD, H, LX, LA,LB,LC,LD);
QQ=[Qi(:, 1) Qf(:, 1)];
AAA = ObtainRobotMotion(QQ, leftDH, NN);

AnimateRobot(H2,AAA,P,h,0.05, true, block)

Qi = QQ(:,2);
Qf = QQ(:,2);
Qf(1)=pi;
QQ=[Qi(:, 1) Qf(:, 1)];
AAA = ObtainRobotMotion(QQ, leftDH, NN);

AnimateRobot(H2,AAA,P,h,0.05, true, block)

Qi = QQ(:,2);
Qf = invkinL(-DTT-WBL/2,-LBL/2,HTC+HBL, H, LX, LA,LB,LC,LD);
Qf(1)=pi;
QQ=[Qi(:, 1) Qf(:, 1)];
AAA = ObtainRobotMotion(QQ, leftDH, NN);

AnimateRobot(H2,AAA,P,h,0.05, true, block)
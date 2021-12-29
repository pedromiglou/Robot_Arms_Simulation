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

leftBlock= Block(trans(-DTF-LTF+WBL/2, -STF/2-WTS/2, HTB+HBL), LBL, WBL, HBL, 'b');
rightBlock= Block(trans(-DTF-LTF+WBL/2, STF/2+WTS/2, HTA+HBL), LBL, WBL, HBL, 'b');

NN=50;

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

% right arm
rightDH = [ pi/2 0 H -pi/2
    -pi/2 0 LX -pi/2 %extra
    pi/2 0 LA pi/2
    0 0 0 -pi/2
    0 0 LB pi/2
    pi/2 LC 0 0
    -pi/2 0 0 -pi/2 %extra
    0 0 0 pi/2
    0 0 0 -pi/2
    0 0 LD 0
];

[leftH, rightH] = InitRobot(leftDH, rightDH);

leftQQ=[zeros(10,1) invkinL(-DTF-WBL/2,-STF/2-WTS/2,HTB+HBL, H, LX, LA,LB,LC,LD)];
rightQQ=[zeros(10,1) invkinR(-DTF-WBL/2,STF/2+WTS/2,HTA+HBL, H, LX, LA,LB,LC,LD)];

leftAAA = ObtainRobotMotion(leftQQ, leftDH, NN);
rightAAA = ObtainRobotMotion(rightQQ, rightDH, NN);

for i=1:50
    Org = LinkOrigins(leftAAA(:,:,:,i));
    leftH.XData=Org(1,:);
    leftH.YData=Org(2,:);
    leftH.ZData=Org(3,:);

    Org = LinkOrigins(rightAAA(:,:,:,i));
    rightH.XData=Org(1,:);
    rightH.YData=Org(2,:);
    rightH.ZData=Org(3,:);

    leftBlock = leftBlock.update(trans(-DTF-LTF+WBL/2+i/50*(LTF-WBL), -STF/2-WTS/2, HTB+HBL));
    rightBlock = rightBlock.update(trans(-DTF-LTF+WBL/2+i/50*(LTF-WBL), STF/2+WTS/2, HTC+HBL));
    pause(0.05);
end

leftQQ=[leftQQ(:,2) invkinL(-DTF/2,-LBL/2,H-LD, H, LX, LA,LB,LC,LD)];
rightQQ=[rightQQ(:,2) invkinR(-DTF/2,LBL/2,H-LD, H, LX, LA,LB,LC,LD)];
leftAAA = ObtainRobotMotion(leftQQ, leftDH, NN);
rightAAA = ObtainRobotMotion(rightQQ, rightDH, NN);

AnimateRobot(leftAAA, rightAAA, leftH, rightH, 0.05, true, leftBlock, rightBlock);

% Qi = leftQQ(:,2);
% Qf = leftQQ(:,2);
% Qf(1)=pi;
% leftQQ=[Qi(:, 1) Qf(:, 1)];
% leftAAA = ObtainRobotMotion(leftQQ, leftDH, NN);
% 
% AnimateRobot(leftAAA,leftH,0.05, true, leftBlock);
% 
% Qi = leftQQ(:,2);
% Qf = invkinL(-DTT-WBL/2,-LBL/2,HTC+HBL, H, LX, LA,LB,LC,LD);
% Qf(1)=pi;
% leftQQ=[Qi(:, 1) Qf(:, 1)];
% leftAAA = ObtainRobotMotion(leftQQ, leftDH, NN);
% 
% AnimateRobot(leftAAA,leftH,0.05, true, leftBlock);
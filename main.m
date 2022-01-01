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

%% go to 50 units above pickup position
leftQQ=[zeros(10,1) invkinL(-DTF-WBL/2,-STF/2-WTS/2,HTB+HBL+50, H, LX, LA,LB,LC,LD)];
rightQQ=[zeros(10,1) invkinR(-DTF-WBL/2,STF/2+WTS/2,HTA+HBL+50, H, LX, LA,LB,LC,LD)];

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

%% go down 50 units 
N=50;
dr=([0;0;-50;0;0;0])/N;

leftQ=leftQQ(:,2);
leftQQ=leftQQ(:,2);
rightQ=rightQQ(:,2);
rightQQ=rightQQ(:,2);
for n=1:N
    Ji=jacobianLInv(leftQ,H,LX,LA,LB,LC,LD);
    dq =Ji*dr;
    leftQ=leftQ+[0 0 dq(1) dq(2) 0 dq(3) 0 dq(4) dq(5) dq(6) ]';
    leftQQ=[leftQQ leftQ];

    Ji=jacobianRInv(rightQ,H,LX,LA,LB,LC,LD);
    dq =Ji*dr;
    rightQ=rightQ+[0 0 dq(1) dq(2) 0 dq(3) 0 dq(4) dq(5) dq(6) ]';
    rightQQ=[rightQQ rightQ];
end

MDH=GenerateMultiDH(leftDH, leftQQ, zeros(height(leftDH), 1));
leftAAA = CalculateRobotMotion(MDH);
MDH=GenerateMultiDH(rightDH, rightQQ, zeros(height(rightDH), 1));
rightAAA = CalculateRobotMotion(MDH);

AnimateRobot(leftAAA, rightAAA, leftH, rightH, 0.05, true);

%% motion to 50 units away from joining position
leftQQ=[leftQQ(:,end) invkinL(-DTF,-LBL/2-50,H-LD, H, LX, LA,LB,LC,LD)];
%rightQQ=[rightQQ(:,end) invkinR(-DTF/2,LBL/2,H-LD, H, LX, LA,LB,LC,LD)];
rightQQ=[rightQQ(:,end) -leftQQ(:,2)];
leftAAA = ObtainRobotMotion(leftQQ, leftDH, NN);
rightAAA = ObtainRobotMotion(rightQQ, rightDH, NN);

AnimateRobot(leftAAA, rightAAA, leftH, rightH, 0.05, true, leftBlock, rightBlock);

%% 50 units movement to join
N=50;
dr=([0;50;0;0;0;0])/N;

leftQ=leftQQ(:,2);
leftQQ=leftQQ(:,2);
rightQ=rightQQ(:,2);
rightQQ=rightQQ(:,2);
for n=1:N
    Ji=jacobianLInv(leftQ,H,LX,LA,LB,LC,LD);
    dq =Ji*dr;
    leftQ=leftQ+[0 0 dq(1) dq(2) 0 dq(3) 0 dq(4) dq(5) dq(6) ]';
    leftQQ=[leftQQ leftQ];

    Ji=jacobianRInv(rightQ,H,LX,LA,LB,LC,LD);
    dq =Ji*-dr;
    rightQ=rightQ+[0 0 dq(1) dq(2) 0 dq(3) 0 dq(4) dq(5) dq(6) ]';
    rightQQ=[rightQQ rightQ];
end

MDH=GenerateMultiDH(leftDH, leftQQ, zeros(height(leftDH), 1));
leftAAA = CalculateRobotMotion(MDH);
MDH=GenerateMultiDH(rightDH, rightQQ, zeros(height(rightDH), 1));
rightAAA = CalculateRobotMotion(MDH);

AnimateRobot(leftAAA, rightAAA, leftH, rightH, 0.05, true, leftBlock, rightBlock);

%% rotate robot
leftQQ=[leftQQ(:,end) leftQQ(:,end)];
leftQQ(1,:) = [0 pi];
% rightQQ=[rightQQ(:,2) rightQQ(:,2)];
rightQQ = -leftQQ;
rightQQ(1,:) = [0 pi];
leftAAA = ObtainRobotMotion(leftQQ, leftDH, NN);
rightAAA = ObtainRobotMotion(rightQQ, rightDH, NN);

AnimateRobot(leftAAA, rightAAA, leftH, rightH, 0.05, true, leftBlock, rightBlock);

%% put down blocks (DTF,LBL/2,H-LD)->(DTT+WBL/2,LBL/2,HTC+HBL)
N=400;
dr=([DTT+WBL/2-DTF;0;HTC+HBL+50-H+LD;0;0;0])/N;

leftQ=leftQQ(:,2);
leftQQ=leftQQ(:,2);
rightQ=rightQQ(:,2);
rightQQ=rightQQ(:,2);
for n=1:N
    Ji=jacobianLInv(leftQ,H,LX,LA,LB,LC,LD);
    dq =Ji*dr;
    leftQ=leftQ+[0 0 dq(1) dq(2) 0 dq(3) 0 dq(4) dq(5) dq(6) ]';
    leftQQ=[leftQQ leftQ];

    Ji=jacobianRInv(rightQ,H,LX,LA,LB,LC,LD);
    dq =Ji*dr;
    rightQ=rightQ+[0 0 dq(1) dq(2) 0 dq(3) 0 dq(4) dq(5) dq(6) ]';
    rightQQ=[rightQQ rightQ];
end

MDH=GenerateMultiDH(leftDH, leftQQ, zeros(height(leftDH), 1));
leftAAA = CalculateRobotMotion(MDH);
MDH=GenerateMultiDH(rightDH, rightQQ, zeros(height(rightDH), 1));
rightAAA = CalculateRobotMotion(MDH);

AnimateRobot(leftAAA, rightAAA, leftH, rightH, 0.005, true, leftBlock, rightBlock);
function TP2Animation(plotPath, iterations)
    if nargin==0
        plotPath=1;
    end

    if nargin==1
        iterations=2;
    end

    [HTA, HTB, STF, LTF, DTF, DTT, LTT, WTS, HTC, LBL, WBL, HBL, H, LD, LC, LB, LA, LX, LZ] = ReadParameters("tp2.txt");
    
    axis equal; axis([-DTF*2-LTF DTT*2+LTT -max(WTS-STF, LA+LB+LC+LD*2) max(WTS-STF, LA+LB+LC+LD*2) 0 H*1.25]); hold on; view(3);
    grid on;
    
    xlabel("X"); ylabel("Y"); zlabel("Z");
    
    xmin=-DTF-LTF; xmax=-DTF; ymin=-WTS-STF/2; ymax=-STF/2; zmin=0; zmax=HTB;
    plot3([xmin,xmin],[ymin,ymin],[zmin,zmax],'-k')
    plot3([xmax,xmax],[ymin,ymin],[zmin,zmax],'-k')
    plot3([xmax,xmax],[ymax,ymax],[zmin,zmax],'-k')
    plot3([xmin,xmin],[ymax,ymax],[zmin,zmax],'-k')
    fill3([xmin xmax xmax xmin], [ymin ymin ymax ymax], [zmax zmax zmax zmax], [0 0.7 0]);
    
    xmin=-DTF-LTF; xmax=-DTF; ymin=STF/2; ymax=STF/2+WTS; zmin=0; zmax=HTA;
    plot3([xmin,xmin],[ymin,ymin],[zmin,zmax],'-k')
    plot3([xmax,xmax],[ymin,ymin],[zmin,zmax],'-k')
    plot3([xmax,xmax],[ymax,ymax],[zmin,zmax],'-k')
    plot3([xmin,xmin],[ymax,ymax],[zmin,zmax],'-k')
    fill3([xmin xmax xmax xmin], [ymin ymin ymax ymax], [zmax zmax zmax zmax], [0 0.7 0]);
    
    xmin=DTT; xmax=DTT+LTT; ymin=-WTS/2; ymax=WTS/2; zmin=0; zmax=HTC;
    plot3([xmin,xmin],[ymin,ymin],[zmin,zmax],'-k')
    plot3([xmax,xmax],[ymin,ymin],[zmin,zmax],'-k')
    plot3([xmax,xmax],[ymax,ymax],[zmin,zmax],'-k')
    plot3([xmin,xmin],[ymax,ymax],[zmin,zmax],'-k')
    fill3([xmin xmax xmax xmin], [ymin ymin ymax ymax], [zmax zmax zmax zmax], [0 0.7 0]);
    
    leftBlock= Block(trans(-DTF-LTF+WBL, -STF/2-WTS/2, HTB), LBL, WBL, HBL, 'b');
    rightBlock= Block(trans(-DTF-LTF+WBL, STF/2+WTS/2, HTA), LBL, WBL, HBL, 'r');
    
    N=50;
    
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
    
    %draw robot and grippers
    [leftH, rightH] = InitRobot(leftDH, rightDH);
    leftGripper = Gripper(trans(-LX, -LA-LB-LC-LD, H)*rotx(-pi/2), LBL/3, WBL*1.05, min(50, HBL)-5, [0.7 0.7 0.9]);
    rightGripper = Gripper(trans(-LX, LA+LB+LC+LD, H)*rotx(pi/2), LBL/3, WBL*1.05, min(50, HBL)-5, [0.7 0.7 0.9]);
    
    leftQQ = zeros(10,1);
    rightQQ = zeros(10,1);
    
    for iter=1:iterations
        lb_pos = [-DTF-LTF+LBL/2, -STF/2-LBL/2-rand()*(WTS-LBL), HTB, (rand()-0.5)*2*pi];
        rb_pos = [-DTF-LTF+LBL/2, STF/2+LBL/2+rand()*(WTS-LBL), HTA, (rand()-0.5)*2*pi];

        % go to 50 units above pickup position
        leftQQ=[leftQQ(:,end) invkinL(lb_pos(1)+LTF-LBL,lb_pos(2),lb_pos(3)+HBL+50, H, LX, LA,LB,LC,LD, lb_pos(4))];
        rightQQ=[rightQQ(:,end) invkinR(rb_pos(1)+LTF-LBL,rb_pos(2),rb_pos(3)+HBL+50, H, LX, LA,LB,LC,LD, rb_pos(4))];
        
        if width(leftQQ)==1
            error("Left block pickup position outside working space")
        end
        
        if width(rightQQ)==1
            error("Right block pickup position outside working space")
        end
        
        leftAAA = RobotMotion('l', leftQQ, leftDH, N, plotPath);
        rightAAA = RobotMotion('r', rightQQ, rightDH, N, plotPath);
        
        % go down 50 units
        [leftAAA, leftQQ] = JacobianMotion('l', H, LX, LA, LB, LC, LD, N, leftDH, leftQQ, leftAAA, [0;0;-50;0;0;0], "Left block position outside working space", plotPath);
        [rightAAA, rightQQ] = JacobianMotion('r', H, LX, LA, LB, LC, LD, N, rightDH, rightQQ, rightAAA, [0;0;-50;0;0;0], "Right block position outside working space", plotPath);
        
        % motion to 50 units away from joining position
        leftQQ=[leftQQ(:,end) invkinL(-DTT,-LBL/2-50,H-LD, H, LX, LA,LB,LC,LD)];
        rightQQ=[rightQQ(:,end) -leftQQ(:,2)];
        
        if or(width(leftQQ)==1, width(rightQQ)==1)
            error("Block joining position outside working space")
        end
        
        leftAAA = RobotMotion('l', leftQQ, leftDH, N, plotPath, leftAAA);
        rightAAA = RobotMotion('r', rightQQ, rightDH, N, plotPath, rightAAA);
        
        % 50 units movement to join
        [leftAAA, leftQQ] = JacobianMotion('l', H, LX, LA, LB, LC, LD, N, leftDH, leftQQ, leftAAA, [0;50;0;0;0;0], "Join blocks position outside working space", plotPath);
        [rightAAA, rightQQ] = JacobianMotion('r', H, LX, LA, LB, LC, LD, N, rightDH, rightQQ, rightAAA, [0;-50;0;0;0;0], "Join blocks position outside working space", plotPath);
        
        % rotate robot
        leftQQ=[leftQQ(:,end) leftQQ(:,end)];
        leftQQ(1,:) = [0 pi];
        rightQQ=[rightQQ(:,end) rightQQ(:,end)];
        rightQQ(1,:) = [0 pi];
        
        leftAAA = RobotMotion('l', leftQQ, leftDH, N, plotPath, leftAAA);
        rightAAA = RobotMotion('r', rightQQ, rightDH, N, plotPath, rightAAA);
        
        % put down blocks
        dr = [WBL;0;HTC+HBL-H+LD;0;0;0];
        [leftAAA, leftQQ] = JacobianMotion('l', H, LX, LA, LB, LC, LD, N, leftDH, leftQQ, leftAAA, dr, "Put down blocks position outside working space", plotPath);
        [rightAAA, rightQQ] = JacobianMotion('r', H, LX, LA, LB, LC, LD, N, rightDH, rightQQ, rightAAA, dr, "Put down blocks position outside working space", plotPath);
        
        % go up 50 units
        [leftAAA, leftQQ] = JacobianMotion('l', H, LX, LA, LB, LC, LD, N, leftDH, leftQQ, leftAAA, [0;0;50;0;0;0], "Position 50 units above blocks to free them outside working space", plotPath);
        [rightAAA, rightQQ] = JacobianMotion('r', H, LX, LA, LB, LC, LD, N, rightDH, rightQQ, rightAAA, [0;0;50;0;0;0], "Position 50 units above blocks to free them outside working space", plotPath);
        
        % send away block and return robot to original position
        leftQQ=[leftQQ(:,end) invkinL(-DTF-WBL/2, -STF/2-WTS/2, H-LD, H, LX, LA,LB,LC,LD)];
        rightQQ=[rightQQ(:,end) invkinR(-DTF-WBL/2, STF/2+WTS/2, H-LD, H, LX, LA,LB,LC,LD)];
        
        leftAAA = RobotMotion('l', leftQQ, leftDH, N, plotPath, leftAAA);
        rightAAA = RobotMotion('r', rightQQ, rightDH, N, plotPath, rightAAA);

        % Animate
        for n=1:size(leftAAA,4)
            Org = LinkOrigins(leftAAA(:,:,:,n));
            leftH.XData=Org(1,:);
            leftH.YData=Org(2,:);
            leftH.ZData=Org(3,:);
        
            T = eye(4);
            for j = 1:size(leftAAA,3)
                T = T*leftAAA(:,:,j,n);
            end
        
            leftGripper = leftGripper.update(T);
        
            if n <= 50
                leftBlock = leftBlock.update(trans(lb_pos(1)+n/50*(LTF-LBL), lb_pos(2), lb_pos(3))*rotz(lb_pos(4)));
            end
        
            if and(n>100, n<=300)
                leftBlock = leftBlock.update(T);
            end

            if n > 350
                leftBlock = leftBlock.update(trans(DTT+WBL+(n-350)/50*(LTT-2*WBL), LBL/2, HTC));
            end
        
            Org = LinkOrigins(rightAAA(:,:,:,n));
            rightH.XData=Org(1,:);
            rightH.YData=Org(2,:);
            rightH.ZData=Org(3,:);
        
            T = eye(4);
            for j = 1:size(rightAAA,3)
                T = T*rightAAA(:,:,j,n);
            end
        
            rightGripper = rightGripper.update(T);
        
            if n <= 50
                rightBlock = rightBlock.update(trans(rb_pos(1)+n/50*(LTF-LBL), rb_pos(2), rb_pos(3))*rotz(rb_pos(4)));
            end
        
            if and(n>100, n<=300)
                rightBlock = rightBlock.update(T);
            end

            if n > 350
                rightBlock = rightBlock.update(trans(DTT+WBL+(n-350)/50*(LTT-2*WBL), -LBL/2, HTC));
            end
        
            pause(0.05);
        end
    end
end

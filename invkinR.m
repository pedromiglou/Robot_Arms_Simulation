function Q = invkinR(x, y, z, H, LX, LA, LB, LC, LD)
    turned = 0;
    if x>0
        turned=1;
        x=-x;
        y=-y;
    end

    pwx=x;
    pwy=y;
    pwz=z+LD;

    ortogonalLimit = pi*135/180;

    %% theta5
    q5=acos(((pwx+LX)^2+(pwy-LA)^2+(pwz-H)^2-LB^2-LC^2)/(2*LB*LC));

    % if solutions are not real make it an empty array
    if ~isreal(q5)
        q5=[];
    elseif abs(q5)>ortogonalLimit
        q5=[];
    else
        q5=[q5 -q5];
    end

    %% theta3
    aux=LB^2+LC^2+2*LB*LC*cos(q5)-(pwy-LA)^2;

    % check if aux is not negative because sqrt(aux) will be needed 
    if aux(1)<0
        q3=[];
        q5=[];
    else
        q3i=[2*atan2(-LC*sin(q5)+sqrt(aux), LB+LC*cos(q5)+pwy-LA) 2*atan2(-LC*sin(q5)-sqrt(aux), LB+LC*cos(q5)+pwy-LA)];
        q5i=[q5 q5];

        q3 = [];
        q5 = [];
        for i=1:length(q3i)
            if q3i(i) > pi
                q3i(i) = q3i(i) - 2*pi;
            elseif q3i(i) < -pi
                q3i(i) = q3i(i) + 2*pi;
            end

            if abs(q3i(i)) < ortogonalLimit
                q3 = [q3 q3i(i)];
                q5 = [q5 q5i(i)];
            end
        end
    end

    %% theta2
    aux = LB*sin(q3)+LC*sin(q5).*cos(q3)+LC*cos(q5).*sin(q3);
    q2 = atan2((pwz-H)*sign(aux), (-pwx-LX)*sign(aux));

    %% theta6, theta7, theta8
    allDH = zeros(4,4,length(q5));
    for i=1:length(q5)
        DH = [ pi/2 0 H -pi/2
            -pi/2 0 LX -pi/2 %extra
            q2(i)+pi/2 0 LA pi/2
            q3(i) 0 0 -pi/2
            0 0 LB pi/2
            q5(i)+pi/2 LC 0 0
            -pi/2 0 0 -pi/2 %extra
        ];

        res = Tlinks(DH);
        allDH(:,:,i) = res(:,:,1) * res(:,:,2) * res(:,:,3) * res(:,:,4) * res(:,:,5) * res(:,:,6) * res(:,:,7);
    end
    
    p = zeros(4,4,2);
    p(:,:,1) = trans(x,y,z)*rotx(pi);
    p(:,:,2) = trans(x,y,z)*rotz(pi)*rotx(pi);

    res = zeros(2*length(q5),4,4);

    for i=1:2
        for j=1:length(q5)
            res((i-1)*length(q5)+j,:,:) = inv(allDH(:,:,j))*p(:,:,i);
        end
    end
    q2=[q2 q2];
    q3=[q3 q3];
    q5=[q5 q5];

    q7 = [atan2(sqrt(res(:,1,3)'.^2 + res(:,2,3)'.^2), res(:,3,3)') atan2(-sqrt(res(:,1,3)'.^2 + res(:,2,3)'.^2), res(:,3,3)')];
    q6 = [atan2(-res(:,2,3)',-res(:,1,3)') atan2(res(:,2,3)',res(:,1,3)')];
    q8 = [atan2(res(:,3,2)', -res(:,3,1)') atan2(-res(:,3,2)', res(:,3,1)')];
    q2=[q2 q2];
    q3=[q3 q3];
    q5=[q5 q5];

    Q=[zeros(1,length(q5));zeros(1,length(q5));q2;q3;zeros(1,length(q5));q5;zeros(1,length(q5));q6;q7;q8];

    if ~isempty(Q)
        % choose the option with smallest sum
        RowSum = sum(abs(Q),1);
        [~,n] = min(RowSum);
        Q = Q(:,n);

        if turned
            Q(1)=pi;
        end
    end
end


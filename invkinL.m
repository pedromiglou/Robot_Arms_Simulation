function Q = invkinL(x, y, z, H, LX, LA, LB, LC, LD)
    turned = 0;
    if x>0
        turned=1;
        x=-x;
        y=-y;
    end

    pwx=x;
    pwy=y;
    pwz=z+LD;

    %% theta5
    q5=acos(((pwx+LX)^2+(pwy+LA)^2+(pwz-H)^2-LB^2-LC^2)/(2*LB*LC));

    % test if solutions are real: Nan if not!
    if ~isreal(q5)
        q5=nan;
    end

    q5=[q5 -q5];

    %% theta3
    aux=LB^2+LC^2+2*LB*LC*cos(q5)-(pwy+LA)^2;

    q3=[2*atan2(LC*sin(q5)+sqrt(aux), -LB-LC*cos(q5)+pwy+LA) 2*atan2(LC*sin(q5)-sqrt(aux), -LB-LC*cos(q5)+pwy+LA)];
    q5=[q5 q5];

    %% theta2
    q2 = zeros(1, length(q5));
    for i=1:length(q5)
        aux = LB*sin(q3(i))+LC*sin(q5(i))*cos(q3(i))+LC*cos(q5(i))*sin(q3(i));
        q2(i) = atan2((pwz-H)*sign(aux), (pwx+LX)*sign(aux));
    end
%     q2
%     q3
%     q5

    %%
    allDH = zeros(4,4,length(q5));
    for i=1:length(q5)
        DH = [ pi/2 0 H -pi/2
            -pi/2 0 LX pi/2 %extra
            q2(i)-pi/2 0 LA -pi/2
            q3(i) 0 0 +pi/2
            0 0 LB -pi/2
            q5(i)-pi/2 LC 0 0
            +pi/2 0 0 +pi/2 %extra
        ];

        res = Tlinks(DH);
        allDH(:,:,i) = res(:,:,1) * res(:,:,2) * res(:,:,3) * res(:,:,4) * res(:,:,5) * res(:,:,6) * res(:,:,7);
    end
    allDH(:,:,1)
    
    % rotx(pi) ou rotz(pi)*rotx(pi)
    p = zeros(4,4,2);
    %p(:,:,1) = trans(x,y,z)*rotx(pi);
    p(:,:,1) = [
        1 0 0 x
        0 -1 0 y
        0 0 -1 z
        0 0 0 1
        ];
    %p(:,:,2) = trans(x,y,z)*rotz(pi)*rotx(pi);
    p(:,:,2) = [
        -1 0 0 x
        0 1 0 y
        0 0 -1 z
        0 0 0 1
        ];

    res = zeros(2*length(q5),4,4);

    for i=1:2
        for j=1:length(q5)
            res((i-1)*length(q5)+j,:,:) = inv(allDH(:,:,j))*p(:,:,i);
        end
    end
    q2=[q2 q2];
    q3=[q3 q3];
    q5=[q5 q5];

    %% theta6, theta7, theta8
    q7 = [atan2(sqrt(res(:,1,3)'.^2 + res(:,2,3)'.^2), res(:,3,3)') atan2(- sqrt(res(:,1,3)'.^2 + res(:,2,3)'.^2), res(:,3,3)')];
    q6 = [atan2(res(:,2,3)',res(:,1,3)') atan2(-res(:,2,3)',-res(:,1,3)')];
    q8 = [atan2(res(:,3,2)', -res(:,3,1)') atan2(-res(:,3,2)', res(:,3,1)')];
    q2=[q2 q2];
    q3=[q3 q3];
    q5=[q5 q5];

%     q2
%     q3
%     q5
%     q6
%     q7
%     q8

    Q=[zeros(1,16);zeros(1,16);q2;q3;zeros(1,16);q5;zeros(1,16);q6;q7;q8];

    for i=1:size(Q,1)
        for j=1:size(Q,2)
            if Q(i,j)>pi
                Q(i,j) = Q(i,j) - 2*pi;
            end
            if Q(i,j)<-pi
                Q(i,j) = Q(i,j) + 2*pi;
            end
        end
    end

    if ~any(isnan(Q))
        RowSum = sum(abs(Q),1);
        [~,n] = min(RowSum);
        Q = Q(:,n);
    end

    if turned
        Q(1)=pi;
    end
end


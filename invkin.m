function Q = invkin(x, y, z, d1, d2, d3, d5, d7, d9)
    pwx=x;
    pwy=y;
    pwz=z+d9;

    %% theta6
    q6A=acos((pwx^2+pwy^2+(pwz-d3-d5)^2-d5^2-d7^2)/(2*d5*d7));

    % test if solutions are real: Nan if not!
    if ~isreal(q6A)
        q6A=nan;
    end

    q6=[q6A -q6A]

    %% theta4
    aux=d5^2+d7^2+2*d5*d7*cos(q6)-(pwz-d3-d5)^2;

    q4A=2*atan2(-d7*sin(q6)+aux, d7*cos(q6)+pwz-d3);
    q4B=2*atan2(-d7*sin(q6)-aux, d7*cos(q6)+pwz-d3);
    q4=[q4A q4B]
    q6=[q6 q6];

    %% theta3
    q3 = zeros(1, length(q6));
    for i=1:length(q6)
        aux = d7*sin(q6(i))*cos(q4(i))+d5*sin(q4(i))+d7*cos(q6(i))*sin(q4(i));
        q3(i) = atan2(pwx*-sign(aux), pwy*sign(aux));
    end
    q3
    q4
    q6

    %%
%     DH = [ 0 0 d1 pi/2
%         pi/2 0 d2 pi/2 %extra
%         pi/2 0 d3 pi/2
%         0 0 0 -pi/2
%         0 0 d5 pi/2
%         pi/2 d7 0 0
%         -pi/2 0 0 -pi/2 %extra
%         0 0 0 pi/2
%         0 0 0 -pi/2
%         0 0 d9 0
%     ];
    allDH = zeros(4,4,length(q6));
    for i=1:length(q6)
        DH = [ 0 0 d1 pi/2
            pi/2 0 d2 pi/2 %extra
            q3(i)+pi/2 0 d3 pi/2
            q4(i) 0 0 -pi/2
            0 0 d5 pi/2
            q6(i)+pi/2 d7 0 0
            -pi/2 0 0 -pi/2 %extra
%             0 0 0 pi/2
%             0 0 0 -pi/2
%             0 0 d9 0
        ];

        res = Tlinks(DH);
        allDH(:,:,i) = res(:,:,7) * res(:,:,6) * res(:,:,5) * res(:,:,4) * res(:,:,3) * res(:,:,2) * res(:,:,1);
    end
    
    % rotx(pi) ou rotz(pi)*rotx(pi)
    p = zeros(4,4,2);
    %p(:,:,1) = trans(x,y,z)*rotx(pi);
    p(:,:,1) = [
        1 0 0 x
        0 1 0 y
        0 0 -1 z
        0 0 0 1
        ];
    %p(:,:,2) = trans(x,y,z)*rotz(pi)*rotx(pi);
    p(:,:,2) = [
        -1 0 0 x
        0 -1 0 y
        0 0 -1 z
        0 0 0 1
        ];

    res = zeros(2*length(q6),4,4);

    for i=1:2
        for j=1:length(q6)
            res((i-1)*length(q6)+j,:,:) = inv(allDH(:,:,j))*p(:,:,i);
        end
    end
    q6=[q6 q6];
    q3=[q3 q3];
    q4=[q4 q4];

    %% theta7, theta8, theta9
    q8 = [atan2(sqrt(res(:,1,3)'.^2 + res(:,2,3)'.^2), res(:,3,3)') atan2(-sqrt(res(:,1,3)'.^2 + res(:,2,3)'.^2), res(:,3,3)')];
    q7 = [atan2(-res(:,2,3)',-res(:,1,3)') atan2(res(:,2,3)',res(:,1,3)')];
    q9 = [atan2(res(:,3,2)', -res(:,3,1)') atan2(-res(:,3,2)', res(:,3,1)')];
    q6=[q6 q6];
    q3=[q3 q3];
    q4=[q4 q4];

    q3
    q4
    q6
    q7
    q8
    q9

    Q=[zeros(1,16);zeros(1,16);q3;q4;zeros(1,16);q6;zeros(1,16);q7;q8;q9];
end


function AnimateRobot(leftAAA, rightAAA, leftH, rightH, sd, leftGripper, rightGripper, leftBlock, rightBlock)
% H - vetor de handles dos objetos (drawframes - seixos)
% AAA - suermatriz de posições
% P - vertices do objeto a representar (seixos)
% h - handle da linha do braço robótico
% sd - valor da pausa para a animação

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
            leftBlock = leftBlock.update(trans(-DTF-LTF+WBL/2+n/50*(LTF-WBL), -STF/2-WTS/2, HTB+HBL));
        end

        if n<100
            leftBlock = leftBlock.update(T*rotx(pi));
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
            rightBlock = rightBlock.update(trans(-DTF-LTF+WBL/2+n/50*(LTF-WBL), STF/2+WTS/2, HTA+HBL));
        end

        if n<100
            rightBlock = rightBlock.update(T*rotx(pi));
        end

        pause(sd);
    end
end

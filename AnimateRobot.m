function AnimateRobot(H,AAA,P,h,sd, plotPath, block)
% H - vetor de handles dos objetos (drawframes - seixos)
% AAA - suermatriz de posições
% P - vertices do objeto a representar (seixos)
% h - handle da linha do braço robótico
% sd - valor da pausa para a animação

    if nargin < 7
        moveBlock=false;
    else
        moveBlock=true;
    end

    for n=1:size(AAA,4)
        Org = LinkOrigins(AAA(:,:,:,n));
        h.XData=Org(1,:);
        h.YData=Org(2,:);
        h.ZData=Org(3,:);

        T = eye(4);
        for j = 1:size(AAA,3)
            T = T*AAA(:,:,j,n);
            Q=T*P;
            H{j}.Vertices=Q(1:3,:)';
        end

        if plotPath
            plot3(T(1,4), T(2,4), T(3,4), '.r');
        end

        if moveBlock
            block = block.update(T*rotx(pi));
        end

        pause(sd);
    end
end


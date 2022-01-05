function AAA = CalculateRobotMotion(MDH, plotPath)
    % AAA - superhipermatriz com as transformações geom. todas
    % MDH - Matrizes de DH para as diversas configurações
    % plotPath - 0 ou 1 consoante deve desenhar o caminho

    AAA = zeros(4,4,size(MDH, 1),size(MDH,3));
    
    for n=1:size(MDH,3)
        AAA(:,:,:,n) = Tlinks(MDH(:,:,n));
    end

    if plotPath
        for i=1:50
            T = eye(4);
            for j = 1:size(AAA,3)
                T = T*AAA(:,:,j,i);
            end
    
            plot3(T(1,4), T(2,4), T(3,4), '.g');
        end
    end
end

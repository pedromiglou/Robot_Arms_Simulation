function AAA = CalculateRobotMotion(MDH)
% AAA - superhipermatriz com as transformações geom. todas
% MDH - Matrizes de DH para as diversas configurações

    AAA = zeros(4,4,size(MDH, 1),size(MDH,3));
    
    for n=1:size(MDH,3)
        AAA(:,:,:,n) = Tlinks(MDH(:,:,n));
    end
end


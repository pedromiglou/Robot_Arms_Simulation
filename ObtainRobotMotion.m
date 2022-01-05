function AAA = ObtainRobotMotion(QQ, DH, NN, plotPath, AAA)
    jTypes = zeros(height(DH), 1);
    
    Qi=QQ(:, 1);
    Qf=QQ(:, 2);
    MQ= LinspaceVect(Qi, Qf, NN);
    
    MDH=GenerateMultiDH(DH, MQ, jTypes);

    if nargin==4
        AAA = CalculateRobotMotion(MDH, plotPath);
    else
        newAAA = zeros(4,4,height(DH),size(AAA,4)+NN);
        newAAA(:,:,:,1:end-NN) = AAA;
        newAAA(:,:,:,end-NN+1:end) = CalculateRobotMotion(MDH, plotPath);
        AAA = newAAA;
    end
end

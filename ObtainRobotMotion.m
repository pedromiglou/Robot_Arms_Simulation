function AAA = ObtainRobotMotion(QQ, DH, NN)
    jTypes = zeros(height(DH), 1);
    
    Qi=QQ(:, 1);
    Qf=QQ(:, 2);
    MQ= LinspaceVect(Qi, Qf, NN);
    
    MDH=GenerateMultiDH(DH, MQ, jTypes);
    AAA = CalculateRobotMotion(MDH);
end


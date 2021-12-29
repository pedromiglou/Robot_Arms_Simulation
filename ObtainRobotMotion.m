function AAA = ObtainRobotMotion(QQ, DH, NN)
    jTypes = zeros(height(DH), 1);
    
    MQ=[];
    for n=1: width(QQ)-1
        Qi=QQ(:, n);
        Qf=QQ(:,n+1);
        NN=NN(min(n, numel(NN))); %
        MQ=[MQ, LinspaceVect(Qi, Qf, NN)];
    end
    
    MDH=GenerateMultiDH(DH, MQ, jTypes);
    AAA = CalculateRobotMotion(MDH);
end


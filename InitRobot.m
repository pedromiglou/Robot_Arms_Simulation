function [H, h, P, AAA] = InitRobot(QQ, NN, DH, jTypes, sScale)
%INITROBOT Summary of this function goes here
%   Detailed explanation goes here
    if nargin <5, sScale = 1; end
    if nargin <4, jTypes = zeros(height(DH), 1); end

    MQ=[];
    for n=1: width(QQ)-1
        Qi=QQ(:, n);
        Qf=QQ(:,n+1);
        NN=NN(min(n, numel(NN))); %
        MQ=[MQ, LinspaceVect(Qi, Qf, NN)];
    end
    
    MDH=GenerateMultiDH(DH, MQ, jTypes);
    AAA = CalculateRobotMotion(MDH);
    AA = AAA(:,:,:,1);
    Org = LinkOrigins(AA);
    h=DrawLinks(Org);
    [P, F] =seixos3(sScale);
    H = DrawFames(AA,P,F);
end


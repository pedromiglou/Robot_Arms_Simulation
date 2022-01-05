function [AAA, QQ] = JacobianMotionR(H, LX, LA, LB, LC, LD, N, DH, QQ, AAA, dr, msg, plotPath)
    dr = dr/N;
    Q=QQ(:,end);
    QQ=QQ(:,end);

    for n=1:N-1
        Ji=jacobianRInv(Q,H,LX,LA,LB,LC,LD);
    
        if isnan(Ji)
            error(msg)
        end
    
        dq =Ji*dr;
        Q=Q+[0 0 dq(1) dq(2) 0 dq(3) 0 dq(4) dq(5) dq(6) ]';
        QQ=[QQ Q];
    end
    
    MDH=GenerateMultiDH(DH, QQ, zeros(height(DH), 1));
    newAAA = zeros(4,4,height(DH),size(AAA,4)+50);
    newAAA(:,:,:,1:end-50) = AAA;
    newAAA(:,:,:,end-49:end) = CalculateRobotMotion(MDH, plotPath);
    AAA = newAAA;
end

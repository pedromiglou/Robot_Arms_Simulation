function Jinv = jacobianRInv(Q, H, LX, LA, LB, LC, LD)
    J = jacobianR(Q, H, LX, LA, LB, LC, LD);
    a = det(J);
    if a == 0
        Jinv=NaN;
    else
        Jinv = inv(J);
    end
end

function Jinv = jacobianLInv(Q, H, LX, LA, LB, LC, LD)
    J = jacobianL(Q, H, LX, LA, LB, LC, LD);
    a = det(J);
    if a == 0
        Jinv=NaN;
    else
        Jinv = inv(J);
    end
end

function [leftH, rightH] = InitRobot(leftDH, rightDH)
    leftAA = Tlinks(leftDH);
    rightAA = Tlinks(rightDH);
    leftH=DrawLinks(LinkOrigins(leftAA));
    rightH=DrawLinks(LinkOrigins(rightAA));
end

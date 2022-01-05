function A = Tlink(theta, l, d, alpha)
    A = rotz(theta)*trans(l, 0, d)*rotx(alpha);
end

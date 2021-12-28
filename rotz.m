function T = rotz(a)
% Returns rotation matrix around z

T = [ cos(a)  -sin(a) 0  0
      sin(a)  cos(a)  0  0
      0       0       1  0
      0       0       0  1
    ];
end

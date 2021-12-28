function T = rotx(a)
% Returns rotation matrix around x

T = [ 1 0      0       0
      0 cos(a) -sin(a) 0
      0 sin(a) cos(a)  0
      0 0      0       1
    ];
end


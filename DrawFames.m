function H = DrawFames(AA, P, F, c)
    % AA - given by Tlinks
    % P, F - vertices and faces of object to draw
    % c - array of colors

    % H - array of all graphic handles
    if nargin<4
        c = ['g' 'b' 'r' 'y' 'c'];
    end

    H = cell(1, size(AA, 3)); %pre-alocate space
    patch('Vertices', P(1:3, :)', 'Faces', F, 'FaceColor', 'w');

    T = eye(4);

    for n=1:size(AA, 3)
        T = T * AA(:,:,n);

        P1 = T*P;
        mycolor=c(max(1, mod(n, numel(c)+1)));
        h = patch('Vertices', P1(1:3, :)', 'Faces', F, 'FaceColor', mycolor);
        H{n} = h;
    end

end


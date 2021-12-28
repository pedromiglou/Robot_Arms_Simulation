function Org = LinkOrigins(AA)
% Org - matrix of links origins/ends
% AA - given by Tlinks
    Org = zeros(3, size(AA, 3)+1);

    T = eye(4);
    for n=1:size(AA,3)
        T = T*AA(:,:,n);
        Org(:,n+1) = T(1:3, 4);
    end
end


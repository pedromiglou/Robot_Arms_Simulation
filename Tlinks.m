function AA = Tlinks(DH)
    AA=zeros(4,4,size(DH,1));
    
    for n=1:size(DH, 1)
        AA(:,:,n) = Tlink(DH(n,1), DH(n, 2), DH(n, 3), DH(n ,4));
    end
end

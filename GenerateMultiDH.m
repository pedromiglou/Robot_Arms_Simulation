function MDH = GenerateMultiDH(DH,MQ,t)
    % DH matriz DH base
    % MQ matriz devolvida por LinespaceVect
    % t vetor dos tipos de junta (0 - rot, 1 - prismatica)

    if nargin < 3
        t = zeros(size(DH,1),1);
    end

    t=~~t;

    MDH=zeros(size(DH,1),size(DH,2), size(MQ, 2));

    for n=1:size(MQ,2)
        MDH(:,:,n) = [DH(:,1)+~t.*MQ(:,n)  DH(:,2) DH(:, 3)+t.*MQ(:,n) DH(:, 4)];
    end
end


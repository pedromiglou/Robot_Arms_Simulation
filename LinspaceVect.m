function MQ = LinspaceVect(Qi, Qf, N)
    MQ = zeros(numel(Qi), N);

    for n=1:numel(Qi)
        MQ(n,:) = linspace(Qi(n), Qf(n), N);
    end
end

function h = DrawLinks(Org)
    h = line(Org(1,:), Org(2,:), Org(3,:));

    h.Color = [0.6 0.6 0.9];
    h.Marker = 'o';
    h.MarkerSize=4;
    h.LineWidth = 2;
end

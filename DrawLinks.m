function h = DrawLinks(Org)
    h = line(Org(1,:), Org(2,:), Org(3,:));

    h.Color = 'b';
    h.Marker = 'o';
    h.MarkerSize=6;
    h.LineWidth = 4;
end

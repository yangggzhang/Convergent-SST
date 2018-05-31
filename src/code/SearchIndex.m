function index = SearchIndex(Vactive, x_peer)
index = 0;
for i = 1:1:length(Vactive)
    for m = 1:1:length(x_peer.pos)
        if(Vactive(i).pos(m) ~= x_peer.pos(m))
            break;
        end
        if m == length(x_peer.pos)
            index = i;
        end
    end
end
            
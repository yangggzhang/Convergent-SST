function flag = CheckEquality(x_peer, x_parent)
flag = true;
if isstruct(x_peer)
    for i = 1:1:length(x_peer.pos)
        if ~(x_peer.pos(i) == x_parent.pos(i))
            flag = false;
        end
    end
else
    flag = false;
end
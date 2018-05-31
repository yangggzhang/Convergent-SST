function Vinactive_temp = PruneLeaf(Vinactive, x_peer)
	Vinactive_temp = Vinactive;
	index = SearchIndex(Vinactive_temp, x_peer);
	Vinactive_temp(index) = [];
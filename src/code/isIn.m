function flag = isIn(V, x)
	index = SearchIndex(V,x);
	flag = ~(index == 0);
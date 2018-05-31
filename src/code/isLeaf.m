function flag = isLeaf(Vactive, Vinactive, x_peer)
	flag = true;

	for i = 1:1:length(Vactive)
		if CheckEquality(Vactive(i).parent, x_peer)
			flag = false;
			break;
		end
	end

	if (flag == true)
		for i = 1:1:length(Vinactive)
			if CheckEquality(Vinactive(i).parent, x_peer)
				flag = false;
				break;
			end
		end
	end
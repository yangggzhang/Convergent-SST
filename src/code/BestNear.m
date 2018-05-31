function x_nearest = BestNear(Vactive, s_sample, delta_v)
	num_index = 0;
	for i = 1:1:length(Vactive)
		dist_temp(i) = fake_dist(Vactive(i), s_sample);
		if dist_temp(i) < delta_v
			num_index = num_index + 1;
			index(num_index) = i;
		end
	end

	if num_index == 0
		index_temp = find(dist_temp == min(dist_temp));
		x_nearest = Vactive(index_temp(1));
	else
		for i = 1:1:num_index
			cost_temp(i) = Vactive(index(i)).cost;
		end
		index_temp = find(cost_temp == min(cost_temp));
		x_nearest = Vactive(index(index_temp));
	end


function x_nearest = BestNear_Naive(Vactive, s_sample)
	[~,num_index] = size(Vactive);
    distance = zeros(1,num_index);
	for i = 1:1:num_index
		distance(i) = fake_dist(Vactive(i), s_sample);
	end

	[~,I] = min(distance);
    x_nearest = Vactive(I(1));
end


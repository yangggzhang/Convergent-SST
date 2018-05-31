function [s_new, index] = Nearest(S, x_new)

	for i = 1:1:length(S)
		dist_temp(i) = dist(x_new,S(i));
	end

	index = find(dist_temp == min(dist_temp));
	s_new = S(index);
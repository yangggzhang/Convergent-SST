function s_sample = Sample(dim, upperlimits, lowerlimits)
	for i = 1:1:dim
		s_sample.pos(i) = rand() * (upperlimits(i)-lowerlimits(i)) + lowerlimits(i);
	end

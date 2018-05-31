function s_sample = Sample_Hill(dim, upperlimits, lowerlimits)
	for i = 1:1:dim
		s_sample.pos(i) = rand() * (upperlimits(i)-lowerlimits(i)) + lowerlimits(i);
	end

	s_sample.pos(i+1) = Mountain_Height(s_sample.pos(1),s_sample.pos(2));
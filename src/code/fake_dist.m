function distance = fake_dist(x,s)
	temp = 0;
	if length(x.pos) == length(s.pos)
		for i = 1:1:length(x.pos)-1
			temp = temp + (x.pos(i) - s.pos(i))^2;
		end
		distance = sqrt(temp);
	else
		distance = 0;
		display('Dimension not matching.');
	end

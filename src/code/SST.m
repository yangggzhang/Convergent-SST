function SST()
	% x0 initial point
	% x data structure: x.pos position of x; x.parent parent of x; x.cost cost from x0 to x
	Vactive = [x0];
	Vinactive = [];
	s0.pos = x0.pos; s0.rep = x0; 
	S = [s0];

	delta_v = 0.1; T_prop = 0.1; delta_s = 0.1;
	NumStep = 1000;
	for i = 1:1:NumStep
		s_sample = Sample(dim, upperlimits, lowerlimits);
		x_nearest = BestNear(Vactive, s_sample, delta_v);
		x_new = MonteCarlo_Prop(x_nearest, T_prop);
		if CollisionFree(x_new)
			s_new = Nearest(S, x_new);
			if dist(x_new, s_new) > delta_s
				temp_s.pos = x_new.pos;
				S = [S, temp_s];
				s_new.pos = x_new.pos;
				s_new.rep = 0;
			end
			x_peer = s_new.rep;
			if x_peer == 0
				S_temp = SetRep(S, s_new, x_new);
				S = S_temp;				
				% S[length(S)].rep = x_new;
				% s_new.rep = x_new;
			elseif x_new.cost < x_peer.cost
				% Remove(x_peer, Vactive);
				index_remove = find(Vactive = x_peer);
				Vactive[index_remove] = [];

				Vinactive = [Vinactive, x_peer];
				S_temp = SetRep(S, s_new, x_new);
				S = S_temp;
				% s_new.rep = x_new;
				Vactive = [Vactive, x_new];
				while (isLeaf(Vactive, Vinactive, x_peer) & isIn(Vinactive, x_peer))
					x_parent = x_peer.parent;
					Vinactive_temp = PruneLeaf(Vinactive, x_peer);
					Vinactive = Vinactive_temp;
					x_peer = x_parent;
				end
			end

		end
	end 
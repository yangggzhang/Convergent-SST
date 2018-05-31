function S_temp = SetRep(S, s_new, x_new)
	S_temp = S;
	index = find(S_temp.pos == s_new.pos);
	S_temp(index).rep = x_new;
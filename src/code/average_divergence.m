function Da = average_divergence(points1,points2,dt)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
Vt1 = estimate_volume(points1);
Vt2 = estimate_volume(points2);
Da = 1/dt*log(Vt2/Vt1);
end


function [tout, yout] = ode1(Fun,tspan,y0,p)
% ODE1  A simple ODE solver.
%   yout = ODE1(F,t0,h,tfinal,y0) uses Euler's
%   method with fixed step size h on the interval
%      t0 <= t <= tfinal
%   to solve
%      dy/dt = F(t,y)
%   with y(t0) = y0
%   Copyright 2014 - 2015 The MathWorks, Inc.
t0 = tspan(1);
h = tspan(2)-tspan(1);
tfinal = tspan(end);
y = y0';
yout = y;
for t = t0 : h : tfinal-h
    s = Fun(t,y',p);
    y = y + h*s';
    yout = [yout;y];
end
tout = [t0 : h : tfinal-h,tfinal];
yout = yout;
return
x = -2:0.05:2;
y = 0:0.05:2.5;
[X,Y] = meshgrid(x,y);
F = 3*Y + sin(X+X.*Y);
surf(X,Y,F);
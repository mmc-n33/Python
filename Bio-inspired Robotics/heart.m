t = linspace(-pi,pi, 50);
X = 0.9*t .* sin( pi * 0.886*sin(t)./t);
Y = -abs(t) .* cos(pi * sin(t)./t)-5;
X = X*2.6;
Y = Y*2.6;
figure
plot(X,Y);

save('heart.mat','X','Y');
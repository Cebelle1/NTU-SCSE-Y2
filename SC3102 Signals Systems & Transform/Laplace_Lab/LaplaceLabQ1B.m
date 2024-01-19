close all
clear all

syms a t
h_t = exp(-2*t)*heaviside(t);
x_t = 3*cos(4*pi*t)*heaviside(t);
Hs = laplace(h_t)
Xs = laplace(x_t);
Ys = Hs*Xs;
y_t = ilaplace(Ys);

t = 0:0.1:100;
figure
fplot(y_t,[0,5])

grid on;

%Steady state gain is magnitude of H(jw)
%our input x(t) = 3cos(4pi*t) so w = 4pi
%then steady state gain = 1/(4pi + 2)
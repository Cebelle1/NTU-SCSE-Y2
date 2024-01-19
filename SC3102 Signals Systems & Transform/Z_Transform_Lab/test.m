syms y(t)
ode = diff(y) + 4*y == exp(-t);
cond = y(0) == 1;
ySol = dsolve(ode, cond);

h_n = iztrans((z+0.5)/(z-1/3));

%transfer function B=num A=den
B = [1 1]
A = [1 -1/3]
y = filter(B,A,[1 0 0 0 0 0])
%or use this
fvtool(B,A)
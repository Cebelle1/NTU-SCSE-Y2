close all
clear all
syms s
syms t
syms a w phi k b   %constants (can be used without defining)

%-----General Laplace or Inverse Laplace--- (cannot use .* for laplace())
%a
h_t_a = exp(-a*t)*cos(w*t+phi)*heaviside(t);
H_s_a = laplace(h_t_a)
%h_t_check = ilaplace(H_s_a)
%H_s_check = laplace(h_t_check)     %is correct, idk why, prof say one

%b
h_t_b_1 = exp(-a*t)*sin(w*t+phi)*heaviside(t);
H_s_b = laplace(h_t_b_1)

%c
H_s_c = k*s/((s+a)*(s-b))
h_s_c = ilaplace(H_s_c)

%d
h_t_d = exp(a*t)*heaviside(t)
H_s_d = laplace(h_t_d)

%---Initial and Final Value-----(have to use .*)
%WITH INIT CONSTANTS
a = 0.1; b = 0.2; k = 3; w = 2; phi = 0.1*pi; t = 0:0.1:100;

%a) h(t) = e^(-at) * cos(wt+phi)*u(t)
h_t_a = exp(-a*t).*cos(w*t+phi).*heaviside(t);

figure;
plot(t,h_t_a,'r')

%initial value and final value for a is both 0
final_val = h_t_a(end)
init_val = h_t_a(1)   %must have positive int (can change to t=-1:0.1:100)
%final_val = limit(h_t, t, inf) (for function without .*)
%init_val = limit(h_t, t, 0)

%b)
h_t_b = exp(-a*t).*sin(w*t+phi).*heaviside(t);
final_val_b = h_t_b(end)    %final = 0
init_val_b = h_t_b(1)       %init =0

%c)
%from the ilaplace above,
h_s_c = (a.*k.*exp(-a*t))/(a + b) + (b*k*exp(b*t))/(a + b);
final_val_c = h_s_c(end)    %final = inf
init_val_c = h_s_c(1)       %init = 3
figure;
plot(t,h_s_c,'g');

%d)
h_t_d = exp(a*t).*heaviside(t);
final_val_d = h_t_d(end)    %final = inf
init_val_d = h_t_d(1)       %init =0

%----plot pole and zero diagram----LOOK FOR POLES ONLY, LEFT OF PLANE CAN
%ALR
%a)
%symbo lab since matlab giving rubbish(fix:use vpa can get same coeff, nvm symbolab btr)
H_s_a = (cos(0.1*pi)*(s+0.1)-2*sin(0.1*pi))/((s+0.1)*(s+0.1)+4)
H_s_a = vpa(H_s_a)  %FIX THE HUGE VALUE
        %og eqn numerator coef   %denominator
H_a = tf([0.95105651 -0.5229283],[1 0.2 4])  %copy from H_s_a the numerator and denominator original coeff

pzmap(H_a); %pole is x, zero is o
%pole on left side hence stable

[r,p,k] = residue([0.95105651 -0.5229283],[1 0.2 4])
%plot only the positive j



%b)
H_s_b = (sin(0.1*pi)*(s+0.1)+2*cos(0.1*pi))/((s+0.1)*(s+0.1)+4);
H_s_b = vpa(H_s_b)
H_b = tf([0.30901699 1.9330147323],[1 0.2 4])
pzmap(H_b); %pole is x, zero is o
%pole on left side hence stable

[r,p,k] = residue([0.30901699 1.9330147323],[1 0.2 4])

%c)
H_s_c = 3*s/((s+0.1)*(s-0.2))
H_c = tf([3 0], conv([1 0.1],[1 -0.2]))
pzmap(H_c)
%1 pole on right side, unstable.

%d)
H_s_d = 1/(s-0.1)
H_d = tf([1], [1 -0.1])
pzmap(H_d)
%pole on right side, unstable.



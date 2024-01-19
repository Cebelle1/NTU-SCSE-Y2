syms s
syms t

%lecture slide example 2 evaluate y(t) given x(t)u(t) and zero initial
%condition >> x(t) = cos(t)u(t)  
%then Y(S) = X(S) x H(S)
Y_s = (s/(s^2+1)*(1/(s+1)));
               %numerator       %denominator 
[r,p,k] = residue([1,0], conv([1,0,1],[1,1]));

%solved using matlab 
w = 1; theta = 0; p1 = -1; endTime =20;

h_t = exp(p1*t)
H_s = laplace(h_t)
x_t = cos(w*t +theta);
X_s = laplace(x_t);
Y_S = X_s*H_s;
y_t = ilaplace(Y_s);

%plot

figure;
tvec = 0:(1/(10*w)):endTime;
ip_x = cos(w*tvec+theta);
plot(tvec,ip_x,'b'); hold on;

transient_ip = -0.5*exp(-1*tvec);
plot(tvec, transient_ip,'g-'); hold on;

H_w = 1/(j*w-p1);
op_y_ss = abs(H_w)*1*cos(w*tvec+angle(H_w)+theta);
plot(tvec, op_y_ss,'go'); hold on;

yt_plot_laplace(t) = y_t;
plot(tvec, vpa(yt_plot_laplace(tvec)), 'r-+'); hold on;

legend('ip=x(t)','op=0.5e^(-1t) transient)', 'op=y(t) steady state');
hold off;


%zero state and zero input
w = 1; theta = 0; p1 = -1; endTime =20;
y0 = 3;
h_t = exp(p1*t)
H_s = laplace(h_t)
x_t = cos(w*t +theta);
X_s = laplace(x_t);
Y_S = (X_s+y0)*H_s;
y_t = ilaplace(Y_s);

%plot

figure;
tvec = 0:(1/(10*w)):endTime;
Y_zs = H_s*(X_s);
y_zs = ilaplace(Y_zs);
Y_zi = H_s*y0;
y_zi = ilaplace(Y_zi);
y_complete = y_zs + y_zi;

ip_x = cos(w*tvec+theta);
plot(tvec,ip_x,'b'); hold on;
yzi_plot(t) = y_zi;
plot(tvec, vpa(yzi_plot(tvec)),'g-'); hold on;
yzs_plot(t) = y_zs;
plot(tvec, vpa(yzs_plot(tvec)),'go'); hold on;
y_complete_plot(t) = y_complete;
plot(tvec, vpa(y_complete_plot(tvec)),'r+-'); hold on;

legend('y=x(t)','y=zero ip','y=zero state','y=complete')
xlabel('time (sec)')
hold off;



%========q3=========
n = [0 0 3 1];
d1 = [1 2 5];
d2 = [1 3];
d = conv(d1,d2);
[r,p,k] = residue(n,d);
%for this qn using symbolab is easier for partial


Y_s = (3*s+1)/((s^2+2*s+5)*(s+3));
y_t = ilaplace(Y_s);
y_t;
ezplot(y_t, [0:10]);



%=====qn4=======
Y_s = (1.5*s + 19.5)/(s^2 +5*s +6);
y_t = ilaplace(Y_s);
[r,p,k] = residue([1.5, 19.5],[1,5,6])
ezplot(y_t, [0:10]);

%qn4b x(t) = 10u(t)
Y_s =60/(s^3+s^2+6*s);
y_t = ilaplace(Y_s);
y_t
[r,p,k] = residue([60],[1 1 6 0])
[b,a] = residue(r,p,k);

ezplot(y_t, [0:10]);

final_val = limit(y_t, t, Inf);
init_val = limit(y_t, t, 0);


%======qn 5========
t = -5:0.1:10;
x_t = exp(-0.4*t).*exp(1i*4*t);
y_t = 0.3348 * exp(-0.4 * t) .* exp(1i * (4 * t - 1.5645));

%input x(t)
figure;
subplot(2,1,1);
plot(t,real(x_t));
hold on;
plot(t, imag(x_t),'--');

%steady state response = e^(s0t)(H(s0))
subplot(2,1,2);
plot(t, real(y_t));
hold on;
plot(t, imag(y_t),'--');
hold off;

%=========



function [Kp, Kd, Kpp, Kdd] = calcula_ganhos (min_fr)

worst_inercia = [0.4246 0.3988 0.09708 0.02215 0.01804 0.01698];
qsi = 1;
wn = (2*pi*min_fr)/10;
Kp = worst_inercia*(wn^2);
Kp = diag (Kp);
Kd = worst_inercia*2*wn;
Kd = diag (Kd);
diagonalp = [wn^2 wn^2 wn^2 wn^2 wn^2 wn^2];
Kpp = diag (diagonalp);
diagonald = [2*wn*qsi 2*wn*qsi 2*wn*qsi 2*wn*qsi 2*wn*qsi 2*wn*qsi];
Kdd = diag(diagonald);
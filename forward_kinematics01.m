clear
close
clc
home

syms th1 th2 th3 L1 L2 L3

L1 =4 ; L2 =3; L3 =2;
% 1)
% th1 = 0; th2 = 0;th3 = 0;
% 2)
% th1 = 10/180*pi; th2 = 20/180*pi;th3 = 50/180*pi;
% 3)
th1 = 90/180*pi; th2 = 90/180*pi;th3 = 90/180*pi;

T01 = [cos(th1) -sin(th1) 0 0;
       sin(th1) cos(th1)  0 0;
       0          0       1 0;
       0          0       0 1];

T12 = [cos(th2) -sin(th2) 0 L1;
       sin(th2) cos(th2)  0 0;
       0          0       1 0;
       0          0       0 1];

T23 = [cos(th3) -sin(th3) 0 L2;
       sin(th3) cos(th3)  0 0;
       0          0       1 0;
       0          0       0 1];

T3H = [1 0 0 L3;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];

T02 = T01 * T12;
T03 = T01 * T12 * T23;
T0H = T03 * T3H;

% T03 = simplify(T03)
% T0H = simplify(T0H)

figure('Position', [-2500, 800, 500, 400])
axis equal
axis([-4 8 -2 6]);
line([T01(1, 4), T02(1, 4)], [T01(2, 4), T02(2, 4)], 'color', 'r', 'linewidth', 2)
line([T02(1, 4), T03(1, 4)], [T02(2, 4), T03(2, 4)], 'color', 'g', 'linewidth', 2)
line([T03(1, 4), T0H(1, 4)], [T03(2, 4), T0H(2, 4)], 'color', 'b', 'linewidth', 2)
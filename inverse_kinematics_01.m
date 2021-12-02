clear all
close all
clc
home

% syms th1 th2 th3 L1 L2 L3

L1 = 4 ; L2 = 3; L3 = 2;

% % 1)
% T0H = [1 0 0 9;
%        0 1 0 0;
%        0 0 1 0;
%        0 0 0 1];
% % 2)
% T0H = [0.5   -0.866  0  7.5373;
%        0.866  0.6    0  3.9266;
%        0      0      1  0;
%        0      0      0  1];
% % 3)
% T0H = [0 1 0 -3;
%       -1 0 0  2;
%        0 0 1  0;
%        0 0 0  1];
% % 4)
% T0H = [0.866  0.5    0  -3.1245;
%       -0.5    0.8    0   9.1674;
%        0      0      1   0;
%        0      0      0   1];

% ----- for desired position & orientation -----
d_phi = 90/180*pi;
d_px = 1;
d_py = 5;
T0H = [cos(d_phi) -sin(d_phi) 0 d_px;
       sin(d_phi)  cos(d_phi) 0 d_py;
               0           0  1    0;
               0           0  0    1];

% ---------Inverse Kinematics -----------
T3H = [1 0 0 L3;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];
T03 = T0H * inv(T3H);

x = T03(1, 4);
y = T03(2, 4);
c_phi = T03(1, 1);
s_phi = T03(2, 1);

c2 = (x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2);

%% s2 > 0
s2 = sqrt(1-c2^2);

th2 = atan2(s2, c2);

k1 = L1 + L2 * c2;
k2 = L2 * s2;

tmp = inv([k1 -k2; k2 k1]) * [x; y];
c1 = tmp(1, 1)
s1 = tmp(2, 1)

th1 = atan2(s1, c1)

phi =  atan2(s_phi, c_phi)
th3 = phi - th1 - th2

th1_deg = th1 * 180/pi
th2_deg = th2 * 180/pi
th3_deg = th3 * 180/pi
phi_deg = phi * 180/pi

% %% s2 < 0
% s2 = -sqrt(1-c2^2);
% 
% th2 = atan2(s2, c2);
% 
% k1 - L1 + L2 * c2;
% k2 = L2 * s2;
% 
% tmp = inv([k1 -k2; k2 k1]) * [x; y];
% c1 = tmp(1, 1);
% s1 = tmp(2, 1);
% 
% th1 = atan2(s1, c1);
% 
% phi =  atan2(s_phi, c_phi);
% th3 = phi - th1 - th2;
% 
% th1_deg = th1 * 180/pi;
% th2_deg = th2 * 180/pi;
% th3_deg = th3 * 180/pi;
% phi_deg = phi * 180/pi;

% --------- Forward Kinematics -----------
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
T03 = T02 * T23;
T0H = T03 * T3H;

% --------- Draw Robot -----------
a = figure(1);
hold on
line([T01(1, 4), T02(1, 4)], [T01(2, 4), T02(2, 4)], 'color', 'r', 'linewidth', 2)
line([T02(1, 4), T03(1, 4)], [T02(2, 4), T03(2, 4)], 'color', 'g', 'linewidth', 2)
line([T03(1, 4), T0H(1, 4)], [T03(2, 4), T0H(2, 4)], 'color', 'b', 'linewidth', 2)
axis([-2 10 -1 6]);
axis equal
figure('Position', [-2000, 800, 500, 400])
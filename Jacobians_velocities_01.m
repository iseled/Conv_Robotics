clear all
close all
clc
home

L1 = 4 ; L2 = 3; L3 = 2;
% 1)
th1 = 10/180*pi; th2 = 20/180*pi; th3 = 30/180*pi;
% 2)
% th1 = 0/180*pi; th2 = 0/180*pi; th3 = 0/180*pi;
% 3)
% th1 = 90/180*pi; th2 = 90/180*pi; th3 = 90/180*pi;
% 4)
TH = [th1; th2; th3];

% dx = 0.2; dy = -0.3; wz = -0.2;
dx = -0.2; dy = 0; wz = 0;
% dx = 0; dy = 0; wz = 0;

dX = [dx; dy; wz];

fx = 1; fy = 2; wz = 3;
F = [fx; fy; wz];

T = 5;     % 5초 간격으로 움직일때 각속도를 구함
% T = 6    % to find the singularity point
dt = 0.1;
t = 0;

datasize = T/dt;

for i = 1 : datasize+1
    % --------- Forward Kinematics ---------------

    T01 = [cos(th1) -sin(th1) 0 0;
           sin(th1) cos(th1)  0 0;
           0          0       1 0;
           0          0       0 1];

    T12 = [cos(th2) -sin(th2) 0 L1;
           sin(th2) cos(th2)  0 0;
           0          0       -1 0;
           0          0       0  1];
    
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

    X = [T0H(1, 4); T0H(2, 4); atan2(T0H(2, 1), T0H(1, 1))];

    z1 = T01(1:3, 3:3);
    z2 = T02(1:3, 3:3);
    z3 = T03(1:3, 3:3);

    p1 = T01(1:3, 4:4);
    p2 = T02(1:3, 4:4);
    p3 = T03(1:3, 4:4);

    pe = T0H(1:3, 4:4);

    Jac_6 = [cross(z1, (pe-p1)) cross(z2, (pe-p2)) cross(z3, (pe-p3))
                           z1                 z2                 z3 ];

    Jac_modified = [Jac_6(1, :); Jac_6(2, :); Jac_6(6, :)];

    s1 = sin(th1);                c1 = cos(th1);
    s12 = sin(th1 + th2);         c12 = cos(th1 + th2);
    s123 = sin(th1 + th2 + th3);  c123 = cos(th1 + th2 + th3);

    Jac = [-L1*s1-L2*s12-L3*s123   -L2*s12-L3*s123   -L3*s123;
            L1*c1+L2*c12+L3*c123    L2*c12+L3*c123    L3*c123;
                               1                 1          1];

    dTH = inv(Jac_modified)*dX;
    tau = Jac_modified'*F;

    save_t(i) = t;
    save_dTH(:, i) = dTH; 
    save_TH(:, i) = TH;
    save_X(:, i) = X;
    save_detJac(i) = det(Jac_modified);
    save_tau(:, i) = tau;

    save_T01(:, i) = T01(1:2, 4);
    save_T02(:, i) = T02(1:2, 4);
    save_T03(:, i) = T03(1:2, 4);
    save_T0H(:, i) = T0H(1:2, 4);

    % ------------update joint angles & time
    TH = TH + dTH * dt;
    th1 = TH(1); th1 = TH(2); th3 = TH(3);   % 각도값을 계속 업데이트함..

    t = t + dt;
end

% figure('Position', [-2500, 700, 14*70, 7*70]); hold on;
% plot(save_t, save_dTH(1,:), 'r'); plot(save_t, save_dTH(2,:), 'g'); plot(save_t, save_dTH(3,:), 'b');
% figure('Position', [-2500, 700, 14*70, 7*70]); hold on;
% plot(save_t, save_TH(1,:), 'r'); plot(save_t, save_TH(2,:), 'g'); plot(save_t, save_TH(3,:), 'b');
% figure('Position', [-2500, 700, 14*70, 7*70]); hold on;
% plot(save_t, save_X(1,:), 'r'); plot(save_t, save_X(2,:), 'g'); plot(save_t, save_X(3,:), 'b');
% figure('Position', [-2500, 700, 14*70, 7*70]); hold on;
% plot(save_t, save_detJac, 'r');
% figure('Position', [-2500, 700, 14*70, 7*70]); hold on;
% plot(save_t, save_tau(1,:), 'r'); plot(save_t, save_tau(2,:), 'g'); plot(save_t, save_tau(3,:), 'b');

% -------- Draw Robot --------------
for i = 1:datasize
    a = figure(6);
    axis([0 9 -1 4]);
    set(a, 'Position', [-2500, 700, 14*70, 7*70])

    tic;

    line([save_T01(1, i), save_T02(1, i)], [save_T01(2, i), save_T02(2, i)], 'color', 'r', 'linewidth', 2)
    line([save_T02(1, i), save_T03(1, i)], [save_T02(2, i), save_T03(2, i)], 'color', 'g', 'linewidth', 2)
    line([save_T03(1, i), save_T0H(1, i)], [save_T03(2, i), save_T0H(2, i)], 'color', 'b', 'linewidth', 2)

    while(toc < 0.1)
        drawnow;
    end

    if i < datasize
        clf;
    end
end
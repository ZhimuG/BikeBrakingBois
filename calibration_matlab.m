clear;
clc;

no_brake = [4.63 4.43 4.35 4.30 4.30 4.28 4.19 4.15 4.14 4.13 4.05 7.84 7.83 7.78 7.79 7.56 7.52 7.40 7.37 7.34 7.28 7.08 7.04 7.01 6.98 6.93 6.76 6.68 6.65 6.65 6.58 6.43 6.35 6.32 6.28 6.24 6.19 6.01 6.00 5.96 5.94 5.88 5.74 5.67 5.66 5.63 5.61 5.47 5.38 5.37 5.33 5.30 5.25 5.13 5.06 5.06 5.04 5.00 4.86 4.81 4.79 4.78 4.75 4.63 4.55 4.55 4.52 4.49 4.43 4.32 4.29 4.29 4.24 4.21 4.10 4.06 4.04 4.02 4.01 3.89 3.84 3.82 3.80 3.77 3.72 3.63 3.60 3.59 3.57 3.53 3.43 3.39 3.38 3.36 3.34 3.24 3.20 3.18 3.16 3.14 ];
brake_20 = [2.30 8.61 4.14 4.13 4.13 4.11 4.03 7.84 7.79 7.76 7.73 7.52 7.42 7.38 7.40 7.24 7.22 7.03 6.97 6.94 6.92 6.85 6.67 6.61 6.58 6.54 6.51 6.35 6.26 6.22 6.20 6.16 6.08 5.91 5.88 5.86 5.84 5.77 5.63 5.57 5.54 5.51 5.50 5.35 5.27 5.24 5.22 5.18 5.12 5.00 4.95 4.93 4.94 4.84 4.73 4.67 4.66 4.64 4.61 4.49 4.42 4.40 4.38 4.35 4.29 4.17 4.15 4.12 4.11 4.07 3.94 3.90 3.88 3.87 3.84 3.74 3.68 3.66 3.64 3.61 3.55 3.47 3.42 3.41 3.40 3.35 3.26 3.21 3.20 3.18 3.16 3.07 3.01 3.00 2.98 2.96 2.90 2.82 2.79 2.77];
brake_30 = [2.74 20.43 10.12 19.73 8.41 5.89 7.72 7.79 7.55 7.60 7.43 7.18 7.27 7.31 6.99 7.04 6.93 6.74 21.21 3.97 6.64 6.44 6.45 19.37 3.75 14.64 3.95 5.91 6.04 5.83 5.92 5.76 5.63 5.59 5.57 13.33 6.85 4.73 20.61 6.08 5.13 5.18 11.47 6.17 4.91 4.83 4.97 4.78 4.79 4.62 4.57 4.54 4.54 8.29 5.07 4.68 8.15 5.32 4.20 4.17 4.04 3.98 4.01 3.91 3.91 3.81 3.70 3.72 3.65 3.68 3.54 3.47 3.42 3.45 3.38 3.34 3.23 3.18 3.18 3.14 3.12 3.01 2.96 2.93 2.90 2.90 2.82 2.72 2.68 2.68 2.66 2.60 2.52 2.46 2.45 4.31 2.68 2.34 2.25 2.23];
t = linspace(30, 3000, 100);

temp = zeros(2,100);
temp(1,:) = no_brake;
temp(2,:) = t;
no_brake = temp;
no_brake(:, 1:11) = [];

temp = zeros(2,100);
temp(1,:) = brake_20;
temp(2,:) = t;
brake_20 = temp;
brake_20(:,1:7) = [];

temp = zeros(2,100);
temp(1,:) = brake_30;
temp(2,:) = t;
brake_30 = temp;
brake_30(:,1:6)=[];
brake_30(:,13:14)=[];
brake_30(:,16:19)=[];
brake_30(:,24:28)=[];
brake_30(:,26:27)=[];
brake_30(:,35:39)=[];
brake_30(:,72:73)=[];

%% Quadratic Fitting
% fit_no = fit(no_brake(2,:)', no_brake(1,:)'.*(2*pi), 'poly2');
% fit_20 = fit(brake_20(2,:)', brake_20(1,:)'.*(2*pi), 'poly2');
% fit_30 = fit(brake_30(2,:)', brake_30(1,:)'.*(2*pi), 'poly2');
% 
% fitted_no = zeros(2,1e4);
% fitted_no(1,:) = linspace(min(no_brake(2,:)), max(no_brake(2,:)), 1e4);
% fitted_no(2,:) = (fitted_no(1,:).^2).*fit_no.p1+fitted_no(1,:).*fit_no.p2+fit_no.p3;
% 
% fitted_20 = zeros(2,1e4);
% fitted_20(1,:) = linspace(min(brake_20(2,:)), max(brake_20(2,:)), 1e4);
% fitted_20(2,:) = (fitted_20(1,:).^2).*fit_20.p1+fitted_20(1,:).*fit_20.p2+fit_20.p3;
% 
% fitted_30 = zeros(2,1e4);
% fitted_30(1,:) = linspace(min(brake_30(2,:)), max(brake_30(2,:)), 1e4);
% fitted_30(2,:) = (fitted_30(1,:).^2).*fit_30.p1+fitted_30(1,:).*fit_30.p2+fit_30.p3;

%% Linear Fitting
fit_no = fit(no_brake(2,:)', no_brake(1,:)'.*(2*pi), 'poly1');
fit_20 = fit(brake_20(2,:)', brake_20(1,:)'.*(2*pi), 'poly1');
fit_30 = fit(brake_30(2,:)', brake_30(1,:)'.*(2*pi), 'poly1');

fitted_no = zeros(2,1e4);
fitted_no(1,:) = linspace(min(no_brake(2,:)), max(no_brake(2,:)), 1e4);
fitted_no(2,:) = (fitted_no(1,:)).*fit_no.p1+fit_no.p2;

fitted_20 = zeros(2,1e4);
fitted_20(1,:) = linspace(min(brake_20(2,:)), max(brake_20(2,:)), 1e4);
fitted_20(2,:) = (fitted_20(1,:)).*fit_20.p1+fit_20.p2;

fitted_30 = zeros(2,1e4);
fitted_30(1,:) = linspace(min(brake_30(2,:)), max(brake_30(2,:)), 1e4);
fitted_30(2,:) = (fitted_30(1,:)).*fit_30.p1+fit_30.p2;

%% Difference Fitting
% fit_no = fit(no_brake(2,:)', no_brake(1,:)'.*(2*pi), 'poly2');
% fit_20 = fit(brake_20(2,:)', brake_20(1,:)'.*(2*pi), 'poly2');
% fit_30 = fit(brake_30(2,:)', brake_30(1,:)'.*(2*pi), 'poly2');
% 
% fitted_no = zeros(2,1e4);
% fitted_no(1,:) = linspace(min(no_brake(2,:)), max(no_brake(2,:)), 1e4);
% fitted_no(2,:) = (fitted_no(1,:).^2).*fit_no.p1+fitted_no(1,:).*fit_no.p2+fit_no.p3;
% 
% fitted_20 = zeros(2,1e4);
% fitted_20(1,:) = linspace(min(brake_20(2,:)), max(brake_20(2,:)), 1e4);
% fitted_20(2,:) = (fitted_20(1,:).^2).*fit_20.p1+fitted_20(1,:).*fit_20.p2+fit_20.p3;
% 
% fitted_30 = zeros(2,1e4);
% fitted_30(1,:) = linspace(min(brake_30(2,:)), max(brake_30(2,:)), 1e4);
% fitted_30(2,:) = (fitted_30(1,:).^2).*fit_30.p1+fitted_30(1,:).*fit_30.p2+fit_30.p3;
% 
% diff_no_10 = fitted_20(2,:) - fitted_no(2,:);
% diff_10_20 = fitted_30(2,:) - fitted_20(2,:);
% diff_no_20 = fitted_30(2,:) - fitted_no(2,:);

%% Plotting
figure('DefaultAxesFontSize',20);
hold on;
plot(no_brake(2,:), no_brake(1,:).*(2*pi),'b.', 'markersize', 15);
plot(brake_20(2,:), brake_20(1,:).*(2*pi),'r.', 'markersize', 15);
plot(brake_30(2,:), brake_30(1,:).*(2*pi),'k.', 'markersize', 15);
plot(fitted_no(1,:), fitted_no(2,:),'b-', 'linewidth', 1.5, 'markersize', 15);
plot(fitted_20(1,:), fitted_20(2,:),'r-', 'linewidth', 1.5, 'markersize', 15);
plot(fitted_30(1,:), fitted_30(2,:),'k-', 'linewidth', 1.5, 'markersize', 15);
% ylim([0, 50]);
legend('No Braking Force', '10 Braking Force', '20 Braking Force','location','northeast');
xlabel('Time $[ms]$', 'Interpreter','latex');
ylabel('$\omega_{wheel}\quad[rad/s]$', 'Interpreter','latex');

% figure();
% hold on;
% plot(fitted_30(1,:), diff_no_10, 'b.');
% plot(fitted_30(1,:), diff_10_20, 'k.');
% plot(fitted_30(1,:), diff_no_20, 'r.');


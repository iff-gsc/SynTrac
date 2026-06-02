function params = airfoilAnalyticSimpleFit(polar,alpha_tr,is_plot)

if nargin < 3
    is_plot = 0;
end
if nargin < 2
    alpha_tr = 0;
end

% indices of points to fit
[~,idx_tr]=min(abs(polar.alpha-alpha_tr));
idx_Delta = ceil(length(polar.alpha)/20);
idx = [idx_tr,idx_tr+idx_Delta];
idx3 = [idx_tr-idx_Delta,idx_tr,idx_tr+2*idx_Delta];

% lift coefficient
params.alpha_0 = deg2rad(polar.alpha(idx(1)) - ...
    diff(polar.alpha(idx))/diff(polar.c_L(idx)) * polar.c_L(idx(1)));
compress_correct = 1/sqrt(abs(1-polar.info.Mach^2));
params.c_L_alpha = diff(polar.c_L(idx)) / deg2rad(diff(polar.alpha(idx))) ...
    / compress_correct;

% pitching moment coefficient
params.x_ac = 0.25 - ...
    (polar.c_m(idx(2))-polar.c_m(idx(1)))/(polar.c_L(idx(2))-polar.c_L(idx(1)));
params.c_m0 = polar.c_m(idx(1)) + ...
    (params.x_ac - 0.25) * polar.c_L(idx(1));

% drag coefficient
% fit 3 parameters from 3 points: c_D_vec = A * [ c_D0; c_D1; c_D2 ]
c_D_vec = polar.c_D(idx3)';
A = [ [1;1;1], deg2rad(polar.alpha(idx3))', (deg2rad(polar.alpha(idx3)')).^2 ];
c_Dx = A \ c_D_vec;
params.c_D0 = c_Dx(1);
params.c_D1 = c_Dx(2);
params.c_D2 = c_Dx(3);

if is_plot

    if min(polar.c_L) > 0
        alpha = [rad2deg(params.alpha_0),polar.alpha];
    else
        alpha = polar.alpha;
    end
    c_L = airfoilAnalyticSimpleCl( params, alpha*pi/180, polar.info.Mach );

    % Pitching moment coefficient
    figure
    plot(polar.alpha,polar.c_m,'bx-')
    hold on
    plot(alpha, airfoilAnalyticSimpleCm( params, c_L ))
    grid on
    xlabel('Angle of attack, deg')
    ylabel('Pitching moment coefficient')
    legend('Data','Model','location','north')

    % Drag coefficient
    figure
    plot(polar.alpha,polar.c_D,'bx-')
    hold on
    plot(alpha,airfoilAnalyticSimpleCd( params, deg2rad(alpha) ))
    grid on
    xlabel('Angle of attack, deg')
    ylabel('Drag coefficient')
    legend('Data','Model','location','north')
    ylim([0,inf])

    % Lift coefficient
    figure
    plot(polar.alpha,polar.c_L,'bx-')
    hold on
    plot(alpha,c_L)
    grid on
    xlabel('Angle of attack, deg')
    ylabel('Lift coefficient')
    legend('Data','Model','location','north')

end

end
% Simple airfoil parameters (SynTrac configuration 2a wing)


% c_L = c_L_alpha * ( alpha - alpha_0 )
% c_D = c_D0 + c_D1*alpha + c_D2*alpha^2
% c_m = c_m0 + x_ac * c_L

% lift curve slope, in 1/rad
c_L_alpha 	= 6.9964;
% zero lift angle of attack, in rad
alpha_0 	= -0.0842;

% drag coefficient at zero angle of attack, dimensionless
c_D0    	= 0.0053;
% c_D1, in 1/rad
c_D1        = 0.0044;
% c_D2, in 1/rad^2
c_D2     	= 0.1885;

% pitching moment coefficient at zero lift with respect to the 25% chord
% position, dimensionless
c_m0        = -0.1236;
% aerodynamic center (ac) position relative to the chord (backward positive,
% 0.25 means that ac is at 25% chord position), dimensionless
x_ac        = 0.2690;


function [omega_s, w_theta, delta_alpha] = ...
    inducedSwirl(Q, v_disk, v_wing, R, rho, r_eval) %#codegen
% inducedSwirl computes the swirl (rotational induced velocity) of a 
% propeller wake.
%
%   The computation is based on the momentum theory. The propeller reaction
%   torque Q is delivered to the slipstream as angular momentum. Assuming
%   the slipstream rotates as a solid body (w_theta = omega_s * r),
%   conservation of angular momentum at the disk gives
%
%       Q = integral_0^R rho*v_disk*(omega_s*r)*r*(2*pi*r) dr
%         = (pi/2)*rho*v_disk*omega_s*R^4 ,
%
%   so that
%
%       omega_s = 2*Q / (pi*rho*R^4*v_disk) .
%
%   The relation is free of any thrust/power coefficient convention; it
%   works straight from the torque in N*m. The model is intended for
%   low-fidelity propeller-wing interaction: the swirl changes the local
%   inflow angle on a wing immersed in the slipstream, with opposite sign
%   on either side of the disk, and so introduces a power-dependent rolling
%   moment. A downwash reduces the local angle of attack.
%
%   The propeller rotation axis is the body +x axis (FRD), and the positive
%   (normal) rotation sense is the one for which the blades move downward
%   on the right side of the aircraft.
%
%   Two axial velocities enter, with different roles:
%
%     v_disk  the throughflow at the disk, V + v_i. It is the mass flux
%             that carries the angular momentum, so it sets the swirl
%             MAGNITUDE. It is fixed by the disk and is not a tuning
%             parameter.
%
%     v_wing  the axial velocity the wing actually sees, used only to turn
%             the swirl into an angle-of-attack change. The wing sits
%             between the disk value (V + v_i) and the fully developed
%             slipstream (V + 2*v_i); choosing v_wing in that range is how
%             the caller places the wing in the developing wake. Setting
%             v_wing = v_disk evaluates right at the disk.
%
%   With v_disk = V + v_i the result is the propeller-plane swirl. The 
%   fully developed wake swirl is roughly twice as large (the same factor
%   as v_i -> 2*v_i); that amplification of the swirl magnitude is NOT 
%   applied here, so for a wing well downstream scale w_theta up
%   accordingly.
%
% Literature:
% [1]   Glauert, H. (1935): Airplane Propellers. In: Durand, W. F. (Ed.):
%       Aerodynamic Theory, Vol. IV. Springer.
%
% [2]   McCormick, B. W. (1995): Aerodynamics, Aeronautics and Flight
%       Mechanics. 2nd ed., Wiley.
%
% [3]   Veldhuis, L. L. M. (2005): Propeller Wing Aerodynamic Interference.
%       PhD thesis, TU Delft.
%
% Syntax: [omega_s, w_theta, delta_alpha] = ...
%               inducedSwirl(Q, v_disk, v_wing, R, rho, r_eval)
%
% Inputs:
%   Q           propeller torque delivered to the air (e.g. Q = P/omega)
%               (scalar), in N*m
%   v_disk      axial throughflow at the disk, V + v_i; sets the swirl
%               magnitude (see description)
%               (scalar), in m/s
%   v_wing      axial velocity seen by the wing, between V + v_i (at the
%               disk) and V + 2*v_i (fully developed wake); used for the
%               angle-of-attack conversion
%               (scalar), in m/s
%   R           propeller radius
%               (scalar), in m
%   rho         air density
%               (scalar), in kg/m^3
%   r_eval      spanwise station(s) of the wing strip(s) along the FRD
%               y-axis, signed: positive to the right, negative to the
%               left. Only meaningful inside the slipstream tube,
%               |r_eval| <= R
%               (scalar or vector), in m
%
% Outputs:
%   omega_s     slipstream angular rate
%               (scalar), in rad/s
%   w_theta     swirl velocity at the strip; for a strip on the FRD y-axis
%               this is the vertical component, positive downward (+z).
%               Right-wing strips (r_eval > 0) see a downwash for the
%               assumed rotation sense (see Notes)
%               (same size as r_eval), in m/s
%   delta_alpha signed change in effective angle of attack from the swirl,
%               positive increases alpha. A downwash reduces alpha, so for
%               the assumed rotation delta_alpha < 0 on the right wing and
%               > 0 on the left wing. Add it onto the local geometric angle
%               of attack
%               (same size as r_eval), in rad
%
% Example:
%   % --- all inputs are SI and come straight from propeller_aero ------
%   V   = 150;                       % flight speed                [m/s]
%   vi  = 1.0;                       % induced velocity (momentum) [m/s]
%   Q   = 2000;                      % propeller torque = P/omega  [N*m]
%   R   = 1.175;                     % propeller radius            [m]
%   rho = 0.65;                      % air density                 [kg/m^3]
%
%   % place the wing in the developing wake: s = 0 at disk, 1 at far wake
%   s      = 0.5;
%   v_disk = V + vi;                 % fixed: throughflow at the disk
%   v_wing = V + (1 + s)*vi;         % blend between V+vi and V+2*vi
%
%   r_eval = linspace(-R, R, 21);    % spanwise stations, +y = right   [m]
%
%   [omega_s, w_theta, delta_alpha] = ...
%       inducedSwirl(Q, v_disk, v_wing, R, rho, r_eval);
%
%   % w_theta(k)     : vertical swirl velocity [m/s] at r_eval(k), + = down
%   % delta_alpha(k) : signed angle-of-attack change [rad] at r_eval(k)
%   % omega_s        : the slipstream's solid-body spin rate
%
% Notes:
%   - Q, V and v_i must refer to the same operating point. From
%     propeller_aero use Q = P/omega and v_i from the momentum block, then
%     v_disk = V + v_i.
%   - v_disk sets the swirl magnitude and should stay at the physical disk
%     throughflow V + v_i. v_wing only sets the angle-of-attack conversion;
%     blend it between V + v_i (at the disk) and V + 2*v_i (far wake) to
%     place the wing in the developing slipstream.
%   - Coordinate / sign convention: FRD body frame (x forward, y right,
%     z down). The function assumes the rotation sense for which the blades
%     descend on the right side. With r_eval signed (+y right), delta_alpha
%     comes out correctly signed on both wings. For the opposite rotation
%     sense, negate r_eval.
%   - Only valid inside the slipstream tube; for |r_eval| > R the linear
%     solid-body profile is unphysical, so mask or zero those strips.
%   - Valid in the normal working state only; the vortex ring state and
%     reverse flow are not covered.
%
% See also: propeller_aero, inducedVelocity, inducedVelocityMomentumTheory


% guard the disk throughflow against division by zero
v_disk = sign(v_disk) * max(abs(v_disk), 1e-6);

% slipstream angular rate from angular-momentum conservation at the disk
% (solid-body swirl); free of any thrust/power coefficient convention
omega_s = 2 * Q / (pi * rho * R^4 * v_disk);

% swirl velocity; on the FRD y-axis this is the vertical component,
% positive downward for right-wing strips (r_eval > 0)
w_theta = omega_s .* r_eval;

% change in effective angle of attack. A downward swirl (w_theta > 0) is a
% downwash and reduces alpha (minus sign), evaluated against the velocity
% the wing sees
delta_alpha = -atan2(w_theta, abs(v_wing));

end
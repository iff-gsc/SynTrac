function [omega_s, w_theta, delta_alpha] = ...
    inducedSwirl(Q, v_disk, v_wing, R, rho, r_eval) %#codegen
% inducedSwirl computes the swirl (rotational induced velocity) of a 
% propeller wake.
%
%   Supports multiple rotors: Q, v_disk, v_wing may be length-N vectors
%   (one entry per rotor); R and rho are scalars shared by all rotors.
%   r_eval is a scalar or length-M vector of spanwise strip positions;
%   contributions from all rotors are superposed at each strip.
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
%               (scalar or length-N vector, one entry per rotor), in N*m
%   v_disk      axial throughflow at the disk, V + v_i; sets the swirl
%               magnitude (see description)
%               (scalar or length-N vector, one entry per rotor), in m/s
%   v_wing      axial velocity seen by the wing, between V + v_i (at the
%               disk) and V + 2*v_i (fully developed wake); used for the
%               angle-of-attack conversion
%               (scalar or length-N vector, one entry per rotor), in m/s
%   R           propeller radius, shared by all rotors
%               (scalar), in m
%   rho         air density, shared by all rotors
%               (scalar), in kg/m^3
%   r_eval      spanwise station(s) of the wing strip(s) along the FRD
%               y-axis, signed: positive to the right, negative to the
%               left. Only meaningful inside the slipstream tube of the
%               relevant rotor, |r_eval| <= R. Scalar or length-M vector,
%               same positions used for every rotor; contributions are
%               superposed.
%               (scalar or length-M vector), in m
%
% Outputs:
%   omega_s     slipstream angular rate, one value per rotor
%               (same size as Q), in rad/s
%   w_theta     swirl velocity summed over all rotors at each strip; for a
%               strip on the FRD y-axis this is the vertical component,
%               positive downward (+z).
%               (same size as r_eval), in m/s
%   delta_alpha signed change in effective angle of attack from the
%               superposed swirl, positive increases alpha.
%               (same size as r_eval), in rad
%
% Example:
%   % --- two rotors, same operating point -------------------------------
%   V   = 150;                       % flight speed                [m/s]
%   vi  = [1.0; 1.2];                % induced velocity per rotor  [m/s]
%   Q   = [2000; 1800];              % torque per rotor            [N*m]
%   R   = 1.175;                     % propeller radius            [m]
%   rho = 0.65;                      % air density                 [kg/m^3]
%
%   s      = 0.5;
%   v_disk = V + vi;                 % throughflow at each disk    [m/s]
%   v_wing = V + (1 + s)*vi;         % developing wake blend       [m/s]
%
%   r_eval = linspace(-R, R, 21)';   % spanwise stations           [m]
%
%   [omega_s, w_theta, delta_alpha] = ...
%       inducedSwirl(Q, v_disk, v_wing, R, rho, r_eval);
%
%   % omega_s        : (2×1) angular rate per rotor         [rad/s]
%   % w_theta        : (21×1) superposed swirl velocity     [m/s]
%   % delta_alpha    : (21×1) superposed alpha change       [rad]
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
%     sense, negate r_eval (or negate Q) for that rotor.
%   - Only valid inside the slipstream tube; for |r_eval| > R the linear
%     solid-body profile is unphysical, so mask or zero those strips.
%   - Valid in the normal working state only; the vortex ring state and
%     reverse flow are not covered.
%   - When multiple rotors overlap the same strip the swirl velocities are
%     summed (linear superposition). This is an approximation; mutual
%     interaction between slipstreams is not modelled.
%
% See also: propeller_aero, inducedVelocity, inducedVelocityMomentumTheory


% --- reshape inputs for unambiguous broadcasting --------------------------
% Q, v_disk, v_wing: (N×1) — one row per rotor
% r_eval:            (1×M) — one column per strip
Q      = Q(:);                          % (N×1)
v_disk = v_disk(:);                     % (N×1)
v_wing = v_wing(:);                     % (N×1)
r_eval = r_eval(:).';                   % (1×M)

% guard the disk throughflow against division by zero
v_disk = sign(v_disk) .* max(abs(v_disk), 1e-6);   % (N×1)

% --- per-rotor slipstream angular rate ------------------------------------
% omega_s(n) = 2*Q(n) / (pi*rho*R^4*v_disk(n))     (N×1)
omega_s = 2 * Q ./ (pi * rho * R^4 .* v_disk);     % (N×1)

% --- swirl velocity: (N×M) -----------------------------------------------
% omega_s (N×1) × r_eval (1×M) → (N×M), one row per rotor, col per strip
w_theta = omega_s .* r_eval;                        % (N×M)

% v_wing (N×1) broadcasts across M columns
delta_alpha = -atan2(w_theta, abs(v_wing));         % (N×M)

end
function V_Wb = wingApplySlipstream( wing, prop_y_pos, prop_ang, ...
                    prop_d, v_i, swirls, k_slip )
% wingApplySlipstream computes the wind at each panel created by propellers
% in front of the wing. Two effects are considered: Induced velocity and
% swirl
% 
% Inputs:
%   wing            wing struct, see wingCreate
%   prop_y_pos      Spanwise positions of the propeller centers (1xN_p
%                   array, where N_p is the number of propellers), in m
%   prop_ang        Incidence angles of the propellers, where negative
%                   values mean downward looking propellers (1xN_p array),
%                   in rad
%   prop_d          Propeller diameters (1xN_p array), in m
%   v_i             Induced velocities of each propeller at the propeller
%                   disk (1xN_p array), in m/s
%   V               Airspeed at each propeller (1xN_p array), in m/s
%   rho             Air density (scalar), in kg/m^3
%   Q               Propeller torques (1xN_p array), in Nm
%   prop_dir        Propeller directions of rotation (1xN_p array)
%                       1:  clockwise seen from behind the aircraft
%                       -1: counter-clockwise seen from behind the wing
%   k_slip          Scalar contraction factor of slipstream tube that is 
%                   impacting the wing leading edge (1xN_p array)
%                   1 corresponds to no contraction; 2 to max. contraction
% 
% Outputs:
%   V_Wb            Additional airspeed vector at each panel of the wing in
%                   body frame (FRD) (3xN_panel array), in m/s
% 
% See also:
%   wingCreate, wingPlotGeometry, wingSetState

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2026 Yannic Beyer
%   Copyright (C) 2026 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

V_Wb = zeros(size(wing.state.external.V_Wb));
r_eval = zeros( 1, size(V_Wb,2) );

n_props = length(prop_y_pos);
for i = 1:n_props
    r_k_slip = 1/sqrt(k_slip(i))*(prop_d(i)/2);
    y_l = prop_y_pos(i) - r_k_slip;    % y-value left end of slipstream
    y_r = prop_y_pos(i) + r_k_slip;    % y-value right end of slipstream
    is_slip = wing.geometry.ctrl_pt.pos(2,:) > y_l & ...
        wing.geometry.ctrl_pt.pos(2,:) < y_r;
    V_Wb(1,is_slip) = -v_i(i)*k_slip(i)*cos(prop_ang(i));
    r_eval(is_slip) = wing.geometry.ctrl_pt.pos(2,is_slip) - prop_y_pos(i);
    V_Wb(2,is_slip) = swirls(i)*sqrt(k_slip(i))*r_eval(is_slip)/(prop_d(i)/2);
    V_Wb(3,is_slip) = swirls(i)*sqrt(k_slip(i))*r_eval(is_slip)/(prop_d(i)/2);
    V_Wb(3,is_slip) = V_Wb(3,is_slip) + v_i(i)*k_slip(i)*sin(prop_ang(i));
end

end
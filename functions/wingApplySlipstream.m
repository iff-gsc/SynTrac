function V_Wb = wingApplySlipstream( wing, prop_y_pos, prop_ang, ...
                    prop_d, v_i, v_t, prop_dir, k_vi, k_vt ) 
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
%   v_i             Induced axial velocities of each propeller at the 
%                   propeller disk (1xN_p array), in m/s
%   v_t             Induced tangential velocities of each propeller at the 
%                   propeller disk (1xN_p array), in m/s
%   prop_dir        Propeller directions of rotation (1xN_p array)
%                       1:  clockwise seen from behind the aircraft
%                      -1: counter-clockwise seen from behind the wing
%   k_vi            a value of either 1 or 0 that decides whether axial
%                   slipstream is applied in PWI model
%   k_vt            a value of either 1 or 0 that decides whether
%                   tangential slipstream is applied in PWI model
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
v_i = k_vi(:).*v_i(:);
v_t = k_vt(:).*v_t(:);

n_props = length(prop_y_pos);
for i = 1:n_props
    r_slip = prop_d(i)/2;
    y_l = prop_y_pos(i) - r_slip;    % y-value left end of slipstream
    y_r = prop_y_pos(i) + r_slip;    % y-value right end of slipstream
    is_slip = wing.geometry.ctrl_pt.pos(2,:) > y_l & ...
        wing.geometry.ctrl_pt.pos(2,:) < y_r;
    r_eval(is_slip) = wing.geometry.ctrl_pt.pos(2,is_slip) - prop_y_pos(i);
    
    % constant swirl distribution
    vt_signed = sign(r_eval(is_slip))*prop_dir(i)*v_t(i);

    % ~ r/R swirl distribution
    %vt_signed = 4/3*r_eval(is_slip)*prop_dir(i)*v_t(i)/r_slip

    V_Wb(1,is_slip) = -v_i(i)*cos(prop_ang(i)) + vt_signed*sin(prop_ang(i));

    V_Wb(3,is_slip) = vt_signed*cos(prop_ang(i)) + v_i(i)*sin(prop_ang(i));    
end

end
% Wing parameters (SynTrac configuration 2a VTP)

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% basic parameters

% wing span (scalar), in m
prm.b         	= 4.16;
% is the wing symmetrical to the xz-plane (scalar bolean)?
prm.is_symmetrical = false;

%% wing segments (s segments)

% spanwise coordinates of wing parts (1x(s+1) array), in -
prm.eta_segments_wing = [0,0.0394,1];
% wing chord (1x(s+1) array), in m
prm.c           = [4.562,4.400,2.801];
% quarter chord line sweepback angle for each section (1xs array), in rad
prm.lambda    	= [deg2rad(90)-0.93,deg2rad(90)-0.84];
% wing section twist with respect to root (1xs array), in rad
prm.epsilon   	= deg2rad([0,0]);
% dihedral angle (abs(nu)<pi/2 !) for each section (1xs array), in rad
prm.nu       	= deg2rad([0,0]); 

%% wing device segments (d device segments)

% spanwise coordinates of wing device segments (1x(d+1) array), in -
prm.eta_segments_device = [0,1];
% section name (dx? char array); two airfoil projects available:
%   1) "airfoilAnalyticSimple..." (see airfoilAnalyticSimpleCreate) <-- recommended!
%   2) "airfoilAnalytic0515..." (see airfoilAnalytic0515LoadParams) <-- good luck!
prm.section = char(repmat(["airfoilAnalyticSimple_params_vtp2a"],3,1));
% relative flap depth for each device segment (1xd array), in -
prm.flap_depth          = [0.39];
% second actuator type (dx? char array); available options:
%   1) "none" no second actuator
%   2) "airfoilMicroTab..." (see airfoilMicroTabLoadParams)
prm.actuator_2_type = char(repmat(["none"],3,1));
% index of the control input (0 means no control input) (1x(2*d) array), in -  
prm.control_input_index = [ 0, 1, 0, 0, 1, 0, 0, 0, 0, 0; ...
                            zeros(1,10) ];

%% coordinates in reference frame

% wing incidence relative x-y-plane of reference frame, in rad
prm.i = deg2rad(0);
% wing rotation about x axis of reference frame, in rad
prm.rot_x = deg2rad(90);
% x position of the wing (leading edge at wing root) in reference frame (scalar), in m
prm.x = -24.337; 
% z position of the wing (leading edge at wing root) in reference frame (scalar), in m
prm.z = -0.936;

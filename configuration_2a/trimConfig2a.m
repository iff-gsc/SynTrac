function [ic,u0,infostruct,lin,lin_red] = trimConfig2a(aircraft,V,h)
% trimConfig2a trims configuration 2a for given airspeed and altitude.
%   Flight condition is straight horizontal flight.
% 
% Inputs:
%   aircraft            aircraft parameters struct, see aircraftCreate
%   V                   Airspeed (scalar), in m/s
%   h                   Altitude (scalar), in m
% 
% Outputs:
%   ic                  Initial conditions struct for Syntrac Configuration
%                       2a
%   u0                  Trimmed control inputs struct
%   infostruct          infostruct from jj_trim
%   lin                 State-space struct from linearization containing
%                       all continuous states, outputs are angular and
%                       linear acceleration
%   lin_red             Reduced state-space model containing only the
%                       rigid-body-motion states, lin_red.D is the control
%                       effectiveness matrix
% 
% *************************************************************************

% Simulink model used for trimming
model_name = 'sim_config2a_trim';


% Environmental parameters
envir = envirLoadParams('envir_params_default','envir',h);

% Initialize valid initial conditions
% Inital conditons of rigid-body motion
ic.q_bg = euler2Quat([0;0;-1.4]);
ic.V_Kb = [0.44*envir.atmosphere.a;0;0];
ic.s_Kg = [0; 0; 0];
ic.omega_Kb = [0; 0; 0];
% Initial conditions of actuators
ic.act.aileron = [0,0];
ic.act.elevator = 0;
ic.act.rudder = 0;
ic.act.prop = zeros(1,10);
ic.act.htp_trim = 0;


% To do: backup and restore base workspace variables
assignin('base','envir',envir);
assignin('base','aircraft',aircraft);
assignin('base','ic',ic);

% Compile model
feval(model_name, [], [], [], 'compile');

% Get input, output, and state names
input_names_blk = find_system (model_name, ...
    'FollowLinks', 'on', ...
    'searchdepth', 1, ...
    'Blocktype', 'Inport');
input_names = getArrayPortNames(input_names_blk);
num_inputs = length(input_names);

output_names_blk = find_system (model_name, ...
    'FollowLinks', 'on', ...
    'searchdepth', 1, ...
    'Blocktype', 'Outport');
output_names = getArrayPortNames(output_names_blk);
num_outputs = length(output_names);

[sizes, initial_conditions, state_names] = ...
                feval (model_name, [], [], [], "sizes");

num_cont_states = sizes(1);
num_disc_states = sizes(2);
num_states = num_cont_states + num_disc_states;


% Definition of the trim problem using Simulink block names
init_states = {'q_bg(1)'};
init_states_val = [1];

trim_states = {'q_bg(3)','V__Kb(1)','V__Kb(3)','elevator/A','elevator/dot_A',...
    'Props pitch/A(1:10)','Props pitch/dot_A(1:10)','Downwash'};
trim_states_val = [0,V,0,0,0,zeros(1,10),zeros(1,10),0];
trim_inputs = {'Cmd Elevator','Cmd Collective Thrust'};
trim_inputs_val = [-0.1,0.5];
known_derivs = {'rates(2)','V__Kb(1)','V__Kb(3)','elevator/A','elevator/dot_A',...
    'Props pitch/A(1:10)','Props pitch/dot_A(1:10)','Downwash'};
known_derivs_val = [0,0,0,0,0,zeros(1,10),zeros(1,10),0];
known_outputs = {'Airspeed','Climb Rate'};
known_outputs_val = [V,0];

% Get indices of blocks in state, input, derivatives, and output vector
i_x0 = getBlockIdx(state_names,init_states);
i_x = getBlockIdx(state_names,trim_states);
i_u = getBlockIdx(input_names,trim_inputs);
i_d = getBlockIdx(state_names,known_derivs);
i_y = getBlockIdx(output_names,known_outputs);

% Assign trim conditions to state, input, derivatives, and output vector
x = zeros(num_states,1);
x(i_x) = trim_states_val;
x(i_x0) = init_states_val;
u = zeros(num_inputs,1);
u(i_u) = trim_inputs_val;
d = zeros(num_cont_states,1);
d(i_d) = known_derivs_val;
y = zeros(num_outputs,1);
y(i_y) = known_outputs_val;

% Other jj_trim input parameters
del_x_max = repmat(1e42,size(x));
del_u_max = repmat(2,size(u));
del_x_lin = repmat(1e-6,size(x));
del_u_lin = repmat(1e-6,size(u));

options.EnableMessages = 0;
options.CompileFlag = 0;

% Maximum allowed values in state and input vector
x_min = repmat(-inf,size(x));
x_max = repmat(inf,size(x));
u_min = repmat(0,size(u));
u_min(1:4) = -1;
u_max = repmat(1,size(u));

% Run trim computation (jj_trim)
[x_tr, u_tr, d_tr, y_tr, errflg, infostruct] = ...
    jj_trim( model_name, x, u, d, y, i_x, i_u, i_d, i_y, ...
    state_names, input_names, state_names(1:num_cont_states), output_names, ...
    del_x_max, del_u_max, del_x_lin, del_u_lin, ...
    options, ...
    x_min, x_max, u_min, u_max);

if nargout >= 4
    i_x_lin = (1:num_cont_states)';
    n_panel = aircraft.wing_main.n_panel;
    lin_outputs = {'Omega_Kb_dt(1:3)','V_Kb_dt(1:3)'};
    i_y_lin = getBlockIdx(output_names,lin_outputs);
    i_y_lin2 = length(i_x_lin) + i_y_lin;
    i_u_lin = (1:length(u))';
    not_lin_inputs = {'Cmd Collective Thrust'};
    i_u_not_lin = getBlockIdx(input_names,not_lin_inputs);
    i_u_lin(i_u_not_lin) = [];
    i_u_lin2 = length(x_tr) + i_u_lin;
    jaco = jj_lin(model_name,[x_tr;u_tr],length(x_tr),[i_x_lin;i_u_lin2],...
        [i_x_lin;i_y_lin2]);
    lin.A = jaco(i_x_lin,i_x_lin);
    lin.B = jaco(i_x_lin,length(i_x_lin)+(1:length(i_u_lin)));
    lin.C = jaco(length(i_x_lin)+1:end,i_x_lin);
    lin.D = jaco(length(i_x_lin)+1:end,length(i_x_lin)+(1:length(i_u_lin)));
    lin.StateNames = state_names(i_x_lin);
    lin.InputNames = input_names(i_u_lin);
    lin.OutputNames = output_names(i_y_lin);
    if nargout == 5
        elim = false(size(lin.A,1),1);
        for i = 1:length(elim)
            if ~contains(lin.StateNames{i},'State equations')
                elim(i) = true;
            end
        end
        % [lin_red.A,lin_red.B,lin_red.C,lin_red.D] = modredEmb2( ...
        %     lin.A,lin.B,lin.C,lin.D,elim);
        sys = ss(lin.A,lin.B,lin.C,lin.D);
        sys_red = modred(sys,elim);
        lin_red.A = sys_red.A;
        lin_red.B = sys_red.B;
        lin_red.C = sys_red.C;
        lin_red.D = sys_red.D;
    end
end

% Uncompile model
feval(model_name, [], [], [], 'term');


% Output message
if iscell(infostruct.OutputMessage)
    output_message = [];
    for i = 1:size(infostruct.OutputMessage,1)    
        output_message = [ output_message, newline, infostruct.OutputMessage{i} ];
    end
else
    output_message = infostruct.OutputMessage;
end
if errflg
    error(output_message);
else
    disp(output_message);
end


% Assign trim point to ic struct
% Inital conditons of rigid-body motion
ic.q_bg = [1;0;x_tr(getBlockIdx(state_names,{'q_bg(3)'}));0];
ic.V_Kb = [ x_tr(getBlockIdx(state_names,{'V__Kb(1)'})); 0; ...
    x_tr(getBlockIdx(state_names,{'V__Kb(3)'})) ];
ic.s_Kg = [0; 0; 0];
ic.omega_Kb = [0; 0; 0];
% Initial conditions of actuators
ic.act.aileron = [0,0];
ic.act.elevator = x_tr(getBlockIdx(state_names,{'elevator/A'}));
ic.act.rudder = 0;
ic.act.prop = x_tr(getBlockIdx(state_names,{'Props pitch/A(1:10)'}));
ic.act.htp_trim = 0;

% Trim inputs
u0.cmd_aileron_left = 0;
u0.cmd_aileron_right = 0;
u0.cmd_elevator = u_tr(getBlockIdx(input_names,{'Cmd Elevator'}));
u0.cmd_rudder = 0;
u0.cmd_thrust = u_tr(getBlockIdx(input_names,{'Cmd Collective Thrust'}));

end


% Local function to copy port names according to their dimensions
function port_names = getArrayPortNames(port_names_blk)
port_names = {};
for i = 1:length(port_names_blk)
    dims = get_param(port_names_blk{i},'CompiledPortDimensions');
    if isempty(dims.Inport)
        port_length = prod(dims.Outport);
    else
        port_length = prod(dims.Inport(2:end));
    end
    for j = 1:port_length
        port_names{end+1} = [port_names_blk{i},num2str(j)];
    end
end
end

% Local function to get indices of state, input, or output names (search
% name) from the list of block (block_list)
function i_b = getBlockIdx(block_list,search_list)
i_b = [];
for i = 1:length(search_list)
    search_split = strsplit(search_list{i},'(');
    search_name = search_split{1};
    if length(search_split) > 1
        search_idx = eval(search_split{2}(1:end-1));
    else
        search_idx = 1;
    end
    for j = 1:length(search_idx)
        idx_found = 0;
        for k = 1:length(block_list)
            if contains(block_list{k},search_name)
                idx_found = idx_found + 1;
                if search_idx(j)==idx_found
                    i_b(end+1,1) = k;
                    break;
                end
            end
        end
    end
end
if any(i_b==0)
    error(['Could not find block(s) ',search_list{i_b==0}]);
end
end


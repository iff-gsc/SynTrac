function aircraft_state = aircraftSyntracGetState(aircraft)
%aircraftSyntracGetState picks states structs from aircraft struct

wing_count = 0;
fuse_count = 0;
% assign inputs to aircraft struct
field_names = fieldnames(aircraft);
num_fields = length(field_names);
for i = 1:num_fields
    field_name = field_names{i};
    % if field is a wing, only assign the wing.state
    if contains(field_name,'wing')
        wing_count = wing_count + 1;
        fieldname = ['wing_',num2str(wing_count),'_state'];
        fieldvar = aircraft.(field_name).state;
    elseif contains(field_name,'fuselage')
        if isfield(aircraft.fuselage,'state')
            fuse_count = fuse_count + 1;
            fieldname = ['fuselage_',num2str(fuse_count),'_state'];
            fieldvar = aircraft.(field_name).state;
        end
    elseif strcmp(field_name,'config')
        fieldname = 'config';
        fieldvar = aircraft.(field_name);
    else
        continue;
    end
    % assign to aircraft state struct
    aircraft_state.(fieldname) = fieldvar;
end

aircraft_state.body = rigidBodyCreate();

end
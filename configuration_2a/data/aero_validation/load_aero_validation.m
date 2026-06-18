%% Import data from text file
% Script for importing data from the following text file:
%
%    filename: Cruise_lowThrust_cases.dat
%
% Please find the data for the "inner" configuration ( the last propeller 
% are not at the wing tip but more inwards)  at cruise conditions at 
% "low thrust settings". The columns are separated by space. 
% Each row represents a different angle of attack. 
% The corresponding flow conditions are also included in the file. 
% The areas shown in the file are given as a percentage of the total area. 
% The total area for this configuration is 72.764 m².

%% Import Data Section
filename = "Cruise_lowThrust_cases.dat";

% Reference area: The total area for this configuration is 72.764 m²
RefArea_in_m2 = 72.764;

% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 25);

% Specify range and delimiter
opts.DataLines = [2, Inf];
opts.Delimiter = " ";

% Specify column names and types
opts.VariableNames = ["CaseName", "Density", "Mach", "Velocity", "AOA", "RPM", "Pitch", "CLift_fuselage", "CDrag_fuselage", "CMy_fuselage", "Area_fuselage", "CLift_htp", "CDrag_htp", "CMy_htp", "Area_htp", "CLift_vtp", "CDrag_vtp", "CMy_vtp", "Area_vtp", "CLift_wing", "CDrag_wing", "CMy_wing", "Area_wing", "PropThrust", "PropPower"];
opts.VariableTypes = ["categorical", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "string", "string"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";
opts.LeadingDelimitersRule = "ignore";

% Specify variable properties
opts = setvaropts(opts, ["PropThrust", "PropPower"], "WhitespaceRule", "preserve");
opts = setvaropts(opts, ["CaseName", "PropThrust", "PropPower"], "EmptyFieldRule", "auto");

% Import the data
Cruise_lowThrust_cases = readtable(filename, opts);

% Add reference area column
Cruise_lowThrust_cases.RefArea_in_m2 = repmat(RefArea_in_m2, height(Cruise_lowThrust_cases), 1);

%% Clear temporary variables
clear opts
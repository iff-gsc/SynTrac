function out = readTecplotDistribution( filename )
% readTecplotDistribution loads spanwise CFD forces and moments from
% Tecplot
% 
% Input:
%   filename            Tecplot filename (string), available:
%                           'distributions_AOA0.dat'
%                           'distributions_AOA0.7.dat'
%                           'distributions_AOA1.dat'
%                           'distributions_AOA3.dat'
% Output:
%   out                 Struct with spanwise force and moment distribution
%                       in N/m and Nm/m in a body-fixed coordinate system:
%                       x=backward, y=right, z=up
% 
% Example:
%   CFD = readTecplotDistribution( 'distributions_AOA0.dat' )
% 
% *************************************************************************

data = readTecplotPointFile(filename);

filename_split1 = strsplit(filename,'AOA');
filename_split2 = strsplit(filename_split1{2},'.dat');
filename_split3 = strsplit(filename_split2{1},'_clean');

AoA = str2double(filename_split3{1});

% Extract spanwise coordinate and force/moment derivatives
out.y      = data.values(:,1);   % spanwise position
out.dFx_dy = data.values(:,2);
out.dFy_dy = data.values(:,3);
out.dFz_dy = data.values(:,4);
out.dMx_dy = data.values(:,5);
out.dMy_dy = data.values(:,6);
out.dMz_dy = data.values(:,7);

% Coordinate transformation to compute lift and drag distribution
out.dL_dy = out.dFx_dy*-sind(AoA)+out.dFz_dy*cosd(AoA); % lift distribution
out.dD_dy = out.dFx_dy*cosd(AoA)+out.dFz_dy*sind(AoA); % drag distribution

end
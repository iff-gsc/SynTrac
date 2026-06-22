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

% Extract spanwise coordinate and force/moment derivatives
out.y      = data.values(:,1);   % spanwise position
out.dFx_dy = data.values(:,2);
out.dFy_dy = data.values(:,3);
out.dFz_dy = data.values(:,4);   % lift distribution
out.dMx_dy = data.values(:,5);
out.dMy_dy = data.values(:,6);
out.dMz_dy = data.values(:,7);

end
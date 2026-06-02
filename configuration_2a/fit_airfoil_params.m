addPathConfig2a();

% Assume HTP and VTP airfoil to be NACA0010
polar = airfoilLoadXfoilData('polar_NACA0010.txt');
airfoil_tail = airfoilAnalyticSimpleFit(polar,0,1)

% Main wing
polar = readPolar('polar_Re9.31_Ma0.39.txt');
airfoil_wing = airfoilAnalyticSimpleFit(polar,0,1)
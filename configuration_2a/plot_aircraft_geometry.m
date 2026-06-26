% Plot wing, htp and vtp in one figure

addPathConfig2a();

% Aircraft parameters
aircraft = aircraftSyntracCreate();

figure
wingPlotGeometry(aircraft.wing_main,3)
hold on
wingPlotGeometry(aircraft.wing_htp,3)
hold on
wingPlotGeometry(aircraft.wing_vtp,3)
grid on

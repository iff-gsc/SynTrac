% Plot wing, htp and vtp in one figure

addPathConfig2a();

wing_example_htp2a
close(1)
close(2)
wing_example_vtp2a
close (1)
close (2)
wing_example_2a
close(2)
hold on
wingPlotGeometry(htp,3)
hold on
wingPlotGeometry(vtp,3)
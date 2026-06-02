function polar = readPolar(filename)
    % Reads a txt file from second line
    data = readmatrix(filename, ...
                      'FileType', 'text', ...
                      'NumHeaderLines', 1);

    polar.alpha = data(:,1)';
    polar.c_L = data(:,2)';
    polar.c_D = data(:,3)';
    % cm: reference points seems to be nose, but we want c/4
    polar.c_m = data(:,4)' + 0.25*polar.c_L;

    tokens = regexp(filename, 'Ma([0-9]*\.?[0-9]+)', 'tokens');
    polar.info.Mach = str2double(tokens{1}{1}); 

end
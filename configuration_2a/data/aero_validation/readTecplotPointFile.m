function data = readTecplotPointFile(filename)
%READTECPLOTPOINTFILE Read a Tecplot ASCII file in POINT format.
%
%   DATA = READTECPLOTPOINTFILE(FILENAME) parses a Tecplot-style ASCII
%   data file (TITLE / VARIABLES / ZONE ... F=POINT) and returns the
%   variable names and numeric data as a struct.
%
%   This format is commonly produced by CFD post-processors (e.g. DLR
%   TAU) when exporting force/moment distributions or other point data
%   for use outside the solver's native environment.
%
%   Expected file structure:
%
%       TITLE = "..."
%       VARIABLES = "y", "<math>6</math>F<sub>x</sub>/<math>6</math>y", ...
%       #
%       # optional comment lines (ignored)
%       #
%       ZONE T="DISTR      wing", F=POINT
%       <numeric data, one row per point, one column per variable>
%       ...
%
%   Notes on parsing:
%     - The VARIABLES block may span multiple lines; this function keeps
%       reading lines until it encounters the ZONE keyword.
%     - Variable names are often wrapped in HTML/MathML-like tags (e.g.
%       "<math>6</math>F<sub>x</sub>/<math>6</math>y", representing a
%       partial derivative dFx/dy with formatting markup left over from
%       the export process). These tags are stripped automatically, so
%       "<math>6</math>F<sub>x</sub>/<math>6</math>y" becomes "6Fx/6y".
%     - Lines starting with '#' are treated as comments and skipped.
%     - Only the first ZONE in the file is read. If your file contains
%       multiple ZONE blocks (e.g. separate distributions for wing,
%       fuselage, tail), this function will need to be extended to loop
%       over zones -- it currently concatenates all data rows it finds
%       after the header into a single matrix.
%
%   INPUT:
%     filename - (string) path to the Tecplot ASCII file to read.
%
%   OUTPUT:
%     data - struct with fields:
%       .varNames  - cell array of cleaned variable name strings, in the
%                    order they appear in the file, e.g.
%                    {'y', '6Fx/6y', '6Fy/6y', '6Fz/6y', ...}
%       .zoneTitle - string, the title given in the ZONE T="..." entry
%                    (e.g. "DISTR      wing"), or '' if not found.
%       .values    - [nPoints x nVars] numeric matrix of the data, with
%                    columns in the same order as varNames.
%
%   EXAMPLE:
%       data = readTecplotPointFile('distributions_AOA0.dat');
%       y      = data.values(:,1);
%       dFz_dy = data.values(:,4);   % lift distribution (dFz/dy)
%       plot(y, dFz_dy);
%
%   See also: FOPEN, FGETL, SSCANF, REGEXP

    fid = fopen(filename, 'rt');
    if fid == -1
        error('readTecplotPointFile:fileNotFound', ...
            'Could not open file: %s', filename);
    end

    % Use a try/catch so the file handle always gets closed, even if
    % parsing fails partway through.
    try
        varNames   = {};
        zoneTitle  = '';
        headerDone = false;   % becomes true once we've passed the ZONE line
        dataLines  = {};      % raw text lines containing numeric data

        while ~feof(fid)
            line = strtrim(fgetl(fid));

            if isempty(line)
                continue   % skip blank lines
            end

            if startsWith(line, 'TITLE')
                % TITLE line carries metadata (grid/solution file names)
                % that we don't currently parse out, but is skipped
                % explicitly here so it isn't mistaken for data.
                continue

            elseif startsWith(line, 'VARIABLES')
                % The VARIABLES block lists every column name in double
                % quotes, and may be split across several lines. Keep
                % appending lines until we hit the ZONE keyword, which
                % marks the end of the header.
                varBlock = line;
                nextLine = strtrim(fgetl(fid));
                while ~startsWith(nextLine, 'ZONE') && ~feof(fid)
                    varBlock = [varBlock, ' ', nextLine]; %#ok<AGROW>
                    nextLine = strtrim(fgetl(fid));
                end

                varNames = parseVarNames(varBlock);

                % nextLine now holds the ZONE line itself; extract the
                % zone title from it (e.g. T="DISTR      wing").
                zoneLine = nextLine;
                tok = regexp(zoneLine, 'T="([^"]*)"', 'tokens', 'once');
                if ~isempty(tok)
                    zoneTitle = tok{1};
                end

                headerDone = true;
                continue

            elseif startsWith(line, '#')
                % Comment line, e.g. "# Force distributions of the
                % components". Ignored.
                continue

            elseif startsWith(line, 'ZONE')
                % Reached if VARIABLES and ZONE are not adjacent, or if
                % there were comment lines in between. Extract the zone
                % title the same way as above.
                tok = regexp(line, 'T="([^"]*)"', 'tokens', 'once');
                if ~isempty(tok)
                    zoneTitle = tok{1};
                end
                continue

            elseif headerDone
                % Once the header has been fully consumed, every
                % remaining non-empty, non-comment line is a row of
                % numeric data.
                dataLines{end+1} = line; %#ok<AGROW>
            end
        end

    catch ME
        fclose(fid);
        rethrow(ME);
    end

    fclose(fid);

    if isempty(varNames)
        error('readTecplotPointFile:noVariables', ...
            'No VARIABLES block found in file: %s', filename);
    end

    % Convert the collected text rows into a numeric matrix. sscanf with
    % '%f' extracts all floating point numbers from each line, which
    % naturally handles both space- and tab-separated values.
    numVars = numel(varNames);
    M = zeros(numel(dataLines), numVars);
    for i = 1:numel(dataLines)
        rowValues = sscanf(dataLines{i}, '%f')';
        if numel(rowValues) ~= numVars
            error('readTecplotPointFile:rowMismatch', ...
                ['Data row %d has %d values but %d variables were ', ...
                 'declared. Check file for malformed lines.'], ...
                i, numel(rowValues), numVars);
        end
        M(i,:) = rowValues;
    end

    data.varNames  = varNames;
    data.zoneTitle = zoneTitle;
    data.values    = M;
end


function names = parseVarNames(varBlock)
%PARSEVARNAMES Extract and clean variable names from a VARIABLES block.
%
%   NAMES = PARSEVARNAMES(VARBLOCK) takes the raw text of a Tecplot
%   VARIABLES line (or several lines concatenated together) and returns
%   a cell array of cleaned variable name strings, one per quoted entry.
%
%   Example:
%       parseVarNames('VARIABLES = "y", "<math>6</math>F<sub>x</sub>/<math>6</math>y"')
%       -> {'y', '6Fx/6y'}

    % Every variable name in the file is wrapped in double quotes;
    % extract the contents of each quoted substring.
    toks = regexp(varBlock, '"([^"]*)"', 'tokens');
    names = cellfun(@(c) cleanVarName(c{1}), toks, 'UniformOutput', false);
end


function clean = cleanVarName(raw)
%CLEANVARNAME Strip HTML/MathML-style markup from a raw variable name.
%
%   CLEAN = CLEANVARNAME(RAW) removes any "<tag>...</tag>" or
%   "<tag .../>"-style markup from RAW and trims whitespace.
%
%   This is needed because some Tecplot exports encode partial
%   derivative symbols and subscripts using leftover HTML/MathML tags,
%   e.g. RAW = "<math>6</math>F<sub>x</sub>/<math>6</math>y" represents
%   "dFx/dy" but literally contains "<math>", "</math>", "<sub>" and
%   "</sub>" tags around the meaningful characters. Stripping all
%   "<...>" tags leaves a clean (if slightly compressed) name such as
%   "6Fx/6y".
%
%   Example:
%       cleanVarName('<math>6</math>F<sub>x</sub>/<math>6</math>y')
%       -> '6Fx/6y'

    clean = regexprep(raw, '<[^>]+>', '');  % remove anything between < and >
    clean = strtrim(clean);
end
function writeTikz(filename,do_write,varargin)
% writeTikz export figure to Tikz file with some options
% 
% Inputs:
%   filename            Name of the Tikz file (e.g. 'tikz_file.tex')
%   do_write            Specify if the Tikz file should be written
%                       (logical)
%   options             Add further options (cell of strings) (e.g.
%                       {'scaled y ticks=base 10:-3'})
% 
% 
% In the Latex document, you have to add the following to the preamble:
% \usepackage{pgfplots}
% \newlength\figureheight
% \newlength\figurewidth
% \newlength\tikzlinewidth
% \newlength\tikzmarkersize
% \newcommand{\tikzfontsize}{\small}
% 
% In the document, you have to define the options, e.g.:
% \setlength{\figurewidth}{.43\textwidth}
% \setlength{\figureheight}{.33\textwidth}
% \setlength{\tikzlinewidth}{0.075em}
% \setlength{\tikzmarkersize}{0.3em}
% 
% After that, you can include the Tikz file with:
% \begin{figure}
%     \input{tikz_file_name}
% \end{figure}


if isempty(varargin)
    options = [];
else
    options = varargin{1};
end
if do_write
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = [{'ylabel style={font=\tikzfontsize}','ylabel near ticks',...
        'xlabel style={font=\tikzfontsize}','legend style={font=\tikzfontsize}'},...
        options];
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight, ...
        'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end
end
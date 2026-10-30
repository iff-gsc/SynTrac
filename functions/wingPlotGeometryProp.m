function [] = wingPlotGeometryProp(wing,props,varargin)
% wingPlotGeometryProp is a wrapper function for wingPlotGeometry that
% changes the color of strips behind propellers

n_props = length(props);
colors = repmat('r',n_props,1);
for i = 1:length(varargin)
    if strcmp(varargin{i},'PropColors')
        if size(varargin{i+1},1) == n_props
            colors = varargin{i+1};
        end
    end
end
wingPlotGeometry(wing,varargin{:});
for i = 1:length(props)
    h = gca;
    cv = h.Children;
    for j = 1:length(cv)
        if isa(cv(j),'matlab.graphics.primitive.Patch')
            y_patch = mean(cv(j).Vertices(:,2));
            for k = 1:n_props
                y_prop_l = props(k).pos_c(2) - props(k).diameter/2;
                y_prop_r = props(k).pos_c(2) + props(k).diameter/2;
                if y_patch > y_prop_l && y_patch < y_prop_r
                    cv(j).FaceColor = colors(k,:);
                end
            end
        end
    end
end
end

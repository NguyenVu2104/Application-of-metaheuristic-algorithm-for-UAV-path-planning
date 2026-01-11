clc; clear;

map_file  = 'map4_with_waypoints.mat';
path_file = 'ga_order_result.mat';

S = load(map_file);
R = load(path_file);

E_safe     = S.E_safe;
StartPoint = S.StartPoint;
Waypoints  = S.Waypoints;
path       = R.full_path;

[y_size, x_size] = size(E_safe);
[xg, yg] = meshgrid(1:x_size, 1:y_size);

figure; hold on;

E_plot = E_safe;
E_plot([1 end],:) = NaN;
E_plot(:,[1 end]) = NaN;

h = surf(xg, yg, E_plot);
set(h,'EdgeColor','none','FaceAlpha',0.25);
colormap(parula);
colorbar;
view(3); axis equal;

% Start
plot3(StartPoint.x, StartPoint.y, StartPoint.z, ...
      'go','MarkerSize',10,'MarkerFaceColor','g');

% Waypoints
for i = 1:numel(Waypoints)
    plot3(Waypoints(i).x, Waypoints(i).y, Waypoints(i).z, ...
          'r^','MarkerSize',9,'MarkerFaceColor','r');
end

% Path
plot3(path(:,2), path(:,1), path(:,3), ...
      'b-','LineWidth',2);

hold off;

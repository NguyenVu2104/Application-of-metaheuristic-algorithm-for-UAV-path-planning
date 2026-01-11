clc; clear;

map_file = 'map4_with_waypoints.mat';
K = [1, 1.25];

S = load(map_file);

E3d_safe   = S.E3d_safe;
StartPoint = S.StartPoint;
Waypoints  = S.Waypoints;

P0 = [StartPoint.y, StartPoint.x, StartPoint.z];

n_wp = numel(Waypoints);

Pts = zeros(n_wp+1, 3);
Pts(1,:) = P0;

for i = 1:n_wp
    Pts(i+1,:) = [Waypoints(i).y, Waypoints(i).x, Waypoints(i).z];
end

N = size(Pts,1);

D = inf(N,N);            % distance matrix
PATH = cell(N,N);        % path matrix

for i = 1:N
    for j = 1:N

        if i == j
            D(i,j) = 0;
            PATH{i,j} = [];
            continue;
        end

        Pi = Pts(i,:);
        Pj = Pts(j,:);

        [path, npts] = a_star_3D( ...
            K, ...
            E3d_safe, ...
            Pi(2), Pi(1), Pi(3), ...
            Pj(2), Pj(1), Pj(3), ...
            size(E3d_safe) );

        if isempty(path) || npts == 0
            warning('No path between %d and %d', i, j);
            D(i,j) = Inf;
            PATH{i,j} = [];
        else
            D(i,j) = path_length(path);
            PATH{i,j} = path;
        end
    end
end

save('astar_pairwise_data.mat', 'D', 'PATH', 'Pts');

function L = path_length(path)
    dif = diff(path,1,1);
    L = sum(sqrt(sum(double(dif).^2,2)));
end

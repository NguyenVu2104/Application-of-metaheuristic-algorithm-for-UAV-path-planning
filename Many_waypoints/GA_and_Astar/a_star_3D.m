function [path,n_points]=a_star_3D(K,E3d_safe,x0_old,y0_old,z0_old,xend_old,yend_old,zend_old,sizeE)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cost weights
kg = K(1);
kh = K(2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Algorithm

% Round start / end to integer grid indices (use floor/ceil as original)
x0 = floor(x0_old);
y0 = floor(y0_old);
z0 = floor(z0_old);
xend = ceil(xend_old);
yend = ceil(yend_old);
zend = ceil(zend_old);

% Size
y_size = sizeE(1);
x_size = sizeE(2);
z_size = sizeE(3);

% Guard: if start or end are blocked in E3d_safe, abort early
if E3d_safe(y0, x0, z0)
    warning('a_star_3D: start voxel is blocked in E3d_safe. Returning empty path.');
    path = [];
    n_points = 0;
    return;
end
if E3d_safe(yend, xend, zend)
    warning('a_star_3D: end voxel is blocked in E3d_safe. Returning empty path.');
    path = [];
    n_points = 0;
    return;
end

% Node from which the good neighbour is reached
came_fromx = zeros(sizeE);
came_fromy = zeros(sizeE);
came_fromz = zeros(sizeE);
came_fromx(y0, x0, z0) = x0;
came_fromy(y0, x0, z0) = y0;
came_fromz(y0, x0, z0) = z0;

open_list = sub2ind(sizeE, y0, x0, z0);    % linear indices of open nodes
is_open = false(sizeE); is_open(y0,x0,z0) = true;
is_closed = false(sizeE);

% G and F arrays (initialize to Inf)
G = Inf(sizeE);
F = Inf(sizeE);
G(y0, x0, z0) = 0;
F(y0, x0, z0) = sqrt((xend-x0)^2 + (yend-y0)^2 + (zend-z0)^2);

% Initialize
exit_path = 0;

% Main loop: while open not empty and path not found
while ~isempty(open_list) && exit_path==0

    % vectorized selection of best node in open_list (fast)
    [~, idx_min] = min( F(open_list) );           % vectorized, no loop
    current_lin = open_list(idx_min);
    [ycurrent, xcurrent, zcurrent] = ind2sub(sizeE, current_lin);

    % Check arrival
    if xcurrent==xend && ycurrent==yend && zcurrent==zend
        exit_path = 1;
        break;
    end

    % move current from open to closed (O(1) with masks and list swap)
    is_open(current_lin) = false;
    is_closed(current_lin) = true;
    % remove from open_list by swapping with last element for O(1) removal
    if idx_min < numel(open_list)
        open_list(idx_min) = open_list(end);
    end
    open_list(end) = [];
    % now open_list updated

    % Explore neighbours (26-neighborhood)
    for di = -1:1
        for dj = -1:1
            for dk = -1:1

                % skip self
                if di==0 && dj==0 && dk==0
                    continue;
                end

                xn = xcurrent + di;
                yn = ycurrent + dj;
                zn = zcurrent + dk;

                if xn < 1 || yn < 1 || zn < 1 || xn > x_size || yn > y_size || zn > z_size
                    continue;
                end

                neigh_lin = sub2ind(sizeE, yn, xn, zn);

                % skip if blocked
                if E3d_safe(yn, xn, zn)
                    continue;
                end

                % If neighbour already closed, skip
                if is_closed(neigh_lin)
                    continue;
                end

                % compute tentative g
                cost = norm([xn-xcurrent, yn-ycurrent, zn-zcurrent]);
                g_try = G(ycurrent, xcurrent, zcurrent) + cost;

                if ~is_open(neigh_lin)
                    % first time discovered: add to open
                    is_open(neigh_lin) = true;
                    open_list(end+1) = neigh_lin;
                    % set came_from now
                    came_fromy(yn, xn, zn) = ycurrent;
                    came_fromx(yn, xn, zn) = xcurrent;
                    came_fromz(yn, xn, zn) = zcurrent;
                    % set scores
                    G(yn, xn, zn) = g_try;
                    H = sqrt((xend - xn)^2 + (yend - yn)^2 + (zend - zn)^2);
                    F(yn, xn, zn) = kg * G(yn, xn, zn) + kh * H;
                else
                    % already in open: check for improvement
                    if g_try < G(yn, xn, zn)
                        came_fromy(yn, xn, zn) = ycurrent;
                        came_fromx(yn, xn, zn) = xcurrent;
                        came_fromz(yn, xn, zn) = zcurrent;
                        G(yn, xn, zn) = g_try;
                        H = sqrt((xend - xn)^2 + (yend - yn)^2 + (zend - zn)^2);
                        F(yn, xn, zn) = kg * G(yn, xn, zn) + kh * H;
                    end
                end

            end
        end
    end

end

% If no path found, return empty
if exit_path == 0
    path = [];
    n_points = 0;
    return;
end

% Reconstruct path backwards from current (which is the goal)
path_backwards = [ycurrent, xcurrent, zcurrent];
i = 2;
while xcurrent ~= x0 || ycurrent ~= y0 || zcurrent ~= z0
    py = came_fromy(ycurrent, xcurrent, zcurrent);
    px = came_fromx(ycurrent, xcurrent, zcurrent);
    pz = came_fromz(ycurrent, xcurrent, zcurrent);

    % Safety: if came_from is zero (no parent), abort and return empty with warning
    if py == 0 || px == 0 || pz == 0
        warning('a_star_3D: reconstruction failed, missing came_from for a node. Returning empty path.');
        path = [];
        n_points = 0;
        return;
    end

    path_backwards(i, :) = [py, px, pz];
    ycurrent = py;
    xcurrent = px;
    zcurrent = pz;
    i = i + 1;
end

% Number of waypoints
n_points = size(path_backwards, 1);

% Reverse path sequence to be from start to goal
path = path_backwards(n_points + 1 - (1:n_points), :);

% Reassign original non-integer start/end coordinates
path(1, :) = [y0_old, x0_old, z0_old];
path(n_points, :) = [yend_old, xend_old, zend_old];

end

% a_star_benchmark.m
% Load map1.mat, run A* n_runs times, export metrics CSV/XLSX
clear; clc;

% ------ User params ------
addpath("../map");
mapfile = 'map1.mat';
out_xlsx = 'a_star_map1.xlsx';
out_csv  = 'a_star_map1.csv';
n_runs = 10;                 % số lần chạy (điều chỉnh)
K = [1, 1];            % weight vector [kg, kh, ke] (ke sẽ đặt bên dưới)
use_map_generated = true;    % nếu bạn muốn dùng map1.mat (true)
%rng_seed = 123;      %random map1
rng_seed = 1234;      %random map1
% -------------------------

% load map
if ~isfile(mapfile)
    error('File %s not found. Chạy generate_map.m trước để tạo map1.mat', mapfile);
end
S = load(mapfile, 'E3d', 'E3d_safe', 'StartPoint', 'EndPoint', 'sizeE', 'd_grid');
E3d = S.E3d; E3d_safe = S.E3d_safe;
StartPoint = S.StartPoint; EndPoint = S.EndPoint; sizeE = S.sizeE;

% start / end in (y,x,z) and convert to x/y order used by a_star_3D signature if needed
% In A_star_sum: function [path,n_points]=a_star_3D(K,E3d_safe,x0_old,y0_old,z0_old,xend_old,yend_old,zend_old,sizeE)
x0 = StartPoint.x; y0 = StartPoint.y; z0 = StartPoint.z;
xend = EndPoint.x; yend = EndPoint.y; zend = EndPoint.z;

% Precompute obstacle voxels for clearance calc
[occ_y, occ_x, occ_z] = ind2sub(size(E3d), find(E3d == 1));
occ_vox = double([occ_y, occ_x, occ_z]);

% prepare results table
Results = table((1:n_runs)', nan(n_runs,1), nan(n_runs,1), nan(n_runs,1), ...
    'VariableNames', {'Run','PathLength','AvgClearance','Runtime_s'});

% run loop
for r = 1:n_runs
    fprintf('Run %d/%d ...\n', r, n_runs);
    t0 = tic;
    % call A* (uses signature from A_star_sum)
    [path, n_points] = a_star_3D(K, E3d_safe, x0, y0, z0, xend, yend, zend, sizeE);
    runtime = toc(t0);
    % path may be empty if no solution
    if isempty(path)
        pathlen = NaN;
        avg_clear = NaN;
    else
        % path is Nx3 in [y x z]
        % compute euclidean total length
        pts = path; % rows = waypoints
        dif = diff(pts(:,[1 2 3]), 1, 1); % difference in y,x,z order
        seglen = sqrt(sum(double(dif).^2,2));
        pathlen = sum(seglen);
        % compute average clearance: for each waypoint distance to nearest obstacle voxel
        if isempty(occ_vox)
            avg_clear = Inf;
        else
            Nway = size(pts,1);
            dmins = zeros(Nway,1);
            for i=1:Nway
                pt = double(pts(i,:)); % [y x z]
                difv = occ_vox - pt;               % Nocc x 3
                d2 = sum(difv.^2,2);
                dmins(i) = sqrt(min(d2));
            end
            avg_clear = mean(dmins);
        end
    end
    Results.PathLength(r) = pathlen;
    Results.AvgClearance(r) = avg_clear;
    Results.Runtime_s(r) = runtime;
end

% write outputs
writetable(Results, out_csv);
try
    writetable(Results, out_xlsx);
catch
    warning('Could not write XLSX; CSV saved.');
end

fprintf('Done. Results written to %s and %s\n', out_csv, out_xlsx);

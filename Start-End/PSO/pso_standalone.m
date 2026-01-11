clear; clc;

%% ------------- User parameters -------------
n_runs = 20;             % independent runs % swarm size
n_particles = 60;
n_iter = 150;            % iterations per run
w_inertia = 0.7;         % hệ số quán tính
c1 = 2;                  % hệ số nhận thức
c2 = 1;                  % hệ số xã hội
n_wp = 12;
vel_max_factor = 0.2;    % fraction of grid per axis for vmax
max_segment_sample = 5;  % samples per unit length for penalty evaluation
w_length = 1.0;          % weight for length
w_penalty = 10000.0;       % weight for penetration penalty
init_noise_scale = 0.08; % noise scale for biased init relative to grid diagonal
output_xlsx = 'pso_standalone_map1.xlsx';
map_file = 'map1.mat';
%% ------------------------------------------

% load map
m = load(map_file, 'E3d_safe', 'StartPoint', 'EndPoint', 'sizeE');
if ~isfield(m,'E3d_safe'), error('map file must contain E3d_safe'); end
E3d_safe = double(m.E3d_safe);
P0 = [m.StartPoint.y, m.StartPoint.x, m.StartPoint.z];
Pend = [m.EndPoint.y, m.EndPoint.x, m.EndPoint.z];
[y_size, x_size, z_size] = size(E3d_safe);

% compute distance transform for penetration depth (voxel units)
Ob = (E3d_safe == 1);
dist_to_free = bwdist(~Ob); % integer distances

% velocity limits per axis
vel_max = [vel_max_factor * y_size, vel_max_factor * x_size, vel_max_factor * z_size];

% precompute reference line for biased init
map_diag = sqrt(y_size^2 + x_size^2 + z_size^2);
refT = linspace(0,1,n_wp+2);
refPts = zeros(n_wp+2,3);
for i=1:n_wp+2
    refPts(i,:) = (1-refT(i)) * P0 + refT(i) * Pend;
end
refWay = refPts(2:end-1, :); % floating reference; will add noise and round

% prepare results container
Results = table((1:n_runs)', nan(n_runs,1), nan(n_runs,1), nan(n_runs,1), nan(n_runs,1), ...
    'VariableNames', {'Run','PathLength','Runtime_s','BestGen','TimeToBest_s'});

%% main loop runs
for run_i = 1:n_runs
    fprintf('PSO run %d/%d\n', run_i, n_runs);
    % fully biased initialization (no external initpop)
    swarm_pos = zeros(n_particles, n_wp*3);
    for p = 1:n_particles
        way = biased_waypoint_sample(refWay, init_noise_scale, map_diag, y_size, x_size, z_size);
        swarm_pos(p,:) = reshape(way',1,[]);
    end

    % velocities init small zeros
    swarm_vel = zeros(n_particles, n_wp*3);

    % evaluate initial positions using gradient-based penalty cost
    [pbest_val, pbest_info] = pso_eval_positions_penalty(swarm_pos, P0, Pend, Ob, dist_to_free, w_length, w_penalty, max_segment_sample);
    pbest_pos = swarm_pos;
    [gbest_val, gi] = min(pbest_val);
    gbest_pos = pbest_pos(gi, :);

    best_fit_history = zeros(n_iter,1);
    best_path_history = cell(n_iter,1);

    % tracking best generation/time
    best_so_far = inf;
    gen_of_best = NaN;
    time_to_best = NaN;
    run_tic = tic;

    tstart = tic;
    for it = 1:n_iter
        swarm_pos_eval = zeros(n_particles, n_wp*3);
        for p = 1:n_particles
            r1 = rand(1, n_wp*3);
            r2 = rand(1, n_wp*3);
            swarm_vel(p,:) = w_inertia * swarm_vel(p,:) ...
                + c1 .* r1 .* (pbest_pos(p,:) - swarm_pos(p,:)) ...
                + c2 .* r2 .* (gbest_pos - swarm_pos(p,:));

            % clamp velocity per axis (per waypoint)
            swarm_vel(p,:) = clamp_velocity_by_axis(swarm_vel(p,:), n_wp, vel_max);

            % update position (continuous)
            swarm_pos(p,:) = swarm_pos(p,:) + swarm_vel(p,:);

            % rounding/clamp to grid bounds for evaluation only; keep continuous positions internally
            pos_round = round(swarm_pos(p,:));
            % clamp each triple
            for k = 1:n_wp
                idx = (k-1)*3 + (1:3);
                pos_round(idx(1)) = min(max(1, pos_round(idx(1))), y_size);
                pos_round(idx(2)) = min(max(1, pos_round(idx(2))), x_size);
                pos_round(idx(3)) = min(max(1, pos_round(idx(3))), z_size);
            end
            % use rounded for local evaluation but keep continuous swarm_pos for velocity dynamics
            eval_pos = pos_round;
            % store eval positions temporarily for batch evaluation next
            swarm_pos_eval(p,:) = eval_pos; %#ok<SAGROW>
        end

        % batch evaluate rounded positions with penalty cost
        [vals, infos] = pso_eval_positions_penalty(swarm_pos_eval, P0, Pend, Ob, dist_to_free, w_length, w_penalty, max_segment_sample);

        % update pbest
        improved = vals < pbest_val;
        pbest_val(improved) = vals(improved);
        pbest_pos(improved, :) = swarm_pos_eval(improved, :);

        % update gbest
        [curr_gbest_val, idx] = min(pbest_val);
        if curr_gbest_val < gbest_val
            gbest_val = curr_gbest_val;
            gbest_pos = pbest_pos(idx, :);
        end

        best_fit_history(it) = gbest_val;
        best_path_history{it} = reshape([P0; reshape(round(gbest_pos),3,[])'; Pend], [], 3);

        % tracking time to best
        if gbest_val < best_so_far
            best_so_far = gbest_val;
            gen_of_best = it;
            time_to_best = toc(run_tic);
        end
    end
    runtime = toc(tstart);

    % select generation where best occurred and path
    [~, ig] = min(best_fit_history);
    best_path = best_path_history{ig};

    % ===== CHECK PATH VALIDITY (NO OBSTACLE COLLISION) =====
    is_valid_path = true;

    for s = 1:size(best_path,1)
        py = round(best_path(s,1));
        px = round(best_path(s,2));
        pz = round(best_path(s,3));

        % clamp for safety
        py = min(max(py,1), y_size);
        px = min(max(px,1), x_size);
        pz = min(max(pz,1), z_size);

        if E3d_safe(py,px,pz) == 1
            is_valid_path = false;
            break;
        end
    end

    if is_valid_path
        fprintf('Run %d: VALID path found (no obstacle collision)\n', run_i);
    else
        fprintf('Run %d: INVALID path (collides with obstacle)\n', run_i);
    end
    % ======================================================

    % final metrics
    if isempty(best_path)
        pathlen = Inf;
    else
        pathlen = path_length(best_path);
    end

    Results.PathLength(run_i) = pathlen;
    Results.Runtime_s(run_i) = runtime;
    Results.BestGen(run_i) = gen_of_best;
    Results.TimeToBest_s(run_i) = time_to_best;
end

euclidean_dist = norm(P0 - Pend);
fprintf('------------------------------------------------\n');
fprintf('DEBUG INFO:\n');
fprintf('Start Point (P0): [%.2f, %.2f, %.2f]\n', P0);
fprintf('End Point (Pend): [%.2f, %.2f, %.2f]\n', Pend);
fprintf('Euclidean Distance (Straight Line): %.4f\n', euclidean_dist);
fprintf('------------------------------------------------\n');

% select generation where best occurred and path
[~, ig] = min(best_fit_history);
best_path = best_path_history{ig};

% final metrics
pathlen = path_length(best_path);


% --- Euclid Check ---
if pathlen < euclidean_dist
    warning('Run %d: Path length (%.2f) < Euclidean (%.2f). Dữ liệu P0/Pend có thể bị lỗi!', run_i, pathlen, euclidean_dist);
end
% ----------------------

Results.PathLength(run_i) = pathlen;

% export results
try, writetable(Results, output_xlsx); end
fprintf('PSO finished. Results saved to %s\n', output_xlsx);


%% ---------------- Helper functions ----------------

function way = biased_waypoint_sample(refWay, noise_scale, map_diag, y_size, x_size, z_size)
n_wp_local = size(refWay,1);
noise = randn(n_wp_local,3) .* (noise_scale * map_diag);
way = refWay + noise;
way = round(way);
way(:,1) = min(max(1, way(:,1)), y_size);
way(:,2) = min(max(1, way(:,2)), x_size);
way(:,3) = min(max(1, way(:,3)), z_size);
end

function [vals, infos] = pso_eval_positions_penalty(positions, P0, Pend, Ob, dist_to_free, w_len, w_pen, max_sample)
% positions are rows of flattened (3*n_wp) integer positions (rounded)
M = size(positions,1);
vals = inf(M,1);
infos = cell(M,1);
for i=1:M
    pos_row = positions(i,:);
    way = reshape(pos_row,3,[])';
    coords = [P0; way; Pend];
    total_len = 0;
    total_pen = 0;
    for s = 1:size(coords,1)-1
        A = coords(s,:); B = coords(s+1,:);
        seg_len = norm(double(B-A));
        total_len = total_len + seg_len;
        n_samples = max(1, ceil(seg_len * max_sample));
        t = linspace(0,1,n_samples+1);
        for k = 1:numel(t)
            pt = (1-t(k))*A + t(k)*B;
            py = round(pt(1)); px = round(pt(2)); pz = round(pt(3));
            py = min(max(py,1), size(Ob,1));
            px = min(max(px,1), size(Ob,2));
            pz = min(max(pz,1), size(Ob,3));
            idx = sub2ind(size(Ob), py, px, pz);
            if Ob(idx) == 1
                d_pen = double(dist_to_free(idx)); % distance to nearest free voxel
                total_pen = total_pen + (d_pen)^2;
            end
        end
    end
    vals(i) = w_len * total_len + w_pen * total_pen;
    infos{i} = struct('coords', coords, 'length', total_len, 'penalty', total_pen);
end
end

function v = clamp_velocity_by_axis(v, n_wp, vel_max)
v_mat = reshape(v, 3, [])';
for j=1:size(v_mat,1)
    v_mat(j,1) = max(min(v_mat(j,1), vel_max(1)), -vel_max(1));
    v_mat(j,2) = max(min(v_mat(j,2), vel_max(2)), -vel_max(2));
    v_mat(j,3) = max(min(v_mat(j,3), vel_max(3)), -vel_max(3));
end
v = reshape(v_mat',1,[]);
end

function L = path_length(coords)
if isempty(coords), L = Inf; return; end
dif = diff(coords,1,1);
L = sum(sqrt(sum(double(dif).^2,2)));
end

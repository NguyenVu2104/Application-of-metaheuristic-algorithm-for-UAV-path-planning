clear; clc;

%% -------------------- USER PARAMETERS --------------------
map_file = 'map1.mat';        
pop_size = 20;                
n_gen = 200;                  
n_wp = 12;                    
p_crossover = 0.9;
p_mutation = 0.25;
tournament_k = 3;
max_segment_sample = 5;       
w_length = 1.0;               
w_penalty = 10000.0;            
init_noise_scale = 0.08;      
min_bound_margin = 1;                           
strict_mode = false;
output_results = 'ga_standalone_map1.xlsx';

n_runs = 20;                
%% ---------------------------------------------------------

% load map
S = load(map_file);
if ~isfield(S,'E3d_safe') || ~isfield(S,'StartPoint') || ~isfield(S,'EndPoint')
    error('Map file must contain E3d_safe, StartPoint, EndPoint.');
end
E3d_safe = double(S.E3d_safe);
P0 = [S.StartPoint.y, S.StartPoint.x, S.StartPoint.z];
Pend = [S.EndPoint.y, S.EndPoint.x, S.EndPoint.z];
grid_sz = size(E3d_safe);
y_size = grid_sz(1); x_size = grid_sz(2); z_size = grid_sz(3);

% Precompute distance transform for penalty
Ob = (E3d_safe == 1);
dist_to_free = bwdist(~Ob);

% Precompute map diagonal
map_diag = sqrt(y_size^2 + x_size^2 + z_size^2);

% Precompute reference line and reference waypoints
refT = linspace(0,1,n_wp+2);
refPts = zeros(n_wp+2,3);
for i=1:n_wp+2
    refPts(i,:) = round( (1-refT(i)) * P0 + refT(i) * Pend );
end
refWay = refPts(2:end-1, :); % n_wp x 3

% storage for results
Results = table((1:n_runs)', nan(n_runs,1), nan(n_runs,1), nan(n_runs,1), nan(n_runs,1), ...
    'VariableNames', {'Run','PathLength','Runtime_s','BestGen','TimeToBest_s'});


%% ================= RUN MANY TIMES =======================
for run_i = 1:n_runs

    fprintf('\n========== GA run %d/%d ==========\n', run_i, n_runs);

    % ----------------- Build initial pop (biased) -----------------
    pop = zeros(pop_size, n_wp*3);
    pop_valid = false(pop_size,1);
    pop_fitness = inf(pop_size,1);
    raw_paths = cell(pop_size,1);

    for i=1:pop_size
        noise = (randn(n_wp,3) .* (init_noise_scale * map_diag));
        way = refWay + noise;
        way = round(way);
        way(:,1) = min(max(1, way(:,1)), y_size);
        way(:,2) = min(max(1, way(:,2)), x_size);
        way(:,3) = min(max(1, way(:,3)), z_size);

        pop(i,:) = reshape(way',1,[]);
        coords = [P0; way; Pend];
        raw_paths{i} = coords;

        [cost, valid_flag] = cost_path(coords, Ob, dist_to_free, w_length, w_penalty, max_segment_sample);
        pop_fitness(i) = cost;
        pop_valid(i) = valid_flag;
    end

    fprintf('Init: %d/%d valid individuals.\n', sum(pop_valid), pop_size);

    % ----------------- Run GA evolution -----------------
    tstart = tic;
    [best_path, best_fit_history, gen_of_best, time_to_best] = ga_evolve( ...
        pop, pop_fitness, pop_size, n_gen, n_wp, P0, Pend, Ob, dist_to_free, ...
        p_crossover, p_mutation, tournament_k, w_length, w_penalty, max_segment_sample, ...
        y_size, x_size, z_size);
    runtime = toc(tstart);

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

writetable(Results, output_results);
fprintf('\nAll runs completed. Saved to %s\n', output_results);

%% ============================================================
%% =====================  SUBFUNCTIONS  ========================


function [cost, valid_flag] = cost_path(coords, Ob, dist_to_free, w_len, w_pen, max_sample)
    total_len = 0;
    total_pen = 0;
    valid_flag = true;
    nseg = size(coords,1)-1;

    for s=1:nseg
        A = coords(s,:);
        B = coords(s+1,:);
        seg_len = norm(double(B - A));
        total_len = total_len + seg_len;

        n_samples = max(1, ceil(seg_len * max_sample));
        t = linspace(0,1,n_samples+1);
        for k=1:numel(t)
            pt = (1-t(k))*A + t(k)*B;
            py = round(pt(1)); px = round(pt(2)); pz = round(pt(3));
            py = min(max(py,1), size(Ob,1));
            px = min(max(px,1), size(Ob,2));
            pz = min(max(pz,1), size(Ob,3));
            idx = sub2ind(size(Ob), py, px, pz);
            if Ob(idx)==1
                d_pen = double(dist_to_free(idx));
                total_pen = total_pen + (d_pen)^2;
                valid_flag = false;
            end
        end
    end

    cost = w_len * total_len + w_pen * total_pen;
end


function [best_path, best_fit_history, gen_of_best, time_to_best] = ga_evolve(pop, pop_fitness, ...
        pop_size, n_gen, n_wp, P0, Pend, Ob, dist_to_free, p_crossover, p_mutation, ...
        tournament_k, w_len, w_pen, max_sample, y_size, x_size, z_size)

    fitness_vals = pop_fitness(:);
    best_fit_history = zeros(n_gen,1);
    best_path_history = cell(n_gen,1);
    best_so_far = inf;
    gen_of_best = NaN;
    time_to_best = NaN;
    ga_t0 = tic;

    for g=1:n_gen
        new_pop = zeros(size(pop));

        % elitism
        [~, best_idx0] = min(fitness_vals);
        new_pop(1,:) = pop(best_idx0,:);
        keep = 2;

        while keep <= pop_size
            i1 = tournament_select(fitness_vals, tournament_k);
            i2 = tournament_select(fitness_vals, tournament_k);
            parent1 = pop(i1,:);
            parent2 = pop(i2,:);

            if rand < p_crossover
                [c1,c2] = ga_crossover(parent1, parent2);
            else
                c1 = parent1; c2 = parent2;
            end

            if rand < p_mutation
                c1 = ga_mutation(c1, y_size, x_size, z_size);
            end
            if rand < p_mutation && keep+1 <= pop_size
                c2 = ga_mutation(c2, y_size, x_size, z_size);
            end

            new_pop(keep,:) = c1; keep = keep+1;
            if keep <= pop_size
                new_pop(keep,:) = c2; keep = keep+1;
            end
        end

        pop = new_pop;
        fitness_vals = zeros(pop_size,1);
        pop_paths_info = cell(pop_size,1);

        for i=1:pop_size
            way = reshape(pop(i,:),3,[])';
            coords = [P0; way; Pend];
            [f, valid_flag] = cost_path(coords, Ob, dist_to_free, w_len, w_pen, max_sample);
            fitness_vals(i) = f;
            pop_paths_info{i}.coords = coords;
        end

        [best_fit, idx] = min(fitness_vals);
        best_fit_history(g) = best_fit;
        best_path_history{g} = pop_paths_info{idx}.coords;

        if best_fit < best_so_far
            best_so_far = best_fit;
            gen_of_best = g;
            time_to_best = toc(ga_t0);
        end
    end

    [~, ig] = min(best_fit_history);
    best_path = best_path_history{ig};
    if isempty(best_path)
        best_path = [];
    end
end


function idx = tournament_select(fitness_vals, k)
    n = numel(fitness_vals);
    s = randi(n,[k,1]);
    vals = fitness_vals(s);
    if all(isinf(vals))
        idx = s(randi(k));
    else
        [~, pos] = min(vals);
        idx = s(pos);
    end
end


function [c1,c2] = ga_crossover(p1,p2)
    L = numel(p1);
    pt = randi([3, L-3]);
    c1 = [p1(1:pt), p2(pt+1:end)];
    c2 = [p2(1:pt), p1(pt+1:end)];
end


function child = ga_mutation(parent, y_size, x_size, z_size)
    child = parent;
    n = numel(parent)/3;
    nmut = randi([1,2]);
    for k=1:nmut
        wi = randi(n);
        base = (wi-1)*3;
        child(base+1) = min(max(1, child(base+1) + randi([-6 6])), y_size);
        child(base+2) = min(max(1, child(base+2) + randi([-6 6])), x_size);
        child(base+3) = min(max(1, child(base+3) + randi([-3 3])), z_size);
    end
end


function L = path_length(coords)
    if isempty(coords)
        L = Inf;
        return;
    end
    dif = diff(coords,1,1);
    L = sum(sqrt(sum(double(dif).^2,2)));
end

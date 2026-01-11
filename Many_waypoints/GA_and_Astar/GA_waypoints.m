clc; clear;

%% ===== USER INPUT =====
map_file = 'map4_with_waypoints.mat';
pop_size = 60;
n_gen = 400;
p_crossover = 0.9;
p_mutation = 0.4;
tournament_k = 3;


%% ----- LOAD MAP & WAYPOINTS -----
load('astar_pairwise_data.mat', 'D', 'PATH', 'Pts');
S = load(map_file);

E3d_safe   = S.E3d_safe;
E_safe     = S.E_safe;
StartPoint = S.StartPoint;
Waypoints  = S.Waypoints;

K = [1, 1.25];    
map.E3d_safe = E3d_safe;
map.sizeE    = size(E3d_safe);

P0 = [StartPoint.y, StartPoint.x, StartPoint.z];

n_wp = numel(Waypoints);
Wp = zeros(n_wp,3);
for i = 1:n_wp
    Wp(i,:) = [Waypoints(i).y, Waypoints(i).x, Waypoints(i).z];
end

map.E3d_safe = E3d_safe;
map.sizeE = size(E3d_safe);

%% ----- GA INITIALIZATION -----
pop = zeros(pop_size, n_wp);
for i = 1:pop_size
    pop(i,:) = randperm(n_wp);
end

fitness = inf(pop_size,1);

for i = 1:pop_size
    fitness(i) = fitness_order(pop(i,:), D);
end

%% ----- GA LOOP -----
for gen = 1:n_gen
    new_pop = zeros(size(pop));

    % elitism
    [~, best_idx] = min(fitness);
    new_pop(1,:) = pop(best_idx,:);
    ptr = 2;

    while ptr <= pop_size
        i1 = tournament_select(fitness, tournament_k);
        i2 = tournament_select(fitness, tournament_k);

        p1 = pop(i1,:);
        p2 = pop(i2,:);

        if rand < p_crossover
            c1 = ox_crossover(p1,p2);
            c2 = ox_crossover(p2,p1);
        else
            c1 = p1; c2 = p2;
        end

        if rand < p_mutation, c1 = swap_mutation(c1); end
        if rand < p_mutation, c2 = swap_mutation(c2); end

        new_pop(ptr,:) = c1; ptr = ptr + 1;
        if ptr <= pop_size
            new_pop(ptr,:) = c2; ptr = ptr + 1;
        end
    end

    pop = new_pop;

    for i = 1:pop_size
        fitness(i) = fitness_order(pop(i,:), D);
    end

    fprintf('Gen %3d | Best cost = %.2f\n', gen, min(fitness));
end

%% ----- BEST ORDER & FULL PATH -----
[~, best_idx] = min(fitness);
best_order = pop(best_idx,:);

full_path = [];
prev = 1;

for k = 1:numel(best_order)
    curr = best_order(k) + 1;
    seg = PATH{prev, curr};

    if isempty(seg)
        error('Missing path segment %d -> %d', prev, curr);
    end

    if isempty(full_path)
        full_path = seg;
    else
        full_path = [full_path; seg(2:end,:)]; %#ok<AGROW>
    end

    prev = curr;
end

save('ga_order_result.mat', 'best_order', 'full_path');

%% ===== FUNCTION =====

function idx = tournament_select(fitness, k)
    cand = randi(numel(fitness), [k 1]);
    [~, p] = min(fitness(cand));
    idx = cand(p);
end

function child = ox_crossover(p1, p2)
    n = numel(p1);
    cut = sort(randperm(n,2));

    child = zeros(1,n);
    child(cut(1):cut(2)) = p1(cut(1):cut(2));

    remain = p2(~ismember(p2, child));
    child(child==0) = remain;
end

function child = swap_mutation(parent)
    n = numel(parent);
    idx = randperm(n,2);
    child = parent;
    child(idx) = child(fliplr(idx));
end

function cost = fitness_order(order, D)

    cost = 0;
    prev = 1;

    for i = 1:numel(order)
        curr = order(i) + 1;
        d = D(prev, curr);

        if isinf(d)
            cost = Inf;
            return;
        end

        cost = cost + d;
        prev = curr;
    end
end
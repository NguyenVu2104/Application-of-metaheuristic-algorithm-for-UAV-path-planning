% --- File: gen_map_with_waypoints.m ---
% Script chính: cấu hình input và gọi hàm sinh waypoints từ file map .mat
clc; clear;

% -------------------------
% Tham số người dùng (chỉnh ở đây)
map_file      = 'map4_no_waypoint.mat';   % file map đã sinh trước đó
N_waypoints   = 18;                  % số điểm cần sinh
seed          = 1;                   % seed để tái lập kết quả, [] để ngẫu nhiên
min_sep       = 2;                   % khoảng cách ô tối thiểu giữa các waypoint
avoidSE       = true;                % tránh vùng gần StartPoint
clearanceZ    = [];                  % [zmin zmax] để giới hạn z, [] để dùng toàn bộ z
maxAttempts   = 1e4;                 % số lần thử tối đa cho mỗi waypoint
out_file      = 'map4_with_waypoints.mat'; % tên file xuất

% Gọi hàm chính
out_file = generate_waypoints_from_map(map_file, N_waypoints, ...
    'Seed', seed, ...
    'MinSeparation', min_sep, ...
    'AvoidStartEnd', avoidSE, ...
    'ClearanceZ', clearanceZ, ...
    'MaxAttempts', maxAttempts, ...
    'OutFile', out_file);

% -------------------------
% Hàm cục bộ: đặt dưới script để thuận tiện cho người dùng
function out_file = generate_waypoints_from_map(map_file, N, varargin)
% GENERATE_WAYPOINTS_FROM_MAP Sinh N waypoints an toàn từ file map .mat
%
% out_file = generate_waypoints_from_map(map_file, N, 'Name', Value, ...)
%
% Tham số Name-Value:
%  'Seed'          : seed cho rng. Mặc định: [] (ngẫu nhiên)
%  'MinSeparation' : khoảng cách tối thiểu giữa hai waypoint. Mặc định: 0
%  'AvoidStartEnd' : true/false, tránh vùng Start/End nếu có. Mặc định: true
%  'ClearanceZ'    : [zmin zmax], mặc định toàn bộ z của map
%  'MaxAttempts'   : số lần thử tối đa cho mỗi waypoint. Mặc định: 1e4
%  'OutFile'       : tên file .mat kết quả. Mặc định: 'map_with_waypoints.mat'

% ---- parse input
p = inputParser;
p.addRequired('map_file', @(s) ischar(s) || isstring(s));
p.addRequired('N', @(x) isnumeric(x) && isscalar(x) && (x>0) && (floor(x)==x));
p.addParameter('Seed', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x)));
p.addParameter('MinSeparation', 0, @(x) isnumeric(x) && isscalar(x) && x>=0);
p.addParameter('AvoidStartEnd', true, @(x) islogical(x) || ismember(x,[0 1]));
p.addParameter('ClearanceZ', [], @(x) isempty(x) || (isnumeric(x) && numel(x)==2));
p.addParameter('MaxAttempts', 1e4, @(x) isnumeric(x) && isscalar(x) && x>0);
p.addParameter('OutFile', 'map_with_waypoints.mat', @(s) ischar(s) || isstring(s));
p.parse(map_file, N, varargin{:});

seed = p.Results.Seed;
min_sep = double(p.Results.MinSeparation);
avoidSE = logical(p.Results.AvoidStartEnd);
clearanceZ = p.Results.ClearanceZ;
maxAttempts = p.Results.MaxAttempts;
out_file = char(p.Results.OutFile);

% ---- load map
if ~isfile(map_file)
    error('File map không tồn tại: %s', map_file);
end
S = load(map_file);

% Kiểm tra biến E3d_safe hoặc E3d
if isfield(S, 'E3d_safe')
    E3d_safe = S.E3d_safe;
elseif isfield(S, 'E3d')
    % nếu chỉ có E3d, coi ô có E3d==0 là free
    E3d_safe = double(~S.E3d);
else
    error('File map phải chứa biến E3d_safe hoặc E3d.');
end

% Lấy kích thước map
if isfield(S, 'sizeE')
    sizeE = S.sizeE;
else
    sz = size(E3d_safe);
    if numel(sz) == 3
        sizeE = [sz(1), sz(2), sz(3)];
    else
        error('Không xác định được kích thước map 3D.');
    end
end

% ---- seed ngẫu nhiên
if ~isempty(seed)
    rng(seed);
else
    rng('shuffle');
    seed = rng;
end

% ---- z range
if isempty(clearanceZ)
    zmin = 1;
    zmax = sizeE(3);
else
    zmin = max(1, round(clearanceZ(1)));
    zmax = min(sizeE(3), round(clearanceZ(2)));
    if zmin > zmax
        error('ClearanceZ không hợp lệ: zmin > zmax.');
    end
end

% ---- tạo mask ô free theo E3d_safe == 0 và giới hạn z
free_mask = (E3d_safe == 0);
if zmin > 1
    free_mask(:,:,1:(zmin-1)) = false;
end
if zmax < sizeE(3)
    free_mask(:,:,(zmax+1):end) = false;
end

% ---- tránh Start/End nếu yêu cầu
if avoidSE
    se_coords = [];
    if isfield(S, 'StartPoint')
        sp = S.StartPoint;
        % hỗ trợ cả trường hợp struct('x',..,'y',..,'z',..) và struct('y',..,'x',..,'z',..)
        if isfield(sp,'x') && isfield(sp,'y') && isfield(sp,'z')
            se_coords = [se_coords; double([sp.y, sp.x, sp.z])];
        elseif isfield(sp,'y') && isfield(sp,'x') && isfield(sp,'z')
            se_coords = [se_coords; double([sp.y, sp.x, sp.z])];
        end
    end
    if isfield(S, 'EndPoint')
        ep = S.EndPoint;
        if isfield(ep,'x') && isfield(ep,'y') && isfield(ep,'z')
            se_coords = [se_coords; double([ep.y, ep.x, ep.z])];
        elseif isfield(ep,'y') && isfield(ep,'x') && isfield(ep,'z')
            se_coords = [se_coords; double([ep.y, ep.x, ep.z])];
        end
    end
    if ~isempty(se_coords)
        if isfield(S, 'n_low')
            n_low = S.n_low;
        else
            n_low = 2;
        end
        for k=1:size(se_coords,1)
            y0 = se_coords(k,1);
            x0 = se_coords(k,2);
            z0 = se_coords(k,3);
            yr = max(1, y0-n_low) : min(sizeE(1), y0+n_low);
            xr = max(1, x0-n_low) : min(sizeE(2), x0+n_low);
            zr = max(1, z0-n_low) : min(sizeE(3), z0+n_low);
            free_mask(yr, xr, zr) = false;
        end
    end
end

% ---- danh sách ô ứng viên
free_inds = find(free_mask);
if isempty(free_inds)
    error('Không tìm thấy ô free thỏa yêu cầu trong bản đồ với z trong [%d %d].', zmin, zmax);
end
[y_list, x_list, z_list] = ind2sub(size(free_mask), free_inds);
coords = [y_list(:), x_list(:), z_list(:)]; % hàng dạng [y x z]
nCandidates = size(coords,1);

% ---- chọn ngẫu nhiên N điểm với ràng buộc MinSeparation
Waypoints = struct('x', {}, 'y', {}, 'z', {});
selected = zeros(0,3);
attempts_total = 0;

for iwp = 1:N
    found = false;
    attempts = 0;
    while ~found && attempts < maxAttempts
        attempts = attempts + 1;
        attempts_total = attempts_total + 1;
        idx = randi(nCandidates);
        cand = coords(idx, :); % [y x z]
        if ~isempty(selected)
            dists = sqrt(sum((selected - cand).^2, 2));
            if any(dists < min_sep)
                continue;
            end
        end
        selected(end+1, :) = cand; %#ok<AGROW>
        found = true;
    end
    if ~found
        error('Không thể chọn đủ %d waypoint sau %d lần thử. Tăng MaxAttempts hoặc giảm MinSeparation.', N, attempts_total);
    end
end

% ---- tạo struct Waypoints với field x,y,z cho tương thích
for k=1:size(selected,1)
    Waypoints(k).y = selected(k,1);
    Waypoints(k).x = selected(k,2);
    Waypoints(k).z = selected(k,3);
end

% ---- chuẩn bị lưu: giữ nguyên các biến map ban đầu và thêm Waypoints, meta
map_vars = fieldnames(S);
for k=1:numel(map_vars)
    temp.(map_vars{k}) = S.(map_vars{k});
end

meta = struct();
meta.generate_time = datetime('now');
meta.generate_seed = seed;
meta.N = N;
meta.MinSeparation = min_sep;
meta.ClearanceZ = [zmin zmax];
meta.MaxAttempts = maxAttempts;
meta.AvoidStartEnd = avoidSE;

% Lưu file kết quả
save(out_file, '-struct', 'temp');
save(out_file, 'Waypoints', 'meta', '-append');

fprintf('Đã tạo %d waypoints và lưu vào file: %s\n', N, out_file);

end

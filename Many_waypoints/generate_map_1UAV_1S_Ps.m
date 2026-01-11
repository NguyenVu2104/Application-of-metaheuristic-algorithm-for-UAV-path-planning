%Một điểm bắt đầu, nhiều điểm cần đi qua
% --- File: generate_map.m ---
clc; clear;

% 1. Cấu hình tham số bản đồ
sizeE = [80 80 30]; % Kích thước [y, x, z]
d_grid = 1;         % Độ phân giải của lưới (khoảng cách giữa các điểm lưới) 
n_low = 3;          % Bán kính vùng an toàn quanh điểm đầu/cuối

% điểm đầu/cuối
StartPoint = struct('y', 7, 'x', 5, 'z', 4);

% Chuyển đổi sang dạng vector cho hàm
P0 = [StartPoint.y, StartPoint.x, StartPoint.z];

% Chiều cao bay trung bình (tham số cho việc sinh địa hình)
h = round(0.5 * sizeE(3));

% 2. Gọi hàm tạo bản đồ
disp('Đang tạo bản đồ 3D...');
[E, E_safe, E3d, E3d_safe] = grid_3D_safe_zone(sizeE, d_grid, h, P0, n_low);

% 3. Lưu dữ liệu ra file .mat
filename = 'map4_no_waypoint.mat'; % Đặt tên file
save(filename, 'E', 'E_safe', 'E3d', 'E3d_safe', 'sizeE', 'd_grid', 'StartPoint', 'n_low');

disp(['Đã lưu bản đồ vào file: ', filename]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [E,E_safe,E3d,E3d_safe]=grid_3D_safe_zone(sizeE,d_grid,h,P0,n_low)

%kích thước lưới
y_size=sizeE(1);
x_size=sizeE(2);
z_size=sizeE(3);

%dọc
z_grid=1:d_grid:z_size;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Tạo vật cản ngẫu nhiên

%Phân rõ:
%occ (occupancy mask): 0/1, chỉ ra có vật cản hay không
%E (height map): số bất kỳ, chỉ ra độ cao vật cản tại 1 ô, 0 = ko có vật cản

%Tạo điểm ngẫu nhiên độ cao 0 - 1
mean_E=0; %kỳ vọng của phân bố Gaussian: nhiễu trung bình bằng 0, đối xứng quanh giá trị 0
%chọn mean=0 để có thể điều khiển mật độ chướng ngại vật thông qua sigma và k_sigma

sigma=1; %độ lệch chuẩn của phân bố Gaussian
%Quy định mức dao động ngẫu nhiên của giá trị trong ma trận E

k_sigma=2.4; %điều chỉnh mức ngưỡng phân biệt giữa ô trống và ô chướng ngại vật
%Mật độ vật cản, giảm k_sigma tăng số vật cản
%chú ý phân phối chuẩn có dạng: 95% giá trị nằm dưới ±2, chỉ 2.3% giá trị vượt quá 2
%và 1% giá trị vượt quá 2.2

%tmp = random('Normal',mean_E,sigma,y_size,x_size); %tương đương với:
tmp = normrnd(mean_E, sigma, y_size, x_size);

occ = tmp > (k_sigma * sigma);

%tạo ma trận E chứa độ cao
E = zeros(y_size, x_size, 'double');

%độ cao thấp nhất
hh_min=3;

%Tạo ma trận tạm thời để xử lý
EE=E;

%Tạo độ cao ngẫu nhiên cho vật cản trong ma trận E
for i=1:x_size
    for j=1:y_size
        if occ(j,i)==1
            %tạo block ngẫu nhiên bằng phân phối beta và giới hạn block trong bản đồ
            expand_x = 1 + round(betarnd(0.5, 0.5)); %mở rộng 1 hoặc 2
            expand_y = 1 + round(betarnd(0.5, 0.5));
            k = max(1, i - expand_x) : min(x_size, i + expand_x);
            l = max(1, j - expand_y) : min(y_size, j + expand_y);
            
            % sinh chiều cao ngẫu nhiên cho block
            hh=round(random('Normal',0.75*h,0.5*h));
            if hh<hh_min
                hh=hh_min;
            elseif hh>z_size
                hh=z_size;
            end
            E(l,k)=max(E(l,k),hh); % Dùng max để không bị hạ thấp độ cao nếu block đè lên nhau
        end
    end
end
%cập nhật lại occupancy chính xác từ E (trường hợp block có độ cao)
occ = (E > 0);

%Dọn dẹp quanh vùng Start
x_range_0 = max(1, P0(2)-n_low) : min(x_size, P0(2)+n_low);
y_range_0 = max(1, P0(1)-n_low) : min(y_size, P0(1)+n_low);
E(y_range_0, x_range_0) = 0;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Tạo lưới 3D

z_grid_reshaped = reshape(z_grid, 1, 1, []); % Chuyển vector thành 1x1xZ
E3d = (E >= z_grid_reshaped); %ghi chiều cao vật cản vào lưới 3D
%E3d(y,x,z) = 1  nếu  độ cao E(y,x) >= z 
%E3d(y,x,z) = 0  nếu  độ cao E(y,x) < z

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Tạo Safe Zone

%2D Safe Zone
E_safe = E;

E_dilated_max = imdilate(E, ones(3,3));    % giá trị độ cao lớn nhất trong vùng lân cận

E_safe = max(E, min(E_dilated_max + 1, z_size));  % kẹp không vượt quá z_size

% đảm bảo giữ vùng start là bằng không
E_safe(y_range_0, x_range_0) = 0;


%3D Safe Zone
% tái tạo grid z như trước
% chuẩn bị lại kích thước đúng
z_grid_reshaped   = reshape(1:z_size, 1, 1, []);   % 1 x 1 x z_size
E_safe_reshaped   = reshape(E_safe, y_size, x_size, 1); % y_size x x_size x 1

% mặt nạ các voxel có z <= E_safe(y,x)
% kích thước kết quả: y_size x x_size x z_size
mask_safe_voxels = (z_grid_reshaped <= E_safe_reshaped);

% loại trừ voxel vật cản thật (giữ vật cản = 1)
mask_safe_voxels = mask_safe_voxels & (~E3d);

% khởi tạo E3d_safe nếu chưa có
E3d_safe = E3d;

% gán voxel an toàn thành 1
E3d_safe(mask_safe_voxels) = 1;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%tạo biên cho map
% A(i, j)      → chọn phần tử tại dòng i, cột j
% A(:, j)      → chọn toàn bộ dòng, tại cột j
% A(i, :)      → chọn toàn bộ cột, tại dòng i
% A(:,:,k)     → chọn toàn bộ mặt phẳng thứ k trong ma trận 3D

% biên 2D
E([1,end], :) = z_size;
E(:, [1,end]) = z_size;
E_safe([1,end], :) = z_size;
E_safe(:, [1,end]) = z_size;

% biên 3D
E3d([1,end], :, :)     = 1;
E3d(:, [1,end], :)     = 1;
E3d(:, :, [1,end])     = 1;
E3d_safe([1,end], :, :)= 1;
E3d_safe(:, [1,end], :)= 1;
E3d_safe(:, :, [1,end])= 1;

end

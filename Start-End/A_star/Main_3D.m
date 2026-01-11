%Main

clc
clear

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Đọc map đã tạo từ generate_map.m
addpath("../map");
load('map1.mat','E','E_safe','E3d','E3d_safe','StartPoint','EndPoint','sizeE','d_grid','n_low');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Grid and path parameters

% Lấy điểm bắt đầu và kết thúc từ file map1.mat
x0 = StartPoint.x;
y0 = StartPoint.y;
z0 = StartPoint.z;

xend = EndPoint.x;
yend = EndPoint.y;
zend = EndPoint.z;

% A* cost weights
kg = 1;
kh = 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Path generation

%Store gains in vector
K = [kg kh];

%Measure path computation time
tic

%Generate path
[path,n_points] = a_star_3D(K, E3d_safe, x0, y0, z0, xend, yend, zend, sizeE);

T_path = toc;


%Path waypoints partial distance

%Initialize
path_distance=zeros(n_points,1);

for i=2:n_points 
	path_distance(i)=path_distance(i-1)+sqrt((path(i,2)-path(i-1,2))^2+(path(i,1)-path(i-1,1))^2+(path(i,3)-path(i-1,3))^2);      
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %Plot
 
 %Map grid
 x_grid=1:d_grid:sizeE(2);
 y_grid=1:d_grid:sizeE(1);
 
 %Path on safe map
 figure(1)
 surf(x_grid(2:end-1),y_grid(2:end-1),E_safe(2:end-1,2:end-1))
 hold on
 plot3(x0,y0,z0,'gs')
 plot3(xend,yend,zend,'rd')
 plot3(path(:,2),path(:,1),path(:,3),'yx')
 plot3(path(:,2),path(:,1),path(:,3),'w')
 axis tight
 axis equal
 view(0,90);
 colorbar
 
 % %Path on standard map
 % figure(2)
 % surf(x_grid(2:end-1),y_grid(2:end-1),E(2:end-1,2:end-1))
 % hold on
 % plot3(x0,y0,z0,'gs')
 % plot3(xend,yend,zend,'rd')
 % plot3(path(:,2),path(:,1),path(:,3),'yx')
 % plot3(path(:,2),path(:,1),path(:,3),'w')
 % axis tight
 % axis equal
 % view(0,90);
 % colorbar
 % 
 % %Path height profile
 % figure(3)
 % plot(path_distance,path(:,3))
 % grid on
 % xlabel('Khoảng cách')
 % ylabel('Chiều cao')

fprintf('\n===== KẾT QUẢ A* =====\n');
fprintf('Thời gian chạy thuật toán: %.6f giây\n', T_path);
fprintf('Tổng chiều dài đường đi: %.4f \n', path_distance);
fprintf('Số waypoint: %d\n', n_points);

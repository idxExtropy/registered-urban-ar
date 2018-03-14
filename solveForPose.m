warning('off', 'all'); clear variables; warning('on', 'all');
load 'globals.mat';

if im_isEdgeFound < 1
    return % No candidate edges found
end;

%% Compute the corrected Yaws
pc_Orient_Adjustment(1) = 0; % Roll
pc_Orient_Adjustment(2) = 0; % Pitch
pc_Orient_Adjustment(3) = geo_bldgYaw - pc_bldgYaw; % Yaw

disp(['DETAILS: Image Edge (Yaw): ' num2str(im_EdgeYaw) ' degrees']);
disp(['DETAILS: Yaw correction: ' num2str(pc_Orient_Adjustment(3)) ' degrees']);

sensor_Yaw = pc_kfOrient(3) + pc_Orient_Adjustment(3);

%% Compute the corrected position
% Compute ingtersection of line and plane in point cloud space
%pc_rel_edge_angle = pc_bldgYaw + im_EdgeYaw; % Works [32]
pc_rel_edge_angle = pc_kfOrient(3) + im_EdgeYaw;

pc_p2 = [0 0 0]; % Reminder: Y-Axis is flipped
pc_p2(1) = pc_kfPos(1) + 10;
pc_p2(2) = pc_kfPos(2) + (-10 * tan(pc_rel_edge_angle * pi / 180));

[pc_intersection, pc_check] = plane_line_intersect(pc_n, pc_p, pc_kfPos, pc_p2);
pc_intersection(3) = pc_kfPos(3); % any z offset is arbitrary
if pc_check == 0 || pc_check == 2 
    disp('WARNING: Point Cloud point and building do not intersect'); 
end;

openfig('pc_fig.fig'); hold on; 
h = scatter3(pc_intersection(1), pc_intersection(2), pc_intersection(3), 'gd', 'filled'); hold on;
set(h, 'SizeData', 200); clear h;
%set(gca,'zdir','reverse'); 
axis equal;

%% Rotate the point cloud and show the new orientation
A = [cos(-pc_Orient_Adjustment(3) * pi / 180) sin(-pc_Orient_Adjustment(3) * pi / 180) 0 0; ...
     -sin(-pc_Orient_Adjustment(3) * pi / 180) cos(-pc_Orient_Adjustment(3) * pi / 180) 0 0; ...
     0 0 1 0; ...
     0 0 0 1];
pc_data = [pc_data; pc_intersection];
C = ones(length(pc_data), 3) .* [0 0 255];
tform = affine3d(A); pc_ptCloud = pointCloud(pc_data, 'Color', C);
pc_ptCloudOut = pctransform(pc_ptCloud, tform); 

% Plot the rotated point cloud
figure; 
% subplot(1,2,1); pcshow(pc_ptCloud, 'MarkerSize', 60); hold on;
% title('Origial Point Cloud with Pose'); set(gca,'zdir','reverse');
% view(-90, 90);

%subplot(1,2,2); 
pcshow(pc_ptCloudOut, 'MarkerSize', 75);
% title('Rotated Point Cloud with Pose'); 
% set(gca,'zdir','reverse'); 
axis equal;

%% Calculate building offset (point cloud meters => lat/lon)
sensor_Dist(1) = pc_ptCloudOut.Location(end-1, 1) - pc_ptCloudOut.Location(end, 1);
sensor_Dist(2) = -1 * (pc_ptCloudOut.Location(end-1, 2) - pc_ptCloudOut.Location(end, 2));
disp(['DETAILS: Sensor distance from corner: ' num2str(sensor_Dist(1)) ', ' ...
    num2str(sensor_Dist(2)) ' meters']);

sensor_Loc(1) = geo_bldg_face(1) + rad2deg((sensor_Dist(1) / 6372800.0));
sensor_Loc(2) = geo_bldg_face(2) + rad2deg((sensor_Dist(2) / 6372800.0) / (cosd(geo_bldg_face(1))));

openfig('geo_fig.fig'); hold on;
sensor_Mercator = geo2xyPt(sensor_Loc);
plot(sensor_Mercator(1), sensor_Mercator(2), 'b*'); hold on;

% Draw the arrow (Web Mercator)
sensor_dir(1) = cos(deg2rad(sensor_Yaw - 90));
sensor_dir(2) = sin(deg2rad(sensor_Yaw - 90));
sensor_dir = sensor_dir / norm(sensor_dir); % Scale the arrow length
quiver(sensor_Mercator(1), sensor_Mercator(2), sensor_dir(1), sensor_dir(2), 0.0002);

% Display the output
disp(['[RESULT] Sensor Yaw: ' num2str(sensor_Yaw) ' degrees from north']);
disp(['[RESULT] Sensor Position: ' num2str(sensor_Loc(1)) ...
    ' latitude, ' num2str(sensor_Loc(2)), ' longitude']);
disp(' ');

%% Figures and general cleanup
autoArrangeFigures();
clear A; clear tform; clear C; clear ans; clear dist;
PoseIdx = kf_ID + 1; % zero indexed image, 1 indexed data
rawDat = importdata('MapPoints.csv', ',', 1);

% 1 indexed point IDs (not zero)
kfIDAll = rawDat.data(:,1);
ptIDAll = rawDat.data(:,2);
ptPosAll = rawDat.data(:,3:5);

% Only use points in the current keyframe
% idx = find(kfIDAll == PoseIdx);
% ptPosAll = [ptPosAll(idx, 1) ptPosAll(idx, 2) ptPosAll(idx, 3)];

%% RANSAC fit plane
rejectionThreshold = 0.3; trials = 0; % thresh = 0.3, trial = 0
[B, P, inliers] = ransacfitplane(ptPosAll', rejectionThreshold, trials);
inlierPts = ptPosAll(inliers,:);

% Show the points in the plane along with statistics
%scatter3(inlierPts(:,1), inlierPts(:,2), inlierPts(:,3), 'r*'); hold on;
disp(['DETAILS: Total number of points in view: ' num2str(length(ptPosAll))]);
disp(['DETAILS: Number of points in plane: ' num2str(length(inlierPts))]);

%% Fit a vertical plane to the culled data
[n,V,p] = affine_fit(inlierPts);
for i = 1:length(inlierPts)
    inlierPts(i,:) = inlierPts(i,:) - dot(n', (inlierPts(i,:)-p) ) * n';
end

% Check the angle between the plane and gravity
n_ref = [0 0 1]; 
angle = atan2(norm(cross(n_ref,n)), dot(n_ref,n));
angle = abs(angle * 180 / pi) - 90;
disp(['DETAILS: The RANSAC calculated plane is : ' num2str(angle, 3) ' degrees from vertical']);

angle = acos(dot(n_ref,n) / dot(norm(n_ref),norm(n)));
angle = abs(angle * 180 / pi) - 90;
disp(['DETAILS: The Alternate RANSAC calculated plane is : ' num2str(angle, 3) ' degrees from vertical']);

% Get the yaw of the plane
pc_bldgYaw = atan(-n(2)/n(1)) * 180 / pi;
disp(['DETAILS: Point cloud yaw offset alternate is: ' num2str(pc_bldgYaw, 3) ' degrees']);

% Show the results
pc_fig = figure;

scatter3(ptPosAll(:,1), ptPosAll(:,2), ptPosAll(:,3), 'b'); hold on;
scatter3(inlierPts(:,1), inlierPts(:,2), inlierPts(:,3), 'r', 'filled'); hold on;
%h = scatter3(ptPos(:,1), ptPos(:,2), ptPos(:,3), 'md', 'filled'); hold on;
%set(h, 'SizeData', 100);

%% KeyFrame Poses
rawDat = importdata('Poses.csv', ',', 1);

% Parse input dat
kfIdAll = rawDat.data(:,1);
kfPosAll = [-rawDat.data(:,4) rawDat.data(:,2) rawDat.data(:,3)]; %2, 3, 4
kfRotAll = rawDat.data(:,5:13);
idx = find(kfIdAll == PoseIdx);

pc_kfPos = [kfPosAll(idx, 1) kfPosAll(idx, 2) kfPosAll(idx, 3)];
pc_kfRot = kfRotAll(idx, 1:9);
pc_kfRot = reshape(pc_kfRot, 3, 3);

% Show all keyframes
%scatter3(kfPosAll(:,1), kfPosAll(:,2), kfPosAll(:,3), 'g', 'filled'); hold on;

% Show current keyframe
scatter3(pc_kfPos(1), pc_kfPos(2), pc_kfPos(3), 'g', 'filled'); hold on;
xlabel('x'); ylabel('y'); zlabel('z');

%% Point Cloud & Sensor Orientation (taking into account cordinate frames)
pc_kfOrient_Native = SpinCalc('DCMtoEA123', pc_kfRot, 5, 1);

pc_kfRotAligned =  [pc_kfRot(1,3)  -pc_kfRot(1,1)  -pc_kfRot(1,2);
                    pc_kfRot(2,3)  -pc_kfRot(2,1)  -pc_kfRot(2,2);
                    pc_kfRot(3,3)  -pc_kfRot(3,1)  -pc_kfRot(3,2)];      
pc_kfOrient = SpinCalc('DCMtoEA123', pc_kfRotAligned, 5, 1);

% Degrees limited to +/-180 not 0 to 360
for i = 1:3
    if( abs(pc_kfOrient(i)) > 180)
        disp(['WARNING: Point cloud KF sensor angle ' num2str(i) ' > 180 degrees.. adjusting.']);
        pc_kfOrient(i) = pc_kfOrient(i) - 360;
    end
end

pc_kfDCM = SpinCalc('EA123toDCM', pc_kfOrient, 5, 1);     
                
% Plot the camera
% warning('off', 'all');
% plotCamera('Location', pc_kfPos, 'Orientation', pc_kfDCM, 'Size', 1);
% plotCamera('Location', pc_kfPos, 'Orientation', pc_kfRotAligned, 'Size', 1, 'Color', 'm');
% warning('on', 'all');
xlabel('x'); ylabel('y'); zlabel('z');

% Save data to globals
pc_n = n; pc_p = p; pc_data = [inlierPts; pc_kfPos];
save('globals.mat', 'pc_kfOrient', 'pc_kfOrient_Native', 'pc_n', ...
    'pc_p', 'pc_bldgYaw', 'pc_data', 'maxIter', 'kf_ID');
savefig(pc_fig, 'pc_fig.fig');

%% Show the plane
[A,B,C,D] = planeEquation(n, -p);
xLim = [min(inlierPts(:,1)) max(inlierPts(:,1))];
zLim = [min(inlierPts(:,3)) max(inlierPts(:,3))];
[Xdraw,Zdraw] = meshgrid(xLim, zLim);
Ydraw = (A * Xdraw + C * Zdraw + D)/ (-B);
reOrder = [1 2  4 3];

patch(Xdraw(reOrder), Ydraw(reOrder), Zdraw(reOrder), 'g');
xlabel('x'); ylabel('y'); zlabel('z');
% set(gca,'zdir','reverse'); 
axis equal;

title('Plane fit to 3D features');

save('globals.mat',  'pc_kfPos', 'pc_kfRot', '-append');

%% Reset the workspace (maxIter = 52; kf_ID = 32; %25)
resetWorkspace; maxIter = 0; kf_ID = 0; %25

for iter = 0:maxIter
    close all;
    
    %% Only if multiple loops
    if maxIter > 1
        kf_ID = iter;
        out_Loc = [0 0]; out_Yaw = 0; out_Dist = [0 0];
        save('summary.mat',  'out_Dist', 'out_Loc', 'out_Yaw');
    end
    
    disp(['DETAILS: Image is KeyFrame_' num2str(kf_ID) '.bmp']);
    
    %% Image recognition
    %img = imread('bin/google_image_recognition.png');
    img = imread(['experiment_data\KeyFrame_' num2str(kf_ID) '.bmp']);
    imshow(img);

    %% SLAM plane fitting
    addpath(genpath('slam'));
    analyse_orb_map_run_me;
    rmpath(genpath('slam'));

    %% Geo analysis
    addpath(genpath('geo'));
    get_map_data;
    rmpath(genpath('geo'));

    %% Edge detection
    addpath(genpath('edge_detection'));
    edges_run_me;
    rmpath(genpath('edge_detection'));

    if (~im_isEdgeFound)
        disp('FAILED');
        disp(' ');
        continue;
    end
    
    %% Solve for sensor pose, and present results
    solveForPose;

    %% Only if looping
    if maxIter > 1
        load('summary.mat');
        if (im_isEdgeFound)
            out_Loc = [out_Loc; sensor_Loc];
            out_Yaw = [out_Yaw; sensor_Yaw];
            out_Dist = [out_Dist; sensor_Dist];
            save('summary.mat',  'out_Loc', 'out_Yaw', 'out_Dist', '-append');

            if length(out_Yaw) == 101
                break;
            end
        end
    end
    
end

if maxIter > 1
    % Clear the first (empty) element
    out_Dist(1, :) = [];
    out_Loc(1, :) = [];
    out_Yaw(1) = [];
end
resetWorkspace;
load('summary.mat');

out_Pos(1,:) = []; 
out_Yaw(1) = [];
out_Dist(1,:) = []; 

% Heading
out_Yaw_Mean = mean(out_Yaw);
out_Yaw_Error = out_Yaw - out_Yaw_Mean;
out_Res_Yaw_Std = std(out_Yaw_Error);
out_Res_Yaw_Max = max(abs(out_Yaw_Error));

% Position
out_Dist_Mean = mean(out_Dist);
out_Dist_Error = out_Dist - out_Dist_Mean;
out_Res_Pos_Std = std(out_Dist_Error);
out_Res_Pos_Max = max(abs(out_Dist_Error));

out_Res_Pos_Std_Abs = sqrt(out_Res_Pos_Std(1)^2 + out_Res_Pos_Std(2)^2);
out_Res_Pos_Std_Max = sqrt(out_Res_Pos_Max(1)^2 + out_Res_Pos_Max(2)^2);
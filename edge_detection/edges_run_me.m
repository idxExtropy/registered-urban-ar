% Demo for Structured Edge Detector (please see readme.txt first).
load 'globals.mat';

%% set opts for training (see edgesTrain.m)
opts=edgesTrain();                % default options (good settings)
opts.modelDir='models/';          % model will be in models/forest
opts.modelFnm='modelBsds';        % model name
opts.nPos=5e5; opts.nNeg=5e5;     % decrease to speedup training
opts.useParfor=0;                 % parallelize if sufficient memory

%% train edge detector (~20m/8Gb per tree, proportional to nPos/nNeg)
%tic, 
model=edgesTrain(opts); % will load model if already trained
%toc; 

%% set detection parameters (can set after training)
model.opts.multiscale=1;%0        % for top accuracy set multiscale=1
model.opts.sharpen=2;%2           % for top speed set sharpen=0
model.opts.nTreesEval=4;%4        % for top speed set nTreesEval=1
model.opts.nThreads=4;%4          % max number threads for evaluation
model.opts.nms=0;%0               % set to true to enable nms

%% evaluate edge detector on BSDS500 (see edgesEval.m)
if(0), edgesEval( model, 'show',1, 'name','' ); end

%% detect edge and visualize results
I0 = imread(['..\experiment_data\KeyFrame_' num2str(kf_ID) '.bmp']);
I(:,:,1) = I0; I(:,:,2) = I0; I(:,:,3) = I0;

% Camera calibration parameters
intrinsics =   [804.89804317119831  0                   0; ...
                0                   806.25511004337602  0; ...
                602.3930486250797   469.82355641685018  0] ./ 2;
cameraParams = cameraParameters('IntrinsicMatrix', intrinsics, ...
    'RadialDistortion', [-0.13917442 -0.01265713 0.00190636] .* 1.0, ...
	'NumRadialDistortionCoefficients', 3);

% Undistort image
I = undistortImage(I, cameraParams);

% roll the image (TODO: Roll is off.. should be degrees!);
%I = imrotate(I, -pc_kfOrient(1) * pi / 180); % works [32]
%I = imrotate(I, pc_kfOrient(1));
I = imRotateCrop(I, pc_kfOrient(1));

%tic, 
E = 255 - edgesDetect(I,model); 
%toc
figure; 
subplot(1, 2, 1); im(I); 
subplot(1, 2, 2); im(E);

angTol = 5; magTol = 0.8;
[mag, ang] = imgradient(E);
imVertical = (ang >= 180 - angTol | ang <= -180 + angTol ) & mag > magTol;

se = strel('sphere',1); imVertical = imdilate(imVertical, se); 
se = strel('line',11,90); imVertical = imdilate(imVertical, se); 
imVertical = bwareaopen(imVertical, 125);

imgBorderTol = 20;
for width = 1:length(imVertical(1,:))
    for height = 1:length(imVertical(:,1))
        if height < imgBorderTol || width < imgBorderTol
           imVertical(height,width) = 0; 
        end
    end
end

% Find largest blob centroid
% TO DO: find plane edge relative to image to know
% where to expect the correct edge to appear..
hblob = vision.BlobAnalysis;
hblob.AreaOutputPort = true;
hblob.BoundingBoxOutputPort = false;
[area, centroid] = step(hblob, imVertical); % [left top]
idx = find(max(area));

if isempty(centroid)
    im_isEdgeFound = 0; im_EdgeYaw = 0;
    save('../globals.mat',  'im_EdgeYaw', 'im_isEdgeFound', '-append');
    cd ..;
    return;
else
    im_isEdgeFound = 1;
end

% InertiaCam parameters
fc = [804.89804317119831 806.25511004337602] ./ 2;
cc = [602.3930486250797 469.82355641685018] ./ 2;
kc = [-0.13917442 -0.01265713 0.00190636 0.00284389] .* 1.0;
kc = [0 0 0 0];
[u, v] = undistortCoords(centroid(1), centroid(2), fc, cc, kc);
%[u, v] = undistortCoords(centroid(1), centroid(2), fc, cc, kc);

im_EdgeYaw = u * 180 / pi;
save('../globals.mat',  'im_EdgeYaw', 'im_isEdgeFound', '-append');

figure; 
imOut = zeros(size(imVertical));
for width = 1:length(imOut(1,:))
    for height = 1:length(imOut(:,1))
        if width == round(centroid(1), 0)
           imOut(height,width) = 255; 
        end
    end
end
imshow(imOut);
cd ..;
% Google Street Map API Key: AIzaSyC7pcvBYV1h7a93_3q0yrNihbvLIP2-BtE
gps(1,:) = '42.358846,-71.057833';
gps(2,:) = '42.358860,-71.057833';
gps(3,:) = '42.358797,-71.057835';
gps(4,:) = '42.358760,-71.057835';
gps(5,:) = '42.358737,-71.057835';
gps(6,:) = '42.358700,-71.057835';
gps(7,:) = '42.358613,-71.057840';
gps(8,:) = '42.358480,-71.057840';
gps(9,:) = '42.358558,-71.057841';
gps(10,:) = '42.358525,-71.057841';
gps(11,:) = '42.358492,-71.057844';
gps(12,:) = '42.358450,-71.057844';
gps(13,:) = '42.358426,-71.057848';
gps(14,:) = '42.358400,-71.057848';

gps_filename(1,:) = '42.358846,-71.057833';
gps_filename(2,:) = '42.358860,-71.057833';
gps_filename(3,:) = '42.358797,-71.057835';
gps_filename(4,:) = '42.358760,-71.057835';
gps_filename(5,:) = '42.358737,-71.057835';
gps_filename(6,:) = '42.358700,-71.057835';
gps_filename(7,:) = '42.358613,-71.057840';
gps_filename(8,:) = '42.358480,-71.057840';
gps_filename(9,:) = '42.358558,-71.057841';
gps_filename(10,:) = '42.358525,-71.057841';
gps_filename(11,:) = '42.358492,-71.057844';
gps_filename(12,:) = '42.358450,-71.057844';
gps_filename(13,:) = '42.358426,-71.057848';
gps_filename(14,:) = '42.358400,-71.057848';

% Nearby streetview images with the 15 degree heading increments
for idx = 1:14
    for yaw = 90:90:90
        url_img = ['https://maps.googleapis.com/maps/api/streetview?size=640x480&location=' gps(idx,:) '&heading=' num2str(yaw) '&pitch=00&key=AIzaSyC7pcvBYV1h7a93_3q0yrNihbvLIP2-BtE'];
        street_view_img = ['data/straight_walk_2/street_view/ref_' gps_filename(idx,:) '_' num2str(yaw) '.jpg'];
        svImgOrig = imread(url_img, 'jpg');
        svImg = rgb2gray(svImgOrig);
        imwrite(svImg, street_view_img);
    end
end

for idx = 1:7
    for yaw = 0:15:270
        
    end
end

%% Point matching and relative pose estimate
refImg = imread('data/straight_walk_2/images/image_168573.bmp');
refPoints = detectSURFFeatures(refImg);
[refFeatures, refPoints] = extractFeatures(refImg, refPoints);

idx = 1;
for yaw = 0:45:360
    testImg = imread(['data/test_image_' num2str(yaw) '.jpg']);
    testImg = rgb2gray(testImg);
    testPoints = detectSURFFeatures(testImg);
    [testFeatures, testPoints] = extractFeatures(testImg, testPoints);
    
    % Match features between images
    scenePairs = matchFeatures(refFeatures, testFeatures, 'MatchThreshold', 99);
    matchedRefPoints = refPoints(scenePairs(:, 1), :);
    matchedTestPoints = testPoints(scenePairs(:, 2), :);
    
    % Show matches
    figure;
    showMatchedFeatures(refImg, testImg, matchedRefPoints, matchedTestPoints, 'montage');
    title('Matched Points (all)');
    
    % Remove outliers and compute affine transformation
    if (matchedRefPoints.Count > 25 && matchedTestPoints.Count > 25)
        [tform, inlierRefPoints, inlierTestPoints] = estimateGeometricTransform(matchedRefPoints, matchedTestPoints, 'affine');

        if (inlierRefPoints.Count > 25 && inlierTestPoints.Count > 25)
            figure;
            showMatchedFeatures(refImg, testImg, inlierRefPoints, inlierTestPoints, 'montage');
            title('Matched Points (Inliers Only)');
        end
    end
    idx = idx + 1;
end

%% Structural similarity index
refImg = imread('data/around_the_block_2/images/image_169893.bmp');

idx = 1;
for yaw = 0:45:270
    testImg = imread(['data/test_image_' num2str(yaw) '.jpg']);
    testImg = rgb2gray(testImg);
    s(idx) = ssim(testImg, refImg);
    c(idx) = corr2(testImg, refImg);
    p(idx) = psnr(testImg, refImg);
    idx = idx + 1;
end

% Compute results
sNorm = (s - min(s)) / ( max(s) - min(s) );
cNorm = (c - min(c)) / ( max(c) - min(c) );
pNorm = (p - min(p)) / ( max(p) - min(p) );
aNorm = (sNorm + cNorm + pNorm) ./ 3;

subplot(4, 1, 1); plot(sNorm, '*'); title('simNormalized');
subplot(4, 1, 2); plot(cNorm, '*'); title('corNormalized');
subplot(4, 1, 3); plot(pNorm, '*'); title('pNormalized');
subplot(4, 1, 4); plot(aNorm, '*'); title('average');

bestMatch(1) = find(s == max(s));
bestMatch(2) = find(c == max(c));
bestMatch(3) = find(p == max(p));
bestMatch(4) = find(aNorm == max(aNorm));
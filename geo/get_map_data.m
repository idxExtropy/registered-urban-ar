clear magLine;
getWebDataUpdate = 0;
filename = 'map_test.osm';

%% Read data from the network
magHeading = -104; % Note, plot only works because data is flipped on Y
disp(['DETAILS: Magnetometer Heading Estimate (magnetic north): ' num2str(magHeading)]);
roughLoc = [42.531068, -71.281368];

[XYZ, H, DEC, DIP, F] = wrldmagm(0, roughLoc(1), roughLoc(2), decyear(2017,1,1), '2015');
trueHeading = magHeading - DEC;
disp(['DETAILS: Magnetometer Heading Estimate (map north): ' num2str(trueHeading)]);

latBB = [roughLoc(1) - 0.001; roughLoc(1) + 0.001];
lonBB = [roughLoc(2) - 0.001; roughLoc(2) + 0.001];

if getWebDataUpdate == 1
    url = ['http://api.openstreetmap.org/api/0.6/map?bbox=' ...
        num2str(lonBB(1)) ',' num2str(latBB(1)) ',' num2str(lonBB(2)) ',' num2str(latBB(2))];
    outfilename = websave(filename,url);
end
%% Show the data (visual inspection - debugging
map = loadosm(filename);

% Draw rough initial location
loc = geo2xyPt(roughLoc); uncert = 0.0003;

% Compute 3 intersecting lines from mag heading and origin
aYawRad(1) = trueHeading * pi / 180;
aYawRad(2) = (trueHeading+30) * pi / 180;
aYawRad(3) = (trueHeading-30) * pi / 180;
aDist = 0.0003;

for i=1:3
    aX(i) = sin(aYawRad(i)) * aDist + loc(1);
    aY(i) = cos(aYawRad(i)) * aDist + loc(2);
end

% Draw rough heading (from magnetometers): INCOMPLETE
magLine(1,:) = loc; magLine(2,:) = [aX(1); aY(1)]; magLine(3,:) = [NaN; NaN];
magLine(4,:) = loc; magLine(5,:) = [aX(2); aY(2)]; magLine(6,:) = [NaN; NaN];
magLine(7,:) = loc; magLine(8,:) = [aX(3); aY(3)];
magLine = magLine';

% Draw building edges
bldgs = find([map.ways.isBuilding]) ;
rawMapData = osmgetlines(map, bldgs);
lines = geo2xy(rawMapData) ; 

% Show building without mag lines
geo_fig = figure;
rectangle('Position', [loc(1)-0.5*uncert,loc(2)-0.5*uncert,uncert,uncert], ...
  'Curvature', [1,1], 'FaceColor', 'r'); hold on;
plot(lines(1,:), lines(2,:), 'g-', 'linewidth', 0.8);
set(gca,'ydir','reverse');
xlabel('Web Mercator X');
ylabel('Web Mercator Y');
title('Open Street Map Building Edges with Initial Position'); hold on;
axis equal; box on;
savefig(geo_fig, 'geo_fig.fig');

% Show building with mag lines
figure;
rectangle('Position', [loc(1)-0.5*uncert,loc(2)-0.5*uncert,uncert,uncert], ...
  'Curvature', [1,1], 'FaceColor', 'r'); hold on;
plot(magLine(1,:), magLine(2,:)); hold on;
plot(lines(1,:), lines(2,:), 'g-', 'linewidth', 0.8);
set(gca,'ydir','reverse');
xlabel('Web Mercator X');
ylabel('Web Mercator Y');
title('Open Street Map Building Edges with Initial Position & Heading');
axis equal; box on;

%% Find the closest building line that intersects with one (or all) of the mag lines
warning('off', 'all'); figure; k = 0;

% Center Line
l1 = [magLine(1,1) magLine(2,1) magLine(1,2) magLine(2,2)]; j = 1;
subplot(1,3,1);
for i = 1:length(lines)-1
    l2 = [lines(1,i) lines(2,i) lines(1,i+1) lines(2,i+1)];
    [x(j), y(j)] = lineintersect(l1,l2);
    
    if (~isnan(x(j)) && ~isnan(y(j)))
        line([l1(1) l1(3)],[l1(2) l1(4)], 'Color', 'r'); hold on;
        line([l2(1) l2(3)],[l2(2) l2(4)], 'Color', 'g'); hold on;
        set(gca,'ydir','reverse'); axis equal; box on;
        j = j + 1;
        
        % TODO: make general (-90 is for normal into the building)
        geo_bldgYaw = atan((l2(3)-l2(1))/-(l2(4)-l2(2))) * 180 / pi - 90;
        geo_initYaw = trueHeading; 
        geo_bldgLine = l2; 
        geo_initLoc = loc;
        geo_bldg_face = [rawMapData(1,i+1) rawMapData(2,i+1)];
        save('globals.mat', 'geo_initLoc', 'geo_bldgLine', ...
            'geo_initYaw', 'geo_bldgYaw', 'geo_bldg_face', '-append');
    end
end
title('Center Mag Line');
k = k + j - 1;

% Bottom Line
l1 = [magLine(1,4) magLine(2,4) magLine(1,5) magLine(2,5)]; j = 1;
subplot(1,3,2);
for i = 1:length(lines)-1
    l2 = [lines(1,i) lines(2,i) lines(1,i+1) lines(2,i+1)];
    [x(j), y(j)] = lineintersect(l1,l2);
    
    if (~isnan(x(j)) && ~isnan(y(j)))
        line([l1(1) l1(3)],[l1(2) l1(4)], 'Color', 'r'); hold on;
        line([l2(1) l2(3)],[l2(2) l2(4)], 'Color', 'g'); hold on;
        set(gca,'ydir','reverse'); axis equal; box on;
        j = j + 1;
    end
end
title('Bottom Mag Line');
k = k + j - 1;

% Top Line
l1 = [magLine(1,7) magLine(2,7) magLine(1,8) magLine(2,8)]; j = 1;
subplot(1,3,3);
for i = 1:length(lines)-1
    l2 = [lines(1,i) lines(2,i) lines(1,i+1) lines(2,i+1)];
    [x(j), y(j)] = lineintersect(l1,l2);
    
    if (~isnan(x(j)) && ~isnan(y(j)))
        line([l1(1) l1(3)],[l1(2) l1(4)], 'Color', 'r'); hold on;
        line([l2(1) l2(3)],[l2(2) l2(4)], 'Color', 'g'); hold on;
        set(gca,'ydir','reverse'); axis equal; box on;
        j = j + 1;
    end
end
title('Top Mag Line');
k = k + j - 1;

disp(['DETAILS: Found ' num2str(k) ' potential magnetometer heading-to-building intersections']);
if k > 0
   disp('DETAILS: Selecting best building surface.'); 
end

warning('on', 'all');
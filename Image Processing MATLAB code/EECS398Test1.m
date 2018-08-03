% Bowen Jin
% EECS 398 Automation Project Test 1
% Image Processing

tic; % Start timer.
clc; % Clear command window.
clearvars; % Get rid of variables from prior run of this m-file.
fprintf('Running test398.m...\n'); % Message sent to command window.
workspace; % Make sure the workspace panel with all the variables is showing.
imtool close all;  % Close all imtool figures.
format long g;
format compact;
captionFontSize = 14;

hasIPT = license('test', 'image_toolbox');% Check if the Image Processing Toolbox is installed.
if ~hasIPT
	% User does not have the toolbox installed.
	message = sprintf('Image Processing Toolbox not installed.\Exit?');
    reply = questdlg(message, 'Toolbox missing', 'Yes', 'No', 'Yes');
	if strcmpi(reply, 'Yes')
		% User said Yes, so exit.
		return;
	end
end


baseFileName = 'Test12.jpg';
folder = fileparts(which(baseFileName)); 
fullFileName = fullfile(folder, baseFileName);
if ~exist(fullFileName, 'file')
	% It doesn't exist in the current folder.
	% Look on the search path.
	if ~exist(baseFileName, 'file')
		% It doesn't exist on the search path either.
		% Alert message.
		warningMessage = sprintf('Error: the input image file\n%s\nwas not found.\nClick OK to exit.', fullFileName);
		uiwait(warndlg(warningMessage));
		fprintf(1, 'Finished running test398.m.\n');
		return;
	end
	% Found it on the search path.  Construct the file name.
	fullFileName = baseFileName; 
end


originalImage = imread(fullFileName);
% Check if the image is grayscale.
[rows, columns, numberOfColorChannels] = size(originalImage);
if numberOfColorChannels > 1
	% if not, do the conversion using standard book formula
	originalImage = rgb2gray(originalImage);
end

normalizedThresholdValue = 0.75; % In range 0 to 1.
thresholdValue = normalizedThresholdValue * max(max(originalImage)); % Gray Levels.
binaryImage = im2bw(originalImage, normalizedThresholdValue);       % threshold to binary

subplot(3, 3, 1);
imshow(binaryImage); 
title('Binary Image, obtained by thresholding', 'FontSize', captionFontSize); 

labeledImage = bwlabel(binaryImage, 8);     % Label each object
subplot(3, 3, 3);
imshow(labeledImage, []);  
title('Labeled Image', 'FontSize', captionFontSize);

objectMeasurements = regionprops(labeledImage, originalImage, "all");
numberOfObjects = size(objectMeasurements, 1);

subplot(3, 3, 6);
imshow(originalImage);
title('Outlines', 'FontSize', captionFontSize); 
axis image; 
hold on;
boundaries = bwboundaries(binaryImage);
numberOfBoundaries = size(boundaries, 1);
for k = 1 : numberOfBoundaries
	thisBoundary = boundaries{k};
	plot(thisBoundary(:,2), thisBoundary(:,1), 'g', 'LineWidth', 2);
end
hold off;

textFontSize = 14;	
labelShiftX = -7;	
objectECD = zeros(1, numberOfObjects);


fprintf(1,'Object #      Mean Intensity  Area   Perimeter    Centroid       Diameter\n');

for k = 1 : numberOfObjects          
	thisObjectsPixels = objectMeasurements(k).PixelIdxList;  % Get list of pixels in current object.
	meanGL = mean(originalImage(thisObjectsPixels)); % Find mean intensity (in original image!)
	meanGL2008a = objectMeasurements(k).MeanIntensity; % Mean again, but only for version >= R2008a
	
	objectArea = objectMeasurements(k).Area;		% Get area.
	objectPerimeter = objectMeasurements(k).Perimeter;		% Get perimeter.
	obejctCentroid = objectMeasurements(k).Centroid;		% Get centroid one at a time
	fprintf(1,'#%2d %17.1f %11.1f %8.1f %8.1f %8.1f % 8.1f\n', k, meanGL, objectArea, objectPerimeter, obejctCentroid, objectECD(k));
	text(obejctCentroid(1) + labelShiftX, obejctCentroid(2), num2str(k), 'FontSize', textFontSize, 'FontWeight', 'Bold');
end


vertical = any(binaryImage, 2);
horizontal = any(binaryImage, 1);
ymin = find(vertical, 1, 'first') % Y1
ymax = find(vertical, 1, 'last') % Y2
xmin = find(horizontal, 1, 'first') % X1
xmax = find(horizontal, 1, 'last') % X2
boxY = [ymin ymin ymax ymax ymin];
boxX = [xmin xmax xmax xmin xmin];

% Get the image size.
[rows columns numberOfColorChannels] = size(binaryImage);
% Create the spatial calibration factors.
realWorldUnitsPerPixel_x = (xmax - xmin) / columns;
realWorldUnitsPerPixel_y = (ymax - ymin) / rows;



realWorldX = xmin + realWorldUnitsPerPixel_x * obejctCentroid;
realWorldY = ymin + realWorldUnitsPerPixel_y * obejctCentroid;


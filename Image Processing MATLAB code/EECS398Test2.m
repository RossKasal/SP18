% Bowen Jin
% EECS 398 Automation Project Test 2
% Image Processing

clc;    % Clear the command window.
close all;  % Close all figures (except those of imtool.)
clear;  % Erase all existing variables. Or clearvars if you want.
workspace;  % Make sure the workspace panel is showing.
format long g;
format compact;
fontSize = 20;

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


% Read in a gray scale demo image.
baseFileName = 'Test12.jpg';
folder = fileparts(which(baseFileName)); % Determine where demo folder is (works with all versions).
% Get the full filename, with path prepended.
fullFileName = fullfile(folder, baseFileName);
% Check if file exists.
if ~exist(fullFileName, 'file')
	% File doesn't exist -- didn't find it there.  Check the search path for it.
	fullFileNameOnSearchPath = baseFileName; % No path this time.
	if ~exist(fullFileNameOnSearchPath, 'file')
		% Still didn't find it.  Alert user.
		errorMessage = sprintf('Error: %s does not exist in the search path folders.', fullFileName);
		uiwait(warndlg(errorMessage));
		return;
	end
end
% Read in image.
grayImage = imread(fullFileName);
% Get the dimensions of the image.  
% numberOfColorBands should be = 1.
[rows, columns, numberOfColorBands] = size(grayImage);
if numberOfColorBands > 1
	% Convert it to gray scale by taking only the green channel.
	grayImage = grayImage(:, :, 2); % Take green channel.
end
% Display the original gray scale image.
subplot(2, 2, 1);
imshow(grayImage, []);
title('Original Grayscale Image', 'FontSize', fontSize, 'Interpreter', 'None');

% Set up figure properties:
% Enlarge figure to full screen.
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
% Get rid of tool bar and pulldown menus that are along top of figure.
set(gcf, 'Toolbar', 'none', 'Menu', 'none');
% Give a name to the title bar.
set(gcf, 'Name', 'Demo by ImageAnalyst', 'NumberTitle', 'Off') 


%compute and display the histogram.
[pixelCount, grayLevels] = imhist(grayImage);
subplot(2, 2, 2); 
bar(grayLevels, pixelCount); % Plot it as a bar chart.
grid on;
title('Histogram of original image', 'FontSize', fontSize, 'Interpreter', 'None');
xlabel('Gray Level', 'FontSize', fontSize);
ylabel('Pixel Count', 'FontSize', fontSize);
xlim([0 grayLevels(end)]); % Scale x axis manually.

normalizedThresholdValue = 0.75; % In range 0 to 1.
thresholdValue = normalizedThresholdValue * max(max(grayImage)); % Gray Levels.
binaryImage = im2bw(grayImage, normalizedThresholdValue);       % threshold to binary
% Get rid of white touching the border
binaryImage = imclearborder(binaryImage);
% Display the image.
subplot(2, 2, 3);
imshow(binaryImage, []);
title('Binary Image', 'FontSize', fontSize, 'Interpreter', 'None');

% Get the largest blob.   
binaryImage = bwareafilt(binaryImage, 1);
% Fill holes.
binaryImage = imfill(binaryImage, 'holes');
% Display the image.
subplot(2, 2, 4);
imshow(binaryImage, []);
title('Largest Blob', 'FontSize', fontSize, 'Interpreter', 'None');

% Use regionprops to get bounding box of it
[labeledImage, numRegions] = bwlabel(binaryImage);
measurements = regionprops(labeledImage, 'Centroid', 'PixelList');
for k = 1 : numRegions
	% Get the centroid
	xCenter = measurements(k).Centroid(1);
	yCenter = measurements(k).Centroid(2);
	% Get the distances of every point in the blob to the centroid.
	allX = measurements(k).PixelList(:, 1);
	allY = measurements(k).PixelList(:, 2);
	% Sum up the powers of the gray levels, weighted by the distance from the centroids.
	highestMomentNumber = 5;
	mu = zeros(highestMomentNumber, highestMomentNumber); % Allocate up to 5 moment (probably way overkill).
	for q = 1 : size(mu, 2) % in the column, x direction.
		for p = 1 : size(mu, 1) % in the row, y direction
			for location = 1 : length(allX)
				% For every pixel in this blob....
				% Sum up for moment "mu sub pq" - i.e. the pqth moment.
				thisGrayLevel = grayImage(allY(location), allX(location));
				mu(p, q) = mu(p, q) + ...
					(allX(location) - xCenter).^q * ...
					(allY(location) - yCenter).^p * ...
					double(thisGrayLevel);
			end
		end
	end
	fprintf('\nFor blob #%d, here are the spatial central moments:\n', k);
	% Display the values of mu in the command window.
	mu
    %Display the values of centroid in the command window.
    fprintf('\nBlob Centroid(x,y)\n',k);
    xCenter, yCenter
end

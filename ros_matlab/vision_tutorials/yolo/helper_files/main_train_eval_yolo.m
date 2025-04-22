%% Train and Eval Yolo

%% Load image labeller data
training_data = load('./vision_tutorials/yolo/img_labeller_projs/marker_spam_label.mat');
training_data = objectDetectorTrainingData(training_data.gTruth);


%% Work with label imager training_data output
trainingData = training_data; % Retain training_data variable
summary(trainingData);

%% Create Training, Validation, and Testing Datasets
% Randomly shuffle data for training. Split the dataset into training, validation, and test sets. Select 60%  of the data for training, 10% for validation, and the rest for testing  the trained detector.
rng(0);
shuffledIndices = randperm(height(trainingData)); % Get table height and permute entries

idx = floor(0.6 * length(shuffledIndices) );

% Training Data
trainingIdx = 1:idx;
trainingDataTbl = trainingData(shuffledIndices(trainingIdx),:);

% Validation Data
validationIdx = idx+1 : idx + 1 + floor(0.1 * length(shuffledIndices) );
validationDataTbl = trainingData(shuffledIndices(validationIdx),:);

% Test Data
testIdx = validationIdx(end)+1 : length(shuffledIndices);
testDataTbl = trainingData(shuffledIndices(testIdx),:);

%% Create DataStores

% Training DS
imdsTrain = imageDatastore(trainingDataTbl.imageFilename);
bldsTrain = boxLabelDatastore(trainingDataTbl(:,2:end));

% Validation DS
imdsValidation  = imageDatastore(validationDataTbl.imageFilename);
bldsValidation  = boxLabelDatastore(validationDataTbl(:,2:end));

% Test DS
imdsTest  = imageDatastore(testDataTbl.imageFilename);
bldsTest  = boxLabelDatastore(testDataTbl(:,2:end));

%% Combined image and box label DS
trainingData    = combine(imdsTrain,bldsTrain);
validationData  = combine(imdsValidation,bldsValidation);
testData        = combine(imdsTest,bldsTest);

%% Diplay one training image/box label for visualization
data = read(trainingData);

I    = data{1};
bbox = data{2};
labels = data{3};

% Extract class names for this training sample
classNames = training_data.Properties.VariableNames(2:end);

% Create colors for labels
label_colors = im2uint8(lines( numel(classNames)) );

% Convert labels to categorical type
labels = categorical(labels,classNames); 

% Insert bbox, label, and color to image
annotatedImage = insertObjectAnnotation(I, 'Rectangle', ...
                                        bbox, cellstr(labels),...
                                        'Color',label_colors(int32(labels),:));
annotatedImage = imresize(annotatedImage,2);

figure(1), imshow(annotatedImage)

% Reset training data
reset(trainingData);

%% Create a YOLO v4 Object Detector Network

% Set the net input size
inputSize = [480,640,3];

% Extract class names
classNames = training_data.Properties.VariableNames(2:end);

%% Specify Achor Boxes

% To avoid scaling problems, images are resized by scaling pixels between 0 and 1. Bounding boxes are also scaled correspondingly.
rng("default")
trainingDataForEstimation = transform(trainingData,@(data)preprocessData(data,inputSize));

%% Estimate best number for anchor boxes
maxNumAnchors = 12;

meanIoU         = zeros([maxNumAnchors,1]);
anchorBoxes     = cell(maxNumAnchors, 1);

for k = 1:maxNumAnchors
    [anchorBoxes{k}, meanIoU(k)] = estimateAnchorBoxes(trainingDataForEstimation,k);    
end

figure, plot(1:maxNumAnchors,meanIoU,'-o');
xlabel("Number of Anchors"), ylabel("Mean IoU"), title("Number of Anchors vs. Mean IoU");

%% Sort Anchose Sizes based on Area
% Compute the area of all your anchor boxes by multiplying their width * height
area = anchors(:,1).*anchors(:,2); % N anchors: [width height]

% Sort the areas from large to small
[~,idx] = sort(area,"descend");

% Set them as sortedAnchors
sortedAnchors = anchors(idx,:);

%% Assign small, med, large anchors to each of the 3 YOLO net heads
thresh = round(best_num_anchors/3);
anchorBoxes = {sortedAnchors( 1             :thresh,     :)...
               sortedAnchors( (thresh+1)    :(2*thresh), :)...
               sortedAnchors( ((2*thresh)+1):(3*thresh), :)};

%%  Instantiate a YOLO v4 object detection network
pretrained_net = "csp-darknet53-coco"; % Or "tiny-yolov4-coco"
detector = yolov4ObjectDetector(pretrained_net,...
                                classNames,...
                                anchorBoxes,...
                                InputSize=inputSize);


%% Training Data Augmentation
% data augmentation improves training accuracy

networkInputSize = detector.InputSize;                                    @(x)augmentDataKeepingAspectRatio(x,networkInputSize,doAugmentation));
preprocessedTrainingData = transform(trainingData,@augmentData);

% Reset datastore (read position reset; data shuffling reset)
reset(preprocessedTrainingData);

%% Display samples of augmented data
augmentedData = cell(4,1);
for k = 1:4
    data = read(preprocessedTrainingData); % Images | BBox | label
    augmentedData{k} = insertShape(data{1},"rectangle",data{2});

    % Reset datastore (read position reset; data shuffling reset)
    reset(preprocessedTrainingData);
end

figure,
montage(augmentedData,BorderSize=10); % display multiple images as a rect montage to show differences from augmented data

%% Train the Yolo Detector

% Set the options
options = trainingOptions("adam", ...
                          GradientDecayFactor           =0.9, ...
                          SquaredGradientDecayFactor    =0.999, ...
                          InitialLearnRate              =0.001, ...
                          LearnRateSchedule             ="piecewise", ...   % or "none"
                          MiniBatchSize                 =8, ...             % or 4
                          L2Regularization              =0.0005, ...
                          MaxEpochs                     =10, ...
                          BatchNormalizationStatistics  ="moving",...
                          DispatchInBackground          =false, ...
                          ResetInputNormalization       =false, ...
                          Shuffle                       ="every-epoch", ...
                          VerboseFrequency              =20, ...
                          ValidationFrequency           =1000, ...
                          CheckpointPath                =tempdir, ...
                          ValidationData                =validationData, ...
                          OutputNetwork                 ="best-validation-loss",...
                          ExecutionEnvironment          = "cpu"... % can be "auto"... or "gpu"
                          );

t = tic;
[detector,info] = trainYOLOv4ObjectDetector(preprocessedTrainingData,detector,options)
elapsedTime = toc(t);

%% Save to your data folder
save("./vision_tutorials/detectors",'detector','info','elapsedTime');

%% Vizualize the training loss
figure;
subplot(2,1,1);
semilogy(1:info.OutputNetworkIteration,info.TrainingLoss);
title('Total Loss'); xlabel('Iteration'); ylabel('Total loss');

subplot(2,1,2);
plot(1:info.OutputNetworkIteration,info.BaseLearnRate);
title('Learning rate'); xlabel('Iteration'); ylabel('Learning rate');

%% Test the Detector
% Sanity Check: let's see if we get expected results on a single image. 
img = imread("allObj.jpg");
[bboxes,scores,labels] = detect(detector,img);

detectedImg = insertObjectAnnotation(img,"Rectangle",bboxes,labels);
figure, imshow(detectedImg);

%% Evaluate using Test Set
% We must evaluate how good the detector is. To this end, we will measure its performance based on our testing data.  
% evaluateObjectDetection is a method that will compute average precision information. 
detectionResults = detect(detector,testData,Threshold=0.01);

% Evaluate the object detector using average precision metric.
metrics = evaluateObjectDetection(detectionResults,testData);
classID = 1;
precision = metrics.ClassMetrics.Precision{classID};
recall = metrics.ClassMetrics.Recall{classID};

%% Visualize
figure
plot(recall,precision)
xlabel("Recall")
ylabel("Precision")
grid on
title(sprintf("Average Precision = %.2f",metrics.ClassMetrics.mAP(classID)))
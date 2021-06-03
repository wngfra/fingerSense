clear all
close all

folderDir = '../data';
fileList = dir(fullfile(folderDir, '*.csv'));

XTrain = [];
labels = [];
numParts = 6;
numFeatures = 16;

for i = 1:length(fileList)
    fileName = fullfile(folderDir, fileList(i).name);
    namegroups = split(fileList(i).name, '@');
    x = readmatrix(fileName);
    
    L = floor(length(x) / numParts);
    audio = reshape(x(1:L*numParts, :), L, numFeatures, numParts);
    label(1:numParts) = namegroups(1);
    
    audioSeqs = squeeze(num2cell(audio, [2, 1]));
    
    XTrain = [XTrain; audioSeqs];
    labels = [labels, label(1:numParts)];
end
YTrain = categorical(labels).';

% Prepare network
numHiddenUnits = 200;
numClasses = length(unique(labels));
layers = [ ...
    sequenceInputLayer(numFeatures)
    lstmLayer(numHiddenUnits,'OutputMode','last')
    fullyConnectedLayer(numClasses)
    softmaxLayer
    classificationLayer];

numObservations = numel(XTrain);
for i=1:numObservations
    sequence = XTrain{i};
    sequenceLengths(i) = size(sequence,2);
end

[sequenceLengths,idx] = sort(sequenceLengths);
XTrain = XTrain(idx);
YTrain = YTrain(idx);

maxEpochs = 100;
miniBatchSize = 20;

options = trainingOptions('adam', ...
    'ExecutionEnvironment','gpu', ...
    'GradientThreshold',1, ...
    'MaxEpochs',maxEpochs, ...
    'MiniBatchSize',miniBatchSize, ...
    'SequenceLength','longest', ...
    'Shuffle','never', ...
    'Verbose',0, ...
    'Plots','training-progress');

net = trainNetwork(XTrain,YTrain,layers,options);

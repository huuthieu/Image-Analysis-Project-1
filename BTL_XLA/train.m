
%train data
pathfiletrain = fullfile('DATATRAIN');
datatrain = imageDatastore(pathfiletrain,...
    'IncludeSubfolders',true,'LabelSource','foldernames');
[trainDigitData,valDigitData] = splitEachLabel(datatrain,800,'randomized');

%test data
pathfiletest = fullfile('DATATEST');
datatest = imageDatastore(pathfiletest,...
    'IncludeSubfolders',true,'LabelSource','foldernames');

layers = [
    imageInputLayer([28 28 1])
    
    convolution2dLayer(3,6,'Padding','same')
    batchNormalizationLayer
    reluLayer
    
    maxPooling2dLayer(2,'Stride',2)
    
    convolution2dLayer(3,32,'Padding','same')
    batchNormalizationLayer
    reluLayer
    
    maxPooling2dLayer(2,'Stride',2)
    
    convolution2dLayer(3,64,'Padding','same')
    batchNormalizationLayer
    reluLayer
    fullyConnectedLayer(120)
    fullyConnectedLayer(10)
    softmaxLayer
    classificationLayer];
options = trainingOptions('sgdm',...
    'MaxEpochs',30, ...
    'ValidationData',valDigitData,...
    'ValidationFrequency',30,...
    'Verbose',false,...
    'Plots','training-progress');

net = trainNetwork(trainDigitData,layers,options);
temp = classify(net,datatest);
temptrain = datatest.Labels;
accuracytrain = sum(temp == temptrain)/numel(temptrain)

save nhandangchuso_4.mat net
save filetrain_4.mat trainDigitData
save fileval_4.mat valDigitData
save filetest_4.mat datatest
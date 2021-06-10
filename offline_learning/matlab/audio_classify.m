close all
clear all

folderPath = './..';
fileList = dir(fullfile(folderPath, '*.wav'));

tracks = [];
labels = [];

for i=1:length(fileList)
    basename = fileList(i).name;
    fileName = fullfile(folderPath, basename);
    namegroup = split(basename, '.');
    [audio, Fs] = audioread(fileName);
    Fs = Fs/2;
    
    L = length(audio);
    numParts = floor(L/Fs);
    audioMatrix = reshape(audio(1:numParts*Fs), Fs, numParts);
    audioLabel = repmat(namegroup(1), 1, numParts);
    tracks = [tracks, audioMatrix];
    labels = [labels, audioLabel];
end

L = length(tracks);

Y = fft(tracks, L, 1);
P2 = abs(Y/L);
P1 = P2(1:L/2+1, :);
P1(2:end-1, :) = 2*P1(2:end-1, :);
Q = P1(1:150, :).';
V = cov(Q);
[COEFF,latent,explained] = pcacov(V);
X = Q * COEFF(:, 1:3);
gscatter3(X(:,1),X(:,2),X(:,3), labels)
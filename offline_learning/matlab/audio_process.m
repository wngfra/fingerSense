clear all
close all

folderDir = '../';
fileList = dir(fullfile(folderDir, '*.wav'));

tracks = []; % Store all stereo tracks
labels = [];
Fs = 44100; % Default sampling frequency
Ns = Fs/2;

for i = 1:length(fileList)
    fileName = fullfile(folderDir, fileList(i).name);
    namegroups = split(fileList(i).name, '_');
    [y, Fs] = audioread(fileName);
    
    % Split into 1s recordings
    numParts = fix(length(y)/Ns);
    audio = reshape(y(1:Ns*numParts, :), Ns, 2, numParts);
    label(1:numParts) = namegroups(1);
    
    tracks = cat(3, tracks, audio);
    labels = [labels, label(1:numParts)];
end

numTracks = size(tracks, 3);
% Store FFTs of stereo tracks
M.L = zeros(numTracks, Ns/2);
M.R = zeros(numTracks, Ns/2);
for j = 1:numTracks
    label = labels{j};
    data = tracks(:,:,j);
    
    L = length(data);
    X = fft(data, L, 1);
    Xs = abs(X/L);
    Y = Xs(2:L/2+1, :);
    Y(2:end-1, :) = 2*Y(2:end-1, :);
    
    f = Fs*(0:L/2-1)/L; % Frequency range
    
    
    M.L(j, :) = Y(:, 1).';  
    M.R(j, :) = Y(:, 2).';
end

VL = cov(M.L);
VR = cov(M.R);
[coeff,latent,explained] = pcacov(VL);
Q = M.L * coeff(:, 1:3);
figure(1)
gscatter3(Q(:,1), Q(:,2), Q(:,3), labels);
savefig('microphone_pca.fig')
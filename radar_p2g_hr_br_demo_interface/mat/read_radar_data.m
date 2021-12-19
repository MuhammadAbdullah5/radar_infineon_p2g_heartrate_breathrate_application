bdata = load('backgroundData.mat');
data  = load('rawIqData.mat');

Nc = 200;
Ns = 106;

% Get background data
bdata_rx1 = bdata.dataCell{1}.data_rx1;
bdata_rx2 = bdata.dataCell{1}.data_rx2;

bdata_rx1 = reshape(bdata_rx1, Ns, Nc);
bdata_rx2 = reshape(bdata_rx2, Ns, Nc);
bdata_rx1(:, 1) = bdata_rx1(:, 2);
bdata_rx2(:, 1) = bdata_rx2(:, 2);


% Get number of frames in the MAT file.
num_frames = length(data.dataCell);

for frame=5:num_frames
    
    % get data
    data_rx1 = data.dataCell{frame}.data_rx1;
    data_rx2 = data.dataCell{frame}.data_rx2;
    
    data_rx1 = reshape(data_rx1, Ns, Nc);
    data_rx2 = reshape(data_rx2, Ns, Nc);
    
    % subtract background 
    data_rx1 = data_rx1 - bdata_rx1;
    data_rx2 = data_rx2 - bdata_rx2;
    
    plot(real(data_rx1(:, :)));
    plot(abs(fftshift(fft(data_rx2(:, :).*blackman(106), 512), 1)));
end


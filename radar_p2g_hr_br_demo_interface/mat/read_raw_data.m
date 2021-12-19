% read raw data and save that to local structure and return that structure
function dataCell = read_raw_data(fileName)

fileId = fopen(fileName);

Ns = 106;
Nc = 200;

% check how many blocks are present.
frewind(fileId);
data = fscanf(fileId, '%f, %f', [2 Inf])';
data = data(:,1) + 1i * data(:,2);

len = Ns * Nc * 2;
num_frames = length(data) / len;
if num_frames - round(num_frames) ~= 0
    error('Incomplete data block in given file');
end

dataCell = {};
for n=1:num_frames
    frame_data = data((n-1)*len+1:n*len);
    frame_data = reshape(frame_data, Ns, Nc*2);
    frame_data_struct.data_rx1 = frame_data(:, 1:2:Nc*2);
    frame_data_struct.data_rx2 = frame_data(:, 2:2:Nc*2);
    dataCell{end+1} = frame_data_struct;
    figure(1);plot(real(frame_data_struct.data_rx1));
    figure(2);plot(real(frame_data_struct.data_rx2));
    shg;
end

name = [fileName(1:regexp(fileName, '[.]')), 'mat'];
save(name, 'dataCell');

end

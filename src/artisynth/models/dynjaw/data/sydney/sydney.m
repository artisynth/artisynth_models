%
% read jaw data from Sydney
%
% File definition from Grey Murray:
%
% 
%The file with 7 columns of data is the JAWS3D data file and shows from left to righ:
%
%data point number; time; x axis; y axis; z axis; jaw opening angle
%
%For the x axis: anterior-posterior; positive is posterior
%For the y axis: medial-lateral; positive is to right
%For the z axis: superior-inferior; positive is upwards.
%
%For the EMG data, the first column is time, the 2nd is EMG in volts. 
%



jawdata = load('PHANIF43.DAT');
npos = jawdata(:,1);
tpos = jawdata(:,2);
posrate = 1/(tpos(2)-tpos(1));
pos = jawdata(:,3:5);
angle = jawdata(:,6); % radians

emgdata = load('DATA431.DAT');
temg = emgdata(:,1);
emgrate = 1/(temg(2)-temg(1));
emg = emgdata(:,2);

%
% convert to artisynth coord frame (z-up, x-left, y-back) -- rigidbody
% position
%
probedata = [tpos, -pos(:,2), -pos(:,1), pos(:,3)];
outfile = 'jawPosition.txt';
header = sprintf('%d %d 1.0\nLinear %d explicit\n', ...
    tpos(1)*1000000000, tpos(length(tpos))*1000000000, size(probedata,2)-1);

fid = fopen(outfile, 'wt');
fprintf(fid,header);
fclose(fid);
save(outfile, '-append', '-ascii', 'probedata');

%
% convert to axis angle representation -- rigidbody orientation
%
n = length(tpos);
probedata = [tpos, ones(n,1), zeros(n,1), zeros(n,1), angle];
outfile = 'jawAxisangle.txt';

header = sprintf('%d %d 1.0\nLinear %d explicit\n', ...
    tpos(1)*1000000000, tpos(length(tpos))*1000000000, size(probedata,2)-1);

fid = fopen(outfile, 'wt');
fprintf(fid,header);
fclose(fid);
save(outfile, '-append', '-ascii', 'probedata');

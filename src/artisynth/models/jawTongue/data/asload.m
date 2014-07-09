%
% asload - load artisynth data file from time start to time stop
% - assumes 2 lines of header information
% - see doc/misc/artisynthFileFormat.txt
%
% params:
%  file - filename to load
%  start - optional start time in seconds
%  stop - optional stop time in seconds
%
% Ian Stavness - 28/08/2008


function [data, time] = asload(file, start, stop)


% check number and type of arguments
if nargin < 1
  error('Function requires one input argument');
elseif ~isstr(file)
  error('Input must be a string representing a filename');
end

fid = fopen(file);
if fid==-1
  error('File not found or permission denied');
end

% read two header lines (standard ArtiSynth data file format)
DELIMITER = ' ';
HEADERLINES = 2;
datafromfile = importdata(file, DELIMITER, HEADERLINES);
% assignin('base','alldata',datafromfile.('data'));
alldata = datafromfile.data;
[N,M] = size(alldata);
time = alldata(:,1);
data = alldata(:,2:M);

if (nargin < 2)
    return;
elseif (nargin < 3)
    stop = INF;
end

interval = [];
for t = [start, stop]
    z=abs(time-t);  
    interval = [interval, find(min(z)==z)]; 
end

range = interval(1)+1:interval(2);
time = alldata(range,1);
data = alldata(range,2:M);



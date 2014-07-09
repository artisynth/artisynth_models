execfile('r/r.py');
import time;

duration = 0.5;
addBreakPoint(duration);
tasks = ['a','i','rbunch','rtipup','rtipuphg'];
dir = 'r/data/'

for name in tasks:
	loadPosition(dir+name+'.pos');
	time.sleep(1);
	main.screenShot('r/out/bw/'+name+'.png');




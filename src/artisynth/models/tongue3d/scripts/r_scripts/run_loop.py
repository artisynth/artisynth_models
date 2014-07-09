execfile('r/r.py');
import time;

duration = 0.5;
addBreakPoint(duration);
tasks = ['a','i','rbunch','rtipup','rtipuphg'];

for name in tasks:    
	go(name);
	reset();
	run();
	waitForStop();
	preprocess(name);
	time.sleep(1);
	postprocess(name);
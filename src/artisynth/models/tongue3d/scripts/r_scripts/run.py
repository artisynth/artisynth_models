execfile('r/r.py');
import time;

duration = 0.5;
addBreakPoint(duration);
tasks = ['a','i','rbunch','rtipup','rtipuphg'];

# for name in tasks:                              
# 	go(name, 0.4);
# 	reset();
# 	run();
# 	waitForStop();
# 	main.rerender();
# 	save("r/data/"+name);
# 	main.screenShot("r/data/"+name+"_stress.png");
# 	print 'done task ' + name;

name = 'a';
go(name);
reset();
run();
waitForStop();
preprocess(name);
time.sleep(1);
postprocess(name);

name = 'i';
go(name);
run();
waitForStop();
preprocess(name);
time.sleep(1);
postprocess(name);

name = 'rbunch';
go(name);
run();
waitForStop();
preprocess(name);
time.sleep(1);
postprocess(name);

name = 'rtipup'
go(name);
run();
waitForStop();
preprocess(name);
time.sleep(1);
postprocess(name);

name = 'rtipuphg'
go(name);
run();
waitForStop();
preprocess(name);
time.sleep(1);
postprocess(name);

import time
from maspack.properties import *
execfile('r/r.py');

def getMaxRange(tasks, metric) :
	max = VectorNd(len(tasks));
	i = 0;
	for t in tasks:
		v = readVector(dir+t+'.'+metric);
		max.set(i, v.maxElement());
		i = i + 1;
	return max.maxElement();


def getStress(name) :
	return readVector(dir+name+'.'+metric);


def getStressRange() :
	max = VectorNd(len(tasks));
	i = 0;
	for t in tasks:
		v = getStressVals(t);
		max.set(i, v.maxElement());
		i += 1;
	print 'max '+max.toString()
	return max.maxElement();


def getStressVals(name) :
	return getStress(name);


def preShow() :
	filename = dir+name;
	print filename
	loadPosition(filename+'.pos');
#	loadStress(filename+'.stress');
	maxVal = getStressRange();
	stress = getStressVals(name);
#	tongue.setStressPlotRange(DoubleRange(0,maxVal));
	print 'max range %f' % maxVal
#	tongue.setStressPlotRanging(FemModel.Ranging.Fixed);
	setStress(stress);
	main.rerender();


def postShow() :
	main.screenShot('r/out/'+ref+'/'+view+'/'+name+'_'+ref+'_'+outname+'_'+metric+'.png');
	stress = getStress(name);
	writeVector('r/out/'+ref+'/data/'+name+'_'+ref+'.'+metric, stress);


########################################################################

#globals
duration = 0.3;
dir = 'r/data/'
tasks = ['a','i','rbunch','rtipup','rtipuphg'];
refs =  ['a','i'];
poses = ['rbunch','rtipup','rtipuphg'];
metrics = ['stress','strain','mstress'];

addBreakPoint(duration);
tongue = root().models().get(0).models().get(0);
#maxRange = getMaxRange(tasks,metrics[0]);
#tongue.setStressPlotRange(DoubleRange(0,maxRange));
#tongue.setStressPlotRanging(FemModel.Ranging.Fixed);
main.rerender()

#metric = 'stress'
#metric = 'strain'
metric = 'mstress'
ref='loop'
outname = 'abs'
view='lateral'
#view='sagittal'

for metric in metrics:
	print 'met is '+metric
	name = 'a';
	preShow()
	time.sleep(0.1)
	postShow()
	name = 'i';
	preShow()
	time.sleep(0.1)
	postShow()
	name = 'rbunch';
	preShow()
	time.sleep(0.1)
	postShow()
	name = 'rtipup';
	preShow()
	time.sleep(0.1)
	postShow()
	name = 'rtipuphg';
	preShow()
	time.sleep(0.1)
	postShow()




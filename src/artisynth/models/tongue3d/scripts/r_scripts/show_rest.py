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
	maxVal = getStressRange();
	print 'max range %f' % maxVal
	stress = getStressVals(name);
	maxVal = stressRangeMax;
	tongue.setStressPlotRange(DoubleRange(0,maxVal));
	tongue.setStressPlotRanging(FemModel.Ranging.Fixed);
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
metrics = ['stress','strain', 'mstress'];
stressRanges = [15, 1, 10];

addBreakPoint(duration);
tongue = root().models().get(0).models().get(0);
main.rerender()

ref='rest'
outname = 'abs'

#view='lateral'
#view='sagittal'

for i in range(0,len(metrics)):
	metric = metrics[i];
	stressRangeMax = stressRanges[i];
	for name in tasks:
		preShow()
		time.sleep(0.1)
		postShow()
		


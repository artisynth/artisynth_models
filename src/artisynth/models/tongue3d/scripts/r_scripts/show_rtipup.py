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


def getCompStress(name, ref) :
	print 'comp stress'
	stress = readVector(dir+name+'.'+metric);
	ref = readVector(dir+ref+'.'+metric);	
	stress.sub(stress,ref);
	for i in range(0,stress.size()):
		if (stress.get(i) < 0):
			stress.set(i,0);
	return stress;


def getRelStress(name, ref) :
	print 'rel stress'
	stress = readVector(dir+name+'.'+metric);
	ref = readVector(dir+ref+'.'+metric);
	stress.sub(stress,ref);
	stress.absolute();
	return stress;


def getStressRange() :
	max = VectorNd(len(poses)*len(refs));
	i = 0;
	for p in poses:
		for r in refs:
			v = getStressVals(p, r);
			max.set(i, v.maxElement());
			i += 1;
	print 'max '+max.toString()
	return max.maxElement();


def getStressVals(name, ref) :
	if (outname == 'comp'):
		return getCompStress(name, ref);
	else : 
		return getRelStress(name, ref);


def preShow() :
	filename = dir+name;
	print filename
	loadPosition(filename+'.pos');
	maxVal = getStressRange();
	stress = getStressVals(name, ref);
	maxVal = stressRangeMax
	print 'max range %f' % maxVal
	tongue.setStressPlotRange(DoubleRange(0,maxVal));
	tongue.setStressPlotRanging(FemModel.Ranging.Fixed);
	setStress(stress);
	main.rerender();


def postShow() :
	main.screenShot('r/out/'+outname+'/'+view+'/'+name+'_'+ref+'_'+outname+'_'+metric+'.png');
	stress = getStress(name);
	writeVector('r/out/'+outname+'/data/'+name+'_'+ref+'_'+outname+'.'+metric, stress);


########################################################################

#globals
duration = 0.3;
dir = 'r/data/'
tasks = ['a','i','rbunch','rtipup','rtipuphg'];
refs =  ['a','i'];
poses = ['rbunch','rtipup','rtipuphg'];
metrics = ['stress','strain', 'mstress'];

addBreakPoint(duration);
tongue = root().models().get(0).models().get(0);
main.rerender()

# outname = 'rel'
# stressRanges = [10, 1, 10];
# outname = 'comp'
# stressRanges = [2, 0.5, 5];

outnames = ['rel','comp']
stressRanges = [[10, 1, 10],[2, 0.5, 5]];


metric = 'mstress';
outname = 'comp';
stressRangeMax = 6;
name = 'rtipup';
ref = 'a';
preShow();




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


def getMinRange(tasks, metric) :
	min = VectorNd(len(tasks));
	i = 0;
	for t in tasks:
		v = readVector(dir+t+'.'+metric);
		min.set(i, v.minElement());
		i = i + 1;
	return min.minElement();


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


def getStressRangeMax() :
	max = VectorNd(len(poses)*len(refs));
	i = 0;
	for p in poses:
		for r in refs:
			v = getStressVals(p, r);
			max.set(i, v.maxElement());
			i += 1;
	print 'max '+max.toString()
	return max.maxElement();


def getStressRangeMin() :
	min = VectorNd(len(poses)*len(refs));
	i = 0;
	for p in poses:
		for r in refs:
			v = getStressVals(p, r);
			min.set(i, v.minElement());
			i += 1; 	
	print 'min '+min.toString()
	return min.minElement();


def getStressVals(name, ref) :
	if (outname == 'comp'):
		return getCompStress(name, ref);
	else : 
		return getRelStress(name, ref);


def diffPositions(refposfilename) :	
	tongue = root().models().get(0).models().get(0)
	p = Point3d();
	dist = VectorNd(tongue.getNodes().size());
	fd = open(refposfilename, 'r');
	s = fd.read().splitlines();	
	idx = 0;
	for n in tongue.getNodes():
		xyz = s[idx].split();
		p.set(float(xyz[0]), float(xyz[1]), float(xyz[2]));
		dist.set(idx, n.getPosition().distance(p));
		idx += 1;
	print 'distmax = %f' % dist.maxElement();
	return dist;


def preShow() :
	loadPosition(dir+name+'.pos');
	diff = diffPositions(dir+ref+'.pos');
	tongue.setStressPlotRange(DoubleRange(0,maxVal));
	tongue.setStressPlotRanging(FemModel.Ranging.Fixed);
	setStress(diff);
	main.rerender();
	print 'done pre show'
	return diff

def postShow(diff) :
	main.screenShot('r/out/'+outname+'/'+view+'/'+name+'_'+ref+'_'+outname+'.png');
	writeVector('r/out/'+outname+'/data/'+name+'_'+ref+'.'+outname, diff);
	print 'done post show'



########################################################################

#globals
duration = 0.3;
dir = 'r/data/'
tasks = ['a','i','rbunch','rtipup','rtipuphg'];
refs =  ['a','i'];
poses = ['rbunch','rtipup','rtipuphg'];

addBreakPoint(duration);
tongue = root().models().get(0).models().get(0);
main.rerender()

outname = 'disp';

view = 'lateral';
#view = 'sagittal'

maxVal = 25;

for ref in refs:
	for name in poses:
		diff = preShow();
		time.sleep(1);
		postShow(diff);


name = 'rbunch';
ref = 'i';





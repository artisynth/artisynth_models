from artisynth.models.tongue3d import *
from artisynth.models.jawTongue import *

# 	if JawHyoidFemMuscleTongue.isInstance(root()):
# 		loadMexEx("r/"+name+".mexex")
# 	else:
# 		tongue.loadFibreEx("r/"+name+".excitations")

#def go(name, scale) :
#	scaleMexEx(scale);


def go(name) :
	print 'doing task ' + name;
	mech = root().models().get(0)
	tongue = mech.models().get(0);	
	clearMexEx();
	loadMexEx("r/"+name+".mexex");
	if (name=='i'):
		mech.setMaxStepSizeSec(0.002)
	else:
		mech.setMaxStepSizeSec(0.005)
	if (tongue.getStressPlotRanging()==FemModel.Ranging.Auto):
		tongue.resetStressPlotRange();
	reset();
	run();


def addRampedProbes(name) :
	duration = 0.2;
	loadMexExProbes("r/"+name+".mexex", duration);


def preprocess(name) :
	tongue = root().models().get(0).models().get(0);	
	save("r/data/"+name);
	tongue.setSurfaceRendering(FemModel.SurfaceRender.Stress)
	tongue.resetStressPlotRange();
	main.rerender();	


def postprocess(name) :
	main.screenShot("r/data/"+name+"_stress.png");
	print 'done task ' + name;


def goRun(name, duration) :
	tongue = root().models().get(0).models().get(0);	
	tongue.resetStressPlotRange();
	reset();
	addBreakPoint(duration);
	run();
	waitForStop();
	main.rerender();
	save("r/data/"+name);
	main.screenShot("r/data/"+name+"_stress.png");
	print 'done task ' + name;


def save(name) : 
	print 'saving data for ' + name;
	saveMexEx(name+".mexex");
	saveStress(name+".stress");
	saveStrain(name+".strain");
	saveMuscleStress(name+".mstress");
	savePosition(name+".pos");


def load(name) :
	loadMexEx(name+".mexex");
	loadStress(name+".stress");
	loadStrain(name+".strain");
	loadPosition(name+".pos");


def scaleMexEx(scale) :
	tongue = root().models().get(0).models().get(0);
	for mex in tongue.getMuscleExciters():
	 	ex = mex.getExcitation();
	 	mex.setExcitation(scale*ex);


def saveMexEx(filename) :
	tongue = root().models().get(0).models().get(0)
	jaw = root().models().get(0)
	fd = open (filename, 'w');
	for ex in tongue.getMuscleExciters():
		if (ex.getExcitation() > 0):
			fd.write(ex.getName() + " " + Integer.toString(ex.getExcitation()) + "\n");
	for ex in jaw.getMuscleExciters():
		if (ex.getExcitation() > 0):
			fd.write(ex.getName() + " " + Integer.toString(ex.getExcitation()) + "\n");
	fd.close();


def loadMexEx(filename) :
	clearMexEx();
	tongue = root().models().get(0).models().get(0)
	jaw = root().models().get(0)
	fd = open (filename, 'r');
	s = fd.read().splitlines();
	for ss in s:
	 mex = tongue.getMuscleExciters().get(ss.split()[0])
	 if (mex == None):
	  mex = jaw.getMuscleExciters().get(ss.split()[0])
	 if (mex != None): 
	  mex.setExcitation(float(ss.split()[1]));
	fd.close();


def loadMexExProbes(filename, duration) :
	clearMexEx();
	tongue = root().models().get(0).models().get(0)
	jaw = root().models().get(0)
	fd = open (filename, 'r');
	s = fd.read().splitlines();
	for ss in s:
	 mex = tongue.getMuscleExciters().get(ss.split()[0])
	 if (mex == None):
	  mex = jaw.getMuscleExciters().get(ss.split()[0])
	  
	 if (mex != None): 
	  root().addMexProbe(mex, duration, float(ss.split()[1]));
	fd.close();


def clearMexEx():
	tongue = root().models().get(0).models().get(0)
	for ex in tongue.getMuscleExciters():
		ex.setExcitation(0);
	jaw = root().models().get(0)
	for ex in jaw.getMuscleExciters():
		ex.setExcitation(0);


def saveMuscleStress(filename) :
	tongue = root().models().get(0).models().get(0)
	tongue.setMuscleStress();
	saveStress(filename);


def saveStress(filename) :
	saveStressStrain(filename, 1);


def saveStrain(filename) :
	saveStressStrain(filename, 0);


def saveStressStrain(filename, doStress) :	
	tongue = root().models().get(0).models().get(0)
	fd = open (filename, 'w')
	for n in tongue.getNodes():
		if (doStress == 1):
			fd.write('%s\n'%n.getVonMisesStress());
		else:
			fd.write('%s\n'%n.getVonMisesStrain());
	fd.close();


def readVector(filename) :
	fd = open (filename, 'r');
	s = fd.read().splitlines();
	v = VectorNd(len(s));
	for i in range(0,len(s)):
		v.set(i,float(s[i]));
	fd.close();
	return v


def writeVector(filename, v) :
	fd = open (filename, 'w');
	for i in range(0,v.size()):
		fd.write('%g\n'%v.get(i));
	fd.close();


def setStress(vals) :
	setStressStrain(vals, 1);


def setStressStrain(vals, doStress) :
	tongue = root().models().get(0).models().get(0)
	if (tongue == None) :
		print "cannot find tongue model"
		return
	for i in range(0,vals.size()):
		if (doStress == 1):
			tongue.getNode(i).setStress(vals.get(i));
		else:
			tongue.getNode(i).setStrain(vals.get(i));
	main.rerender();


def loadStress(filename) :
	stress = readVector(filename);
	setStressStrain(stress, 1);


def loadStrain(filename) :
	strain = readVector(filename);
	setStressStrain(strain, 0);


def savePositionFem(fd):
	tongue = root().models().get(0).models().get(0)
	for n in tongue.getNodes():
		fd.write(n.getPosition().toString("%g")+"\n");


def savePositionBodies(fd):
	mech = root().models().get(0)
	for rb in mech.rigidBodies():
		if (rb.isDynamic()):
			fd.write(rb.getPosition().toString("%g")+" "+rb.getOrientation().toString("%g")+"\n");


def loadPositionFemAndBodies(fd) :	
	tongue = root().models().get(0).models().get(0)
	p = Point3d();
	s = fd.read().splitlines();	
	idx = 0;
	for n in tongue.getNodes():
		xyz = s[idx].split();
		p.set(float(xyz[0]), float(xyz[1]), float(xyz[2]));
		n.setPosition(p);
		idx += 1;
		
	mech = root().models().get(0)
	p = Point3d();
	for rb in mech.rigidBodies():
		if (rb.isDynamic()):
			pose = s[idx].split();
			rb.setPosition(Point3d(float(pose[0]), float(pose[1]), float(pose[2])));
			rb.setOrientation(AxisAngle(float(pose[3]), float(pose[4]), float(pose[5]), float(pose[6])));
			idx += 1;
	mech.updatePosState();
	main.rerender();




def savePosition(filename) :
	fd = open (filename, 'w')
	savePositionFem(fd);
	savePositionBodies(fd);
	fd.close();


def loadPosition(filename) :
	fd = open (filename, 'r')
	loadPositionFemAndBodies(fd);
	fd.close();
	main.rerender();




for bundle in sel[0].getMuscleBundles():
 name = bundle.getName()
 file = open(name+".bun",'w')
 for f in bundle.getFibres():
  file.write(f.getFirstPoint().getPosition().toString("%8.4f")+"\n")
  file.write(f.getSecondPoint().getPosition().toString("%8.4f")+"\n")
 file.close()




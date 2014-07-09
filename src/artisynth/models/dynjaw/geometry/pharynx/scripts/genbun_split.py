for bundle in sel[0].getMuscleBundles():
 name = bundle.getName()
 cnt = 1
 file = open(name+"_"+str(cnt)+".bun",'w')
 prevPnt = bundle.getFibres().get(0).getFirstPoint();
 for f in bundle.getFibres():
  if (prevPnt != f.getFirstPoint()):
   file.write(prevPnt.getPosition().toString("%8.4f")+"\n")
   file.close()
   cnt += 1
   file = open(name+"_"+str(cnt)+".bun",'w')
  file.write(f.getFirstPoint().getPosition().toString("%8.4f")+"\n")
  prevPnt = f.getSecondPoint();
 file.close()



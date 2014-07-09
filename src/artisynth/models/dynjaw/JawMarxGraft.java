package artisynth.models.dynjaw;

public class JawMarxGraft extends JawHemi
{

   String jawgraftMesh = "postcanmarx.obj";
   
   public JawMarxGraft()
   {
      super();
   }
        
   public JawMarxGraft(String name)
   {
      super(name);
      setJawReconMesh(jawgraftMesh);
      addJoint (leftSideGraft);
   }

   
}

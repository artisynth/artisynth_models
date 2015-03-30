package artisynth.models.dynjaw;

import java.io.IOException;

public class JawMarxGraft extends JawHemi
{

   String jawgraftMesh = "postcanmarx.obj";
   
   public JawMarxGraft()
   {
      super();
   }
        
   @Override
   public void build (String[] args) throws IOException {
      super.build (args);
      setJawReconMesh(jawgraftMesh);
      addJoint (leftSideGraft);
   }

   
}

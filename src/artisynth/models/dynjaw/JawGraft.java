package artisynth.models.dynjaw;

import java.io.IOException;

import artisynth.core.mechmodels.Muscle;

public class JawGraft extends JawHemi
{

   String jawgraftMesh = "postcangraft.obj";
   
   public JawGraft()
   {
      super();
   }
        
   @Override
   public void build (String[] args) throws IOException {
      super.build (args);
      setJawReconMesh(jawgraftMesh);
      addJoint (leftSideGraft);
      doReattach ();
   }
   
   public void doReattach()
   {
      
      
      String reattachedMuscles[] = new String[]{
//         "at",
         "mt",
         "pt",
//         "ip",
//         "sp",
//         "dm",
//         "sm",
//         "am",
//         "pm",
//         "mp"
      };
      
      for (int i = 0; i < reattachedMuscles.length; i++)
      {
         String muscleName = (leftSideGraft?"l":"r")+reattachedMuscles[i];
         Muscle m = (Muscle)myJawModel.axialSprings().get(muscleName);
         m.setEnabled(true);
      }

   }
   
}

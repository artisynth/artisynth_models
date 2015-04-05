package artisynth.models.dynjaw;

import java.io.IOException;
import java.util.ArrayList;

import artisynth.core.materials.PeckAxialMuscle;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ComponentListView;

public class JawLongPharyngealDemo extends JawLarynxDemo{

   /**
    * @param args
    */
   public JawLongPharyngealDemo () {
      // TODO Auto-generated constructor stub
      super();
   }

   
   @Override
   public void build (String[] args) throws IOException {
      super.build (args);
      
      //Get a list of rigid bodies in this myJawModel
      System.out.println("Hi from JawLongPharyngealDemo.build()");
      ComponentListView<RigidBody> rigidBodies = myJawModel.rigidBodies ();
      for (RigidBody rb : rigidBodies){
         System.out.println(rb.getName ());
      }
      
   }
}

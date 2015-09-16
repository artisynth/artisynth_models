package artisynth.models.face;

import java.io.IOException;

public class BadinJawFaceDemo extends BadinJawTongueFaceDemo {

   public BadinJawFaceDemo() {
      super();
   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);
   }

   @Override
   public void addTongueToJaw(TongueType tt) {	
      // do not add tongue
   }

   @Override
   public void attachTongueToJaw() {
      // do not attach tongue
   }

   
   
}

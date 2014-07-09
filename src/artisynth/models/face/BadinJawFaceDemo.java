package artisynth.models.face;

public class BadinJawFaceDemo extends BadinJawTongueFaceDemo {

   public BadinJawFaceDemo() {
      super();
   }

   public BadinJawFaceDemo(String name) {
      super(name);
   }

   @Override
   public void addTongueToJaw() {	
      // do not add tongue
   }

   @Override
   public void attachTongueToJaw() {
      // do not attach tongue
   }

   
   
}

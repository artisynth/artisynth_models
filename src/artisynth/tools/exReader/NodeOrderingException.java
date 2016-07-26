package artisynth.tools.exReader;

public class NodeOrderingException extends Exception {

   private static final long serialVersionUID = 1L;
   public Object myObj = null;
   
   public NodeOrderingException(Object obj, String msg) {
      super(msg);
      myObj = obj;
   }
   public NodeOrderingException(Object obj) {
      super();
      myObj = obj;
   }
   
   public Object getObj() {
      return myObj;
   }
  

}

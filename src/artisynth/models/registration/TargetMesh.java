package artisynth.models.registration;

import artisynth.core.mechmodels.MeshComponent;
import maspack.geometry.MeshBase;
import maspack.matrix.AffineTransform3dBase;

/**
 * Special class for holding a mesh registration target.  Modifying the positions of the
 * underlying mesh vertices corresponds directly to changing the registration target vertex
 * locations.
 */
public class TargetMesh extends MeshComponent {

   public TargetMesh(String name) {
      this();
      setName(name);
   }

   public TargetMesh () {
      super();
   }
   
   public TargetMesh(MeshBase mesh) {
      this(mesh, null, null);
   }

   public TargetMesh (
      MeshBase mesh, String fileName, AffineTransform3dBase X) {
      this();
      setMesh (mesh, fileName, X);
   }
   
}

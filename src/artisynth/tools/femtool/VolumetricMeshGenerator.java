package artisynth.tools.femtool;

import maspack.geometry.BVFeatureQuery;
import maspack.geometry.BVTree;
import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.TriangleIntersector;
import maspack.matrix.Matrix3d;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.SymmetricMatrix3d;
import maspack.matrix.Vector3d;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;

public abstract class VolumetricMeshGenerator {

   public enum GeneratorType {
      HYBRID, SIMPLE
   }

   public abstract GeneratorType getType();
   public abstract FemModel3d generate(PolygonalMesh surface, int[] resolution);

   public static void projNodesToSurface(BVTree bvh,
      Vector3d ray, FemNode3d[] nodes) {
      
      TriangleIntersector isect = new TriangleIntersector();
      BVFeatureQuery query = new BVFeatureQuery();
      
      Vector3d duv = new Vector3d();
      Vector3d duv2 = new Vector3d();
      Face face, face2;
      Point3d ps;
      Vector3d dir = new Vector3d(ray);

      for (FemNode3d n : nodes) {
         
         face = query.nearestFaceAlongRay (
            null, duv, bvh, n.getPosition(), dir);
         dir.negate();

         face2 = query.nearestFaceAlongRay (
            null, duv2, bvh, n.getPosition(), dir);
         
         // find closest face
         if (face == null && face2 != null) {
            face = face2;
         } else if (face != null && face2 != null && Math.abs(duv2.x) < Math.abs(duv.x)) {
            face = face2;
         } else {
            dir.negate();  // return to first normal, use first face
         }
         
         if (face != null) {
            ps = projToSurface(n.getPosition(), dir, face);
            n.setPosition(ps);
         } else {
            System.out.println("Warning: in projNodesToSurface(), " +
               "cannot project to surface!");
         }
      }
   }

   public static Point3d projToSurface(Point3d origP, Vector3d origV,
      Face face_ray) {
      
      Point3d proj = new Point3d();
      Point3d surP = new Point3d();
      Point3d surP_origP = new Point3d();
      Vector3d normal = face_ray.getNormal();

      face_ray.computeCentroid(surP);
      surP_origP.sub(surP, origP);
      proj.set(origV);
      proj.scale(normal.dot(surP_origP) / normal.dot(origV));
      proj.add(origP);

      return proj;
   }

   public static Point3d projToAxis(Point3d origP, Point3d axisP, Vector3d axis) {
      Point3d proj = new Point3d(origP);
      proj.sub(axisP);
      proj.scale(proj.dot(axis), axis);
      proj.add(axisP);
      return proj;
   }
   
   public static Point3d[] getTightBox(PolygonalMesh mesh) {
      RigidTransform3d principle = getPrincipalAxes(mesh);
      return getTightBox(mesh, principle);
   }
   
   public static Point3d[] getTightBox(PolygonalMesh mesh, RigidTransform3d principle) {
      
      Vector3d [] axis = new Vector3d[3];
      double [][] projBounds = new double[3][2];
      
      for (int i=0; i<3; i++) {
         axis[i] = new Vector3d();
         principle.R.getColumn(i, axis[i]);   
      }
      Vector3d p = principle.p;

      // loop through all nodes, projecting onto axes
      for (int i=0; i<mesh.numVertices(); i++) {
         
         for (int j=0; j<3; j++) {
            
            Vector3d vec = new Vector3d(); 
            vec.sub(mesh.getVertex(i).getPosition(), p);
            
            double d =vec.dot(axis[j]);
            if (d < projBounds[j][0]) {
               projBounds[j][0] = d;
            } else if (d > projBounds[j][1]) {
               projBounds[j][1] = d;
            }
         } // end looping through axes
      } // end looping over vertices
      
      // construct bounds
      
      Point3d [] bounds = new Point3d[8];
      // coords: (1,1,1), (1,1,0), (1,0,0), (1,0,1), 
      // (0,1,1), (0,1,0), (0,0,0), (0,0,1)
      int [][] coords = {{1,1,1}, {1,1,0}, {1,0,0}, {1,0,1}, 
                           {0,1,1}, {0,1,0}, {0,0,0}, {0,0,1}};
      
      // create 8 corners
      for (int i=0; i<8; i++) {
         bounds[i] = new Point3d(p);
         for (int j=0; j<3; j++) {
            bounds[i].scaledAdd(projBounds[j][coords[i][j]], axis[j]);
         }
      }
      
      return bounds;
      
   }

   public static RigidTransform3d getPrincipalAxes(PolygonalMesh mesh) {

      Vector3d mov1 = new Vector3d();
      Vector3d mov2 = new Vector3d();
      Vector3d pov = new Vector3d();

      double vol = mesh.computeVolumeIntegrals(mov1, mov2, pov);
      double mass = vol;

      Point3d cov = new Point3d();
      cov.scale(1.0 / vol, mov1); // center of volume

      // [c], skew symmetric
      Matrix3d covMatrix = new Matrix3d(
         0, -cov.z, cov.y,
         cov.z, 0, -cov.x,
         -cov.y, cov.x, 0);
      // J
      Matrix3d J = new Matrix3d(
         (mov2.y + mov2.z), -pov.z, -pov.y,
         -pov.z, (mov2.x + mov2.z), -pov.x,
         -pov.y, -pov.x, (mov2.x + mov2.y));
      
      // Jc = J + m[c][c]
      Matrix3d Jc = new Matrix3d();
      Jc.mul(covMatrix, covMatrix);
      Jc.scale(mass);
      Jc.add(J);

      // Compute eigenvectors and eigenvlaues of Jc
      SymmetricMatrix3d JcSymmetric = new SymmetricMatrix3d(Jc);
      Vector3d lambda = new Vector3d();
      Matrix3d U = new Matrix3d();
      JcSymmetric.getEigenValues(lambda, U);

      // Construct the rotation matrix
      RotationMatrix3d R = new RotationMatrix3d();
      R.set(U);

      lambda.absolute();

      if (lambda.x > lambda.y && lambda.z > lambda.y) {
         R.rotateZDirection(new Vector3d(R.m01, R.m11, R.m21));
      } else if (lambda.x > lambda.z && lambda.y > lambda.z) {
         R.rotateZDirection(new Vector3d(R.m00, R.m10, R.m20));
      }

      return (new RigidTransform3d(cov, R));
   }

}

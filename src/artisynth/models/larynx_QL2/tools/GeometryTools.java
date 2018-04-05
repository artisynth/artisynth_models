package artisynth.models.larynx_QL2.tools;

import java.awt.Color;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;

import maspack.geometry.BVFeatureQuery;
import maspack.geometry.Face;
import maspack.geometry.MeshBase;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.geometry.BVFeatureQuery.InsideQuery;
import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.Renderer.LineStyle;
import maspack.render.Renderer.PointStyle;
import artisynth.core.driver.Main;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemMarker;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Particle;
import artisynth.core.mechmodels.PlanarConnector;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ModelComponent;
import artisynth.models.larynx_QL2.VocalTractBase;
import artisynth.models.larynx_QL2.components.AbstractBody;
import artisynth.models.larynx_QL2.components.HardBody;
import artisynth.models.larynx_QL2.components.SoftBody;
import artisynth.models.larynx_QL2.components.Structures;
import artisynth.models.larynx_QL2.tools.GeometryTools.Warper.WarpingPoint;
/**
 *  <p>
 *   @author Scott Moisik (Scott.Moisik@ntu.edu.sg), 2017, Nanyang Technological University, Singapore. 
 *   <p>
 *   If you use this model in your research, please cite: 
 *   Moisik, S. R., & Gick, B. (2017). The quantal larynx: the stable regions of laryngeal biomechanics and implications for speech production. Journal of Speech Language and Hearing Research, 60(3), 540-560. https://doi.org/10.1044/2016_JSLHR-S-16-0019

 *  **/
public class GeometryTools {

   public static class Warper {
      private ArrayList<WarpingTarget> warpingTargets =
         new ArrayList<WarpingTarget>();
      private ArrayList<WarpingPoint> warpingPoints =
         new ArrayList<WarpingPoint>();
      public HashMap<Particle,WarpingPoint> particleWarpPointMap =
         new HashMap<Particle,WarpingPoint>();
      protected boolean visibleFlag = true;

      Warper() {
      }

      Warper(boolean visibleFlag) {
         this.visibleFlag = visibleFlag;
      }

      public static void writeWarpingDataToCSV(
         String warpingPointFileName, Warper warper) {
         try {
            File warpingPointsFile = new File(warpingPointFileName);
            BufferedWriter bw =
               new BufferedWriter(new FileWriter(warpingPointsFile, false));

            for (WarpingPoint wp : warper.warpingPoints) {
               Point3d start = wp.startPoint.getPosition();
               Point3d stop = wp.stopPoint.getPosition();
               bw.write(
                  "warping_point, " + wp.name + ", pointSize, " + wp.pointSize
                     + ", maxRadius, " + wp.maxRadius + ", warpingCoefficient, "
                     + wp.warpingCoefficient + ", startPoint_xyz, " + start.x
                     + ", " + start.y + ", " + start.z + ", stopPoint_xyz, "
                     + stop.x + ", " + stop.y + ", " + stop.z + "\n");
            }

            for (WarpingTarget wt : warper.warpingTargets) {
               bw.write(
                  "target, " + ((ModelComponent)wt.targetObject).getName()
                     + "\n");
            }

            bw.close();
            System.out.println("Warping data saved to " + warpingPointFileName);
         }
         catch (IOException e) {
            System.err.println(
               "An error occurred writing to file " + warpingPointFileName
                  + "; if the file is open, close it and try again. If the directory does not exist, create it.");
         }
         catch (NullPointerException e) {
            System.err.println(
               "An unknown error occurred when attempting to save warping data.");
         }

      }

      public static Warper readWarpingDataFromCSV(
         String fileName, boolean visibleFlag) {
         ArrayList<String[]> rawTextLines = AuxTools.readDataFromCSV(fileName);
         Warper warper = new Warper(visibleFlag);

         // Read the point data
         for (String[] data : rawTextLines) {
            String type = data[0];
            String name = data[1];

            if (type.contains("warping_point")) {
               double pointSize = Double.valueOf(data[3].trim());
               double maxRadius = Double.valueOf(data[5].trim());
               double warpingCoefficient = Double.valueOf(data[7].trim());

               double startx = Double.valueOf(data[9].trim());
               double starty = Double.valueOf(data[10].trim());
               double startz = Double.valueOf(data[11].trim());
               Point3d start = new Point3d(startx, starty, startz);

               double stopx = Double.valueOf(data[13].trim());
               double stopy = Double.valueOf(data[14].trim());
               double stopz = Double.valueOf(data[15].trim());
               Point3d stop = new Point3d(stopx, stopy, stopz);

               warper.addWarpingPoint(
                  start, stop, name, pointSize, maxRadius, warpingCoefficient);
            }
            else if (type.contains("target")) {
               AbstractBody ab = Structures.getBodyByName(name);

               if (ab instanceof SoftBody) {
                  warper.addWarpingTarget(((SoftBody)ab).getFem());
               }
               else if (ab instanceof HardBody) {
                  warper.addWarpingTarget(((HardBody)ab).getBody());
               }
            }
         }

         return warper;
      }

      public static class WarpingTarget {
         public Object targetObject;
         public PolygonalMesh targetMesh;
         public HashMap<Object,Point3d> originalPoints =
            new HashMap<Object,Point3d>();

         public WarpingTarget(RigidBody rb) {
            targetObject = rb;
            targetMesh = rb.getMesh();
            for (Vertex3d vert : rb.getMesh().getVertices()) {
               Point3d p = vert.getPosition();
               originalPoints.put(vert, new Point3d(p.x, p.y, p.z));
            }
         }

         public WarpingTarget(FemMuscleModel fem) {
            targetObject = fem;
            for (FemNode3d node : fem.getNodes()) {
               Point3d p = node.getPosition();
               originalPoints.put(node, new Point3d(p.x, p.y, p.z));
            }
         }
      }

      public static class WarpingPoint {
         public Warper warper;
         public String name;
         public Particle particle, startPoint, stopPoint;
         public double pointSize;
         public double maxRadius;
         public double warpingCoefficient; // Like the gravitational constant, a
                                           // positive coefficient attracts; a
                                           // negative repels

         public WarpingPoint(Warper warper, Point3d location, String name,
            double pointSize, double maxRadius, double warpingCoefficient,
            boolean visibleFlag) {
            this.name = name;
            this.warper = warper;

            this.particle =
               createParticle(
                  VocalTractBase.mech, location, Color.green, name, maxRadius);
            particle.getRenderProps().setAlpha(0.15);
            this.pointSize = particle.getRenderProps().getPointRadius();
            this.startPoint =
               createParticle(
                  VocalTractBase.mech, particle.getPosition(), Color.blue,
                  name + " starting point", maxRadius * 0.1);
            this.stopPoint =
               createParticle(
                  VocalTractBase.mech, particle.getPosition(), Color.orange,
                  name + " stopping point", maxRadius * 0.1);

            this.maxRadius = maxRadius;
            this.warpingCoefficient = warpingCoefficient;

            this.particle.getRenderProps().setVisible(visibleFlag);
            this.startPoint.getRenderProps().setVisible(visibleFlag);
            this.stopPoint.getRenderProps().setVisible(visibleFlag);
         }

         public WarpingPoint(Warper warper, Point3d start, Point3d stop,
            String name, double pointSize, double maxRadius,
            double warpingCoefficient, boolean visibleFlag) {
            this(
               warper, start, name, pointSize, maxRadius, warpingCoefficient,
               visibleFlag);
            this.stopPoint.setPosition(stop);
         }

         /**
          * Calculates the displacement force exerted by this warping point
          * (based on Netownian gravity)
          **/
         public Point3d calculateDisplacement(Point3d inputPoint) {
            Point3d displacement = new Point3d(particle.getPosition());
            displacement.sub(inputPoint);

            double dist = displacement.norm();
            dist = (maxRadius - dist) / maxRadius;

            if (dist > 0.0) {

               displacement.normalize();
               displacement.scale(warpingCoefficient * dist);
               return displacement;
            }
            else {
               return new Point3d();
            }
         }

         public void update() {
            warper.applyWarping();
         }
      }

      public WarpingPoint addWarpingPoint(
         Point3d location, String name, double pointSize, double maxRadius,
         double warpingCoefficient) {
         WarpingPoint wp =
            new WarpingPoint(
               this, location, name, pointSize, maxRadius, warpingCoefficient,
               visibleFlag);
         warpingPoints.add(wp);
         particleWarpPointMap.put(wp.particle, wp);
         return wp;
      }

      public WarpingPoint addWarpingPoint(
         Point3d start, Point3d stop, String name, double pointSize,
         double maxRadius, double warpingCoefficient) {
         WarpingPoint wp =
            new WarpingPoint(
               this, start, stop, name, pointSize, maxRadius,
               warpingCoefficient, visibleFlag);
         warpingPoints.add(wp);
         particleWarpPointMap.put(wp.particle, wp);
         return wp;
      }

      public boolean isTarget(Object obj) {
         for (WarpingTarget wt : warpingTargets) {
            if (wt.targetObject.equals(obj)) {
               return true;
            }
         }
         return false;
      }

      public void addWarpingTarget(RigidBody rb) {
         warpingTargets.add(new WarpingTarget(rb));
      }

      public void addWarpingTarget(FemMuscleModel fem) {
         warpingTargets.add(new WarpingTarget(fem));
      }

      public void applyWarping() {
         for (WarpingTarget wt : warpingTargets) {
            for (Object key : wt.originalPoints.keySet()) {

               Point3d warpingVector = new Point3d();
               for (WarpingPoint wp : warpingPoints) {
                  Point3d inputPoint = wt.originalPoints.get(key);
                  if (inputPoint != null) {
                     warpingVector.add(wp.calculateDisplacement(inputPoint));
                  }
                  else {
                     System.err.println(
                        "Warper error: An input point could not be located.");
                  }
               }

               Point3d newPosition = new Point3d(wt.originalPoints.get(key));
               newPosition.add(warpingVector);

               if (key instanceof Vertex3d) {
                  ((Vertex3d)key).setPosition(newPosition);
               }
               else if (key instanceof FemNode3d) {
                  ((FemNode3d)key).setPosition(newPosition);
               }
            }

            // RigidBody meshes need to be reset for changes to take effect
            if (wt.targetObject instanceof RigidBody) {
               ((RigidBody)wt.targetObject)
                  .setMesh(new PolygonalMesh(wt.targetMesh));
            }
         }
      }

      public void resetWarping() {
         for (WarpingTarget wt : warpingTargets) {
            for (Object obj : wt.originalPoints.entrySet()) {
               if (obj instanceof Vertex3d) {
                  ((Vertex3d)obj).setPosition(wt.originalPoints.get(obj));
               }
               else if (obj instanceof FemNode3d) {
                  ((FemNode3d)obj).setPosition(wt.originalPoints.get(obj));
               }
            }
         }
      }

      public int getNumberOfPoints() {
         return warpingPoints.size();
      }

      public WarpingPoint getPointByName(String name) {
         for (WarpingPoint wp : warpingPoints) {
            if (wp.name.contains(name)) {
               return wp;
            }
         }
         System.err.println(
            "Could not locate warping point '" + name
               + "'. Available warping points are:");
         for (WarpingPoint wp : warpingPoints) {
            System.err.println(wp.name);
         }
         return null;
      }

   }

   public static Point3d pointOnParametricLine(
      double t, Point3d start, Point3d stop) {
      Point3d diff = new Point3d(stop);
      diff.sub(start);
      diff.scale(t);
      Point3d p = new Point3d(start);
      p.add(diff);
      return p;
   }

   public static Point3d weightedAveragePosition(
      double[] weights, Point3d[] points) {
      if (points.length > 0 && weights.length == points.length) {
         double weightSum = 0.0;
         Point3d avgPos = new Point3d();

         for (int i = 0; i < points.length; i++) {
            avgPos.scaledAdd(weights[i], points[i]);
            weightSum += weights[i];
         }

         if (weightSum > 0.0) {
            avgPos.scale(1 / weightSum);
            return avgPos;
         }
         else {
            throw new IllegalArgumentException(
               "Weight sum must be greater than zero.");
         }
      }
      else {
         throw new IllegalArgumentException(
            "The number of weights must be the same as the number of points.");
      }
   }

   public static Point3d averagePosition(Point3d... points) {
      if (points.length > 0) {
         double numPoints = points.length;
         Point3d avgPos = new Point3d(points[0]);

         for (int i = 1; i < points.length; i++) {
            avgPos.add(points[i]);
         }

         avgPos.scale(1 / numPoints);
         return avgPos;
      }
      else {
         return new Point3d();
      }
   }

   public static Vector3d averageVector(Vector3d... vectors) {
      if (vectors.length > 0) {
         double count = 0.0;
         Vector3d avgVector = new Vector3d();

         for (int i = 0; i < vectors.length; i++) {
            if (vectors[i] != null) {
               avgVector.add(vectors[i]);
               count += 1.0;
            }
         }

         if (count > 0.0) {
            avgVector.scale(1 / count);
            return avgVector;
         }
         else {
            throw new IllegalArgumentException(
               "All vectors provided were null.");
         }
      }
      else {
         return new Vector3d();
      }
   }

   public static Point3d findCharacteristicCenter(ModelComponent mc) {
      if (mc instanceof RigidBody) {
         Point3d centroid = new Point3d();
         ((RigidBody)mc).getMesh().computeCentroid(centroid);
         return centroid;
      }
      else if (mc instanceof Point) {
         return ((Point)mc).getPosition();
      }
      else if (mc instanceof FemElement3d) {
         Point3d centroid = new Point3d();
         ((FemElement3d)mc).computeCentroid(centroid);
         return centroid;
      }
      else if (mc instanceof FemMuscleModel) {
         Point3d centroid = new Point3d();
         ((FemMuscleModel)mc).getSurfaceMesh().computeCentroid(centroid);
         return centroid;
      }

      if (mc != null) {
         System.err
            .println("No center defined for model component " + mc.getName());
      }

      return new Point3d();
   }

   public static Point3d negate(Point3d point) {
      return new Point3d(-point.x, -point.y, -point.z);
   }

   public static Vector3d negate(Vector3d vector) {
      return new Vector3d(-vector.x, -vector.y, -vector.z);
   }

   public static Point3d offsetX(Point3d point, double offset) {
      return new Point3d(point.x + offset, point.y, point.z);
   }

   public static Point3d offsetY(Point3d point, double offset) {
      return new Point3d(point.x, point.y + offset, point.z);
   }

   public static Point3d offsetZ(Point3d point, double offset) {
      return new Point3d(point.x, point.y, point.z + offset);
   }

   public static Point3d rotationalOffsetZY(
      Point3d point, double offset, double angle) {
      Point3d zyVector = new Point3d(0.0, 0.0, offset);
      RotationMatrix3d rotation =
         new RotationMatrix3d(new AxisAngle(new Point3d(1.0, 0.0, 0.0), angle));
      zyVector.transform(rotation);
      zyVector.add(point);
      return zyVector;
   }

   public enum OffsetRotationZY {
      ANTERIOR (Math.PI),
      ANTERO_INFERIOR (Math.PI * 0.75),
      ANTERO_SUPERIOR (-Math.PI * 0.75),
      POSTERIOR (Math.PI),
      POSTERO_INFERIOR (Math.PI * 0.25),
      POSTERO_SUPERIOR (-Math.PI * 0.25),
      INFERIOR (Math.PI * 0.5),
      SUPERIOR (-Math.PI * 0.5);
      public double angle;

      OffsetRotationZY(double angle) {
         this.angle = angle;
      }

      public double getAngle() {
         return angle;
      }
   }

   public static Point3d rotationalOffsetZY(
      Point3d point, double offset, OffsetRotationZY cp) {
      return rotationalOffsetZY(point, offset, cp.getAngle());
   }

   public static FrameMarker createMarker(
      MechModel mech, RigidBody rigidBody, Point3d point, Color color,
      double radius, String pointName) {
      FrameMarker fm = new FrameMarker(rigidBody, point);
      mech.addFrameMarker(fm);
      fm.setName(pointName);
      RenderProps.setVisible(fm, true);
      RenderProps.setPointColor(fm, color);
      RenderProps.setPointRadius(fm, radius);
      return fm;
   }

   public static FemMarker createMarker(
      MechModel mech, FemMuscleModel fem, Point3d point, Color color,
      double radius, String pointName) {
      FemMarker fm = new FemMarker(point);
      fem.addMarker(fm);
      fm.setName(pointName);
      RenderProps.setVisible(fm, true);
      RenderProps.setPointColor(fm, color);
      RenderProps.setPointRadius(fm, radius);
      return fm;
   }

   public static PlanarConnector createConstraintPlane(
      MechModel mech, String name, RigidBody body, Point3d location,
      Point3d axis, double angle, double size, boolean visibleFlag) {
      PlanarConnector plane =
         new PlanarConnector(
            body, location,
            new RigidTransform3d(location, new AxisAngle(axis, angle)));

      if (visibleFlag) {
         plane.setName(name);
         plane.setPlaneSize(size);
         RenderProps.setFaceStyle(plane, Renderer.FaceStyle.FRONT_AND_BACK);
         RenderProps.setAlpha(plane, 0.8);
      }

      mech.addBodyConnector(plane);
      return plane;
   }

   public static RigidBody createRigidBody(MechModel mech, String rigidBodyName, PolygonalMesh mesh, boolean dynamicFlag, double alpha) {
	      PolygonalMesh newMesh = new PolygonalMesh(mesh);
	      RigidBody rb = new RigidBody(rigidBodyName);
	      rb.setMesh(newMesh);
	      rb.setDynamic(dynamicFlag);
	      RenderProps.setAlpha(rb, alpha);
	      mech.addRigidBody(rb);
	      return rb;
	   }
   
   public static RigidBody createRigidBody(
      MechModel mech, String rigidBodyName, String fileName,
      boolean dynamicFlag, double alpha) {
      PolygonalMesh mesh = AuxTools.loadPolygonalMesh(fileName);
      RigidBody rb = new RigidBody(rigidBodyName);
      rb.setMesh(mesh, fileName);
      rb.setDynamic(dynamicFlag);
      RenderProps.setAlpha(rb, alpha);
      mech.addRigidBody(rb);
      return rb;
   }

   public static PolygonalMesh createPlaneMesh(
      double size, RigidTransform3d transform) {
      Point3d[] points = new Point3d[4];
      double hSize = size * 0.5;
      points[0] = new Point3d(hSize, hSize, 0);
      points[1] = new Point3d(-hSize, hSize, 0);
      points[2] = new Point3d(-hSize, -hSize, 0);
      points[3] = new Point3d(hSize, -hSize, 0);
      for (int i = 0; i < points.length; i++) {
         points[i].transform(transform);
      }

      PolygonalMesh mesh = new PolygonalMesh();
      int[][] faceIndices = new int[][] { { 0, 1, 2, 3 } };
      mesh.set(points, faceIndices);
      mesh.triangulate();
      return mesh;
   }

   public static Particle createParticle(
      MechModel mech, Point3d pos, Color color, boolean dynamicFlag,
      double mass, double radius) {
      Particle p = new Particle(mass, pos);
      p.setDynamic(dynamicFlag);
      mech.addParticle(p);
      RenderProps.setPointStyle(p, PointStyle.SPHERE);
      RenderProps.setPointColor(p, color);
      RenderProps.setPointRadius(p, radius);
      RenderProps.setVisible(p, true);

      return p;
   }

   public static Particle createParticle(
      MechModel mech, Point3d pos, Color color, String name, double radius) {
      Particle p = new Particle(1.0, pos);
      p.setDynamic(false);
      p.setName(name);
      mech.addParticle(p);

      RenderProps.setPointStyle(p, PointStyle.SPHERE);
      RenderProps.setPointColor(p, color);
      RenderProps.setPointRadius(p, radius);
      RenderProps.setVisible(p, true);

      return p;
   }

   public static AxialSpring createSpring(
      MechModel mech, Point p1, Point p2, double radius) {
      AxialSpring as = new AxialSpring();
      as.setFirstPoint(p1);
      as.setSecondPoint(p2);
      as.setRestLengthFromPoints();
      RenderProps.setLineColor(as, Color.white);
      RenderProps.setLineRadius(as, radius);
      RenderProps.setLineStyle(as, LineStyle.CYLINDER);
      RenderProps.setVisible(as, true);
      mech.addAxialSpring(as);
      return as;
   }

   public static AxialSpring createSpring(
      MechModel mech, Point p1, Point p2, double radius, Color color) {
      AxialSpring as = new AxialSpring();
      as.setFirstPoint(p1);
      as.setSecondPoint(p2);
      as.setRestLengthFromPoints();
      RenderProps.setLineColor(as, color);
      RenderProps.setLineRadius(as, radius);
      RenderProps.setLineStyle(as, LineStyle.CYLINDER);
      RenderProps.setVisible(as, true);
      mech.addAxialSpring(as);
      return as;
   }

   public static AxialSpring createSpring(
      MechModel mech, Point3d pos1, Point3d pos2, double mass,
      double pointRadius, double lineRadius, Color color) {
      Point p1 =
         createParticle(mech, pos1, Color.green, true, mass, pointRadius);
      Point p2 =
         createParticle(mech, pos2, Color.green, true, mass, pointRadius);

      return createSpring(mech, p1, p2, lineRadius, color);
   }

   public static AxialSpring createSpring(
      MechModel mech, Point3d pos1, Point3d pos2, double mass,
      double pointRadius, double lineRadius) {
      Point p1 =
         createParticle(mech, pos1, Color.green, true, mass, pointRadius);
      Point p2 =
         createParticle(mech, pos2, Color.green, true, mass, pointRadius);

      return createSpring(mech, p1, p2, lineRadius);
   }

   public static AxialSpring createSpring(
      MechModel mech, Point3d pos1, Point3d pos2, double mass, double stiffness,
      double damping, double pointRadius, double lineRadius) {
      Point p1 =
         createParticle(mech, pos1, Color.green, true, mass, pointRadius);
      Point p2 =
         createParticle(mech, pos2, Color.green, true, mass, pointRadius);

      AxialSpring as = createSpring(mech, p1, p2, lineRadius);
      AxialSpring.setStiffness(as, stiffness);
      AxialSpring.setDamping(as, damping);
      return as;
   }

   public static AxialSpring createSpring(
      MechModel mech, Point p1, Point p2, double stiffness, double damping,
      double radius) {
      AxialSpring as = createSpring(mech, p1, p2, radius);
      AxialSpring.setStiffness(as, stiffness);
      AxialSpring.setDamping(as, damping);
      return as;
   }

   public static NearestFaceProps createNormalRepresentation(
      VocalTractBase vtBase, PolygonalMesh mesh, Point3d pos, Color color,
      ArrayList<Particle> particles, ArrayList<AxialSpring> springs) {
      Particle p1 =
         createParticle(VocalTractBase.mech, pos, color, false, 1.0, 0.3);
      particles.add(p1);

      NearestFaceProps nfp = new NearestFaceProps(mesh, pos);
      Point3d tipPos = new Point3d(pos);
      tipPos.add(nfp.faceNormal);
      Particle p2 =
         createParticle(VocalTractBase.mech, tipPos, color, false, 1.0, 0.2);
      particles.add(p2);

      AxialSpring as = createSpring(VocalTractBase.mech, p1, p2, 2.0);
      as.getRenderProps().setLineColor(color);
      springs.add(as);
      return nfp;
   }

   /**
    * Creates a normal vector lying along the line from pos1 to pos2 originating
    * at pos1 and pointing towards pos2
    **/
   public static Vector3d createProjectionRepresentation(
      VocalTractBase vtBase, PolygonalMesh mesh, Point3d pos1, Point3d pos2,
      Color color, ArrayList<Particle> particles,
      ArrayList<AxialSpring> springs) {
      Point3d projPosNormal = new Point3d(pos2);
      projPosNormal.sub(pos1);
      projPosNormal.normalize();

      Particle p1 =
         createParticle(VocalTractBase.mech, pos1, color, false, 1.0, 0.3);
      particles.add(p1);

      Point3d projTip = new Point3d(pos1);
      projTip.add(projPosNormal);

      Particle p2 =
         createParticle(VocalTractBase.mech, projTip, color, false, 1.0, 0.2);
      particles.add(p2);

      AxialSpring as = createSpring(VocalTractBase.mech, p1, p2, 0.1);
      as.getRenderProps().setLineColor(color);
      springs.add(as);

      return projPosNormal;
   }

   /**
    * Checks if a point is contained within a mesh and returns the distance to
    * the nearest face. If this number is zero, the point is on the surface; if
    * it is negative, the point is contained within the mesh, but located at the
    * distance specified away from the nearest face; finally, if it is positive,
    * the point is external and located at the given distance away from the
    * nearest face.
    **/
   public static double checkContainmentExtent(
      PolygonalMesh mesh, Point3d pos) {
      InsideQuery iq = BVFeatureQuery.isInsideMesh(mesh, pos);
      Point3d proj = new Point3d();
      BVFeatureQuery.getNearestFaceToPoint(proj, null, mesh, pos);

      Point3d diff = new Point3d(pos);
      diff.sub(proj);
      double distance = diff.norm();
      return (iq == InsideQuery.INSIDE ? -1.0 * distance : distance);
   }

   public static Vector3d createPerpendicularVector(Point3d point3d) {
      return createPerpendicularVector(new Vector3d(point3d));
   }

   public static Vector3d createPerpendicularVector(Vector3d vector3d) {
      Vector3d c = new Point3d();
      if (vector3d.y != 0.0 || vector3d.z != 0.0) {
         c.set(1.0, 0.0, 0.0);
      }
      else {
         c.set(0.0, 1.0, 0.0);
      }

      Vector3d perp = new Vector3d(vector3d);
      perp.cross(c);

      return perp;
   }

   public static Vector3d getDifferenceVector(Point3d p1, Point3d p2) {
      Vector3d dir = new Vector3d(p1);
      dir.sub(p2);
      return dir;
   }

   public static class NearestFaceProps {
      public Point3d position;
      public Point3d projection = new Point3d();
      public Vector3d faceNormal = new Vector3d();
      public Face face;
      public InsideQuery iq;
      public Point3d faceCentroid;
      public Point3d projPosNormal;
      public double projPosDistance;
      public FemNode3d node;
      public boolean invertedNormalFlag = false;

      public NearestFaceProps(PolygonalMesh mesh, FemNode3d node) {
         this(mesh, node.getPosition());
         this.node = node;
      }

      public NearestFaceProps(PolygonalMesh mesh, Point3d pos) {
         position = pos;
         face = BVFeatureQuery.getNearestFaceToPoint(projection, null, mesh, position);
         iq = BVFeatureQuery.isInsideMesh(mesh, position);
         face.computeNormal(faceNormal);
         faceCentroid = computeCentroid(face);

         projPosNormal = new Point3d(position);
         projPosNormal.sub(projection);
         projPosDistance = projPosNormal.norm();
         projPosNormal.normalize();
      }

      public static Point3d computeCentroid(Face face) {
         Vector3d normal = new Vector3d();
         face.computeNormal(normal);

         Point3d centroid = new Point3d();
         for (Vertex3d v : face.getVertices()) {
            centroid.add(v.getPosition());
         }

         return centroid;
      }
   }

}

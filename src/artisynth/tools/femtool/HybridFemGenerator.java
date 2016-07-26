package artisynth.tools.femtool;

import java.util.Hashtable;

import maspack.geometry.BVFeatureQuery;
import maspack.geometry.BVTree;
import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.TriangleIntersector;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;
import maspack.properties.HasProperties;
import maspack.properties.Property;
import maspack.properties.PropertyList;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.HexElement;
import artisynth.core.femmodels.TetElement;
import artisynth.core.femmodels.WedgeElement;
import artisynth.core.mechmodels.PointList;

public class HybridFemGenerator extends VolumetricMeshGenerator implements HasProperties {

   public enum TemplateType {
      BEAM, CYLINDER, SPINDLE
   }
   
   public static double DEFAULT_ACCURACY = 0.05;
   public static double DEFAULT_BORDER_PERCENTAGE = 0.01;
   public static boolean DEFAULT_PROJECT_END_NODES = true;
   public static TemplateType DEFAULT_TEMPLATE = TemplateType.BEAM;

   private double accuracyPercentage = DEFAULT_ACCURACY;
   private double borderPercentage = DEFAULT_BORDER_PERCENTAGE;
   private boolean projEndNodes = DEFAULT_PROJECT_END_NODES;
   private TemplateType templateType = DEFAULT_TEMPLATE;
   
   public static PropertyList myProps =
      new PropertyList(HybridFemGenerator.class);

   static {
      myProps.add("accuracyPercentage * *", "", DEFAULT_ACCURACY);
      myProps.add("borderPercentage * *", "", DEFAULT_BORDER_PERCENTAGE);
      myProps.add("projEndNodes * *", "", DEFAULT_PROJECT_END_NODES);
      myProps.add("templateType * *", "", DEFAULT_TEMPLATE);
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   public Property getProperty(String path) {
      return PropertyList.getProperty(path, this);
   }
   
   public HybridFemGenerator() {
   }
   
   public HybridFemGenerator(TemplateType type) {
      setTemplateType(type);
   }

   public FemModel3d generate(PolygonalMesh surface, int [] res) {
      
      // align to major axes first, generate model
      RigidTransform3d transform = getPrincipalAxes(surface);
      surface.inverseTransform(transform);
      
      FemModel3d model = null;
      switch(templateType) {
         case BEAM:
            model = createBeamWithSurface(surface, res[0], res[1], res[2]);
            break;
         case CYLINDER:
            model = createCylinderWithSurface(surface, res[0], res[1], res[2]);
            break;
         case SPINDLE:
            if (res.length > 3) {
               model = createEllipsoidWithSurface(surface, res[0], res[1], res[2], res[3]);
            } else {
               model = createEllipsoidWithSurface(surface, res[0], res[1], res[2]);
            }
            break;
      }
      
      // transform back to original orientation
      surface.transform(transform);
      if (model != null) {
         model.transformGeometry(transform);
      }
      
      return model;
      
   }
   
   public static FemModel3d createBeamWithSurface(PolygonalMesh mesh, int nx,
      int ny, int nz) {
      RigidTransform3d transform = getPrincipalAxes(mesh);
      transform.R.rotateZDirection(new Vector3d(
         transform.R.m00, transform.R.m10, transform.R.m20));
      return createBeamWithSurface(mesh, transform, nx, ny, nz);
   }

   public static FemModel3d createBeamWithSurface(PolygonalMesh mesh,
      RigidTransform3d alignment, int nx, int ny, int nz) {
      return createBeamWithSurface(
         mesh, alignment, nx, ny, nz, DEFAULT_BORDER_PERCENTAGE);
   }

   public static FemModel3d createBeamWithSurface(PolygonalMesh mesh,
      RigidTransform3d alignment, int nx, int ny, int nz,
      double borderPercentage) {
      BVTree bvh = mesh.getBVTree();
      Point3d[] edgePoints = getTightBox(mesh, alignment);  // corners of a
                                                             // bounding box

      Vector3d principalAxis_x = new Vector3d();
      Vector3d principalAxis_y = new Vector3d();
      Vector3d principalAxis_z = new Vector3d();
      alignment.R.getColumn(0, principalAxis_x);
      alignment.R.getColumn(1, principalAxis_y);
      alignment.R.getColumn(2, principalAxis_z);

      double xlimit = edgePoints[0].distance(edgePoints[4]);
      double ylimit = edgePoints[1].distance(edgePoints[2]);
      double zlimit = edgePoints[0].distance(edgePoints[1]);
      double endSide = zlimit * borderPercentage;
      double zStep = (zlimit - 2 * endSide) / nz;
      Point3d edge;
      Point3d p = new Point3d();
      Vector3d rayDir = new Vector3d();
      BVFeatureQuery query = new BVFeatureQuery();
      Vector3d duv = new Vector3d();
      Face face;

      // Create template mesh
      FemModel3d fem = new FemMuscleModel();
      FemFactory.createHexGrid(fem, xlimit, ylimit, zlimit, nx, ny, nz);
      
      
      PointList<FemNode3d> nodes = fem.getNodes();

      // Loop to generate to top edge nodes
      p.set(edgePoints[6]);
      p.scaledAdd(endSide, principalAxis_z);
      rayDir.set(principalAxis_x);
      for (int i = 0; i <= nz; i++) {
         if (i == 0 || i == nz)
            edge =
               getEdgePoint(
                  bvh, p, rayDir, principalAxis_y, ylimit, 1, xlimit / 3, true);
         else
            edge =
               getEdgePoint(
                  bvh, p, rayDir, principalAxis_y, ylimit, 1, xlimit / 3,
                  false);
         // drawVector(p,p);
         if (edge != null)
            nodes.get(i * (nx + 1) * (ny + 1)).setPosition(edge);
         p.scaledAdd(zStep, principalAxis_z);
      }
      p.set(edgePoints[5]);
      p.scaledAdd(endSide, principalAxis_z);
      rayDir.set(principalAxis_x);
      for (int i = 0; i <= nz; i++) {
         if (i == 0 || i == nz)
            edge =
               getEdgePoint(
                  bvh, p, rayDir, principalAxis_y, ylimit, -1, xlimit / 3,
                  true);
         else
            edge =
               getEdgePoint(
                  bvh, p, rayDir, principalAxis_y, ylimit, -1, xlimit / 3,
                  false);
         // drawVector(p,p);
         if (edge != null)
            nodes
               .get(i * (nx + 1) * (ny + 1) + ny * (nx + 1)).setPosition(edge);
         p.scaledAdd(zStep, principalAxis_z);
      }

      // Loop to generate to bottom edge nodes
      p.set(edgePoints[2]);
      p.scaledAdd(endSide, principalAxis_z);
      rayDir.set(principalAxis_x);
      rayDir.scale(-1);
      for (int i = 0; i <= nz; i++) {
         if (i == 0 || i == nz)
            edge =
               getEdgePoint(
                  bvh, p, rayDir, principalAxis_y, ylimit, 1, xlimit / 3, true);
         else
            edge =
               getEdgePoint(
                  bvh, p, rayDir, principalAxis_y, ylimit, 1, xlimit / 3,
                  false);
         // drawVector(p,p);
         if (edge != null)
            nodes.get(i * (nx + 1) * (ny + 1) + nx).setPosition(edge);
         p.scaledAdd(zStep, principalAxis_z);
      }
      p.set(edgePoints[1]);
      p.scaledAdd(endSide, principalAxis_z);
      rayDir.set(principalAxis_x);
      rayDir.scale(-1);
      for (int i = 0; i <= nz; i++) {
         if (i == 0 || i == nz)
            edge =
               getEdgePoint(
                  bvh, p, rayDir, principalAxis_y, ylimit, -1, xlimit / 3,
                  true);
         else
            edge =
               getEdgePoint(
                  bvh, p, rayDir, principalAxis_y, ylimit, -1, xlimit / 3,
                  false);
         // drawVector(p,p);
         if (edge != null)
            nodes
               .get(i * (nx + 1) * (ny + 1) + (ny + 1) * (nx + 1) - 1)
               .setPosition(edge);
         p.scaledAdd(zStep, principalAxis_z);
      }

      Point3d pe1 = new Point3d();
      Point3d pe2 = new Point3d();
      Point3d pe3 = new Point3d();
      Point3d pe4 = new Point3d();
      Point3d ps = new Point3d();
      // Generate the top middle nodes
      p.set(edgePoints[6]);
      p.scaledAdd(endSide, principalAxis_z);
      rayDir.set(principalAxis_x);
      for (int i = 0; i <= nz; i++) {
         pe1 = nodes.get(i * (nx + 1) * (ny + 1)).getPosition();
         pe2 = nodes.get(i * (nx + 1) * (ny + 1) + ny * (nx + 1)).getPosition();
         pe1 = projToAxis(pe1, p, principalAxis_y);
         pe2 = projToAxis(pe2, p, principalAxis_y);
         pe3.sub(pe2, pe1);
         pe3.normalize();
         pe3.scale(pe1.distance(pe2) / ny);
         pe4.set(pe1);
         for (int j = 0; j < ny - 1; j++) {
            pe4.add(pe3);
            //face = bvh.intersect(pe4, rayDir, duv, isect);
            face = query.nearestFaceAlongRay (
               null, duv, bvh, pe4, rayDir);
            if (face != null) {
               ps = projToSurface(pe4, rayDir, face);
               nodes
                  .get(i * (nx + 1) * (ny + 1) + (j + 1) * (nx + 1))
                  .setPosition(ps);
            }
            // drawVector(ps,ps);
         }
         p.scaledAdd(zStep, principalAxis_z);
      }
      // Generate the bottom middle nodes
      p.set(edgePoints[2]);
      p.scaledAdd(endSide, principalAxis_z);
      rayDir.set(principalAxis_x);
      rayDir.scale(-1);
      for (int i = 0; i <= nz; i++) {
         pe1 = nodes.get(i * (nx + 1) * (ny + 1) + nx).getPosition();
         pe2 =
            nodes
               .get(i * (nx + 1) * (ny + 1) + (ny + 1) * (nx + 1) - 1)
               .getPosition();
         pe1 = projToAxis(pe1, p, principalAxis_y);
         pe2 = projToAxis(pe2, p, principalAxis_y);
         pe3.sub(pe2, pe1);
         pe3.normalize();
         pe3.scale(pe1.distance(pe2) / ny);
         pe4.set(pe1);
         for (int j = 0; j < ny - 1; j++) {
            pe4.add(pe3);
            //face = bvh.intersect(pe4, rayDir, duv, isect);
            face = query.nearestFaceAlongRay (
               null, duv, bvh, pe4, rayDir);            
            if (face != null) {
               ps = projToSurface(pe4, rayDir, face);
               nodes
                  .get(i * (nx + 1) * (ny + 1) + nx + (j + 1) * (nx + 1))
                  .setPosition(ps);
            }
         }
         p.scaledAdd(zStep, principalAxis_z);
      }

      // Generate the side 1 middle nodes
      p.set(edgePoints[6]);
      p.scaledAdd(endSide, principalAxis_z);
      rayDir.set(principalAxis_y);
      for (int i = 0; i <= nz; i++) {
         pe1 = nodes.get(i * (nx + 1) * (ny + 1)).getPosition();
         pe2 = nodes.get(i * (nx + 1) * (ny + 1) + nx).getPosition();
         pe1 = projToAxis(pe1, p, principalAxis_x);
         pe2 = projToAxis(pe2, p, principalAxis_x);
         pe3.sub(pe2, pe1);
         pe3.normalize();
         pe3.scale(pe1.distance(pe2) / nx);
         pe4.set(pe1);
         for (int j = 0; j < nx - 1; j++) {
            pe4.add(pe3);
            //face = bvh.intersect(pe4, rayDir, duv, isect);
            face = query.nearestFaceAlongRay (
               null, duv, bvh, pe4, rayDir);
            if (face != null) {
               ps = projToSurface(pe4, rayDir, face);
               nodes.get(i * (nx + 1) * (ny + 1) + (j + 1)).setPosition(ps);
            }
         }
         p.scaledAdd(zStep, principalAxis_z);
      }
      // Generate the side 2 middle nodes
      p.set(edgePoints[5]);
      p.scaledAdd(endSide, principalAxis_z);
      rayDir.set(principalAxis_y);
      rayDir.scale(-1);
      for (int i = 0; i <= nz; i++) {
         pe1 = nodes.get(i * (nx + 1) * (ny + 1) + ny * (nx + 1)).getPosition();
         pe2 =
            nodes
               .get(i * (nx + 1) * (ny + 1) + (ny + 1) * (nx + 1) - 1)
               .getPosition();
         pe1 = projToAxis(pe1, p, principalAxis_x);
         pe2 = projToAxis(pe2, p, principalAxis_x);
         pe3.sub(pe2, pe1);
         pe3.normalize();
         pe3.scale(pe1.distance(pe2) / nx);
         pe4.set(pe1);
         for (int j = 0; j < nx - 1; j++) {
            pe4.add(pe3);
            //face = bvh.intersect(pe4, rayDir, duv, isect);
            face = query.nearestFaceAlongRay (
               null, duv, bvh, pe4, rayDir);
            if (face != null) {
               ps = projToSurface(pe4, rayDir, face);
               nodes
                  .get(i * (nx + 1) * (ny + 1) + ny * (nx + 1) + (j + 1))
                  .setPosition(ps);
            }
         }
         p.scaledAdd(zStep, principalAxis_z);
      }

      // Generate the side 3 middle nodes
      Point3d p1 = new Point3d();
      Point3d p2 = new Point3d();
      rayDir.set(principalAxis_z);
      for (int i = 0; i <= ny; i++) {
         pe1 = nodes.get(i * (nx + 1)).getPosition();
         pe2 = nodes.get(i * (nx + 1) + nx).getPosition();
         p1.scaledAdd(-zlimit, principalAxis_z, pe1);
         p2.scaledAdd(-zlimit, principalAxis_z, pe2);
         pe1 = projToAxis(pe1, p1, principalAxis_y);
         pe2 = projToAxis(pe2, p2, principalAxis_y);
         pe3.sub(pe2, pe1);
         pe3.normalize();
         pe3.scale(pe1.distance(pe2) / nx);
         pe4.set(pe1);
         for (int j = 0; j < nx - 1; j++) {
            pe4.add(pe3);
            //face = bvh.intersect(pe4, rayDir, duv, isect);
            face = query.nearestFaceAlongRay (
               null, duv, bvh, pe4, rayDir);
            if (face != null) {
               ps = projToSurface(pe4, rayDir, face);
               nodes.get(i * (nx + 1) + (j + 1)).setPosition(ps);
            }
         }
      }
      // Generate the side 4 middle nodes
      rayDir.set(principalAxis_z);
      rayDir.scale(-1);
      for (int i = 0; i <= ny; i++) {
         pe1 = nodes.get(i * (nx + 1) + (nx + 1) * (ny + 1) * nz).getPosition();
         pe2 =
            nodes
               .get(i * (nx + 1) + (nx + 1) * (ny + 1) * nz + nx).getPosition();
         p1.scaledAdd(zlimit, principalAxis_z, pe1);
         p2.scaledAdd(zlimit, principalAxis_z, pe2);
         pe1 = projToAxis(pe1, p1, principalAxis_y);
         pe2 = projToAxis(pe2, p2, principalAxis_y);
         pe3.sub(pe2, pe1);
         pe3.normalize();
         pe3.scale(pe1.distance(pe2) / nx);
         pe4.set(pe1);
         for (int j = 0; j < nx - 1; j++) {
            pe4.add(pe3);
            //face = bvh.intersect(pe4, rayDir, duv, isect);
            face = query.nearestFaceAlongRay (
               null, duv, bvh, pe4, rayDir);
            if (face != null) {
               ps = projToSurface(pe4, rayDir, face);
               nodes
                  .get(i * (nx + 1) + (nx + 1) * (ny + 1) * nz + (j + 1))
                  .setPosition(ps);
            }
         }
      }

      // Generate the middle nodes
      p.set(edgePoints[6]);
      p.scaledAdd(endSide, principalAxis_z);
      rayDir.set(principalAxis_x);
      for (int i = 1; i < nz; i++) {
         for (int j = 1; j < ny; j++) {
            pe1 =
               nodes.get(i * (nx + 1) * (ny + 1) + j * (nx + 1)).getPosition();
            pe2 =
               nodes
                  .get(i * (nx + 1) * (ny + 1) + j * (nx + 1) + nx)
                  .getPosition();
            pe3.sub(pe2, pe1);
            pe3.normalize();
            pe3.scale(pe1.distance(pe2) / nx);
            pe4.set(pe1);
            for (int k = 0; k < nx - 1; k++) {
               pe4.add(pe3);
               nodes
                  .get(i * (nx + 1) * (ny + 1) + j * (nx + 1) + (k + 1))
                  .setPosition(pe4);
            }
         }
         p.scaledAdd(zStep, principalAxis_z);
      }

      fem.resetRestPosition();

      return fem;
   }

   public static FemModel3d createCylinderWithSurface(PolygonalMesh mesh,
      int nl, int n_in, int n_margin) {
      int n_end = 0;
      RigidTransform3d transform = getPrincipalAxes(mesh);
      return createEllipsoidWithSurface(
         mesh, transform, nl, n_in, n_margin, n_end);
   }

   public static FemModel3d createEllipsoidWithSurface(PolygonalMesh mesh,
      int nl, int n_in, int n_margin) {
      int n_end = n_margin + ((int)n_in / 2);
      RigidTransform3d transform = getPrincipalAxes(mesh);
      return createEllipsoidWithSurface(
         mesh, transform, nl, n_in, n_margin, n_end);
   }

   public static FemModel3d createEllipsoidWithSurface(PolygonalMesh mesh,
      RigidTransform3d alignment, int nl, int n_in, int n_margin) {
      int n_end = n_margin + ((int)n_in / 2);
      return createEllipsoidWithSurface(
         mesh, alignment, nl, n_in, n_margin, n_end);
   }

   public static FemModel3d createEllipsoidWithSurface(PolygonalMesh mesh,
      int nl, int n_in, int n_margin, int n_end) {
      RigidTransform3d transform = getPrincipalAxes(mesh);
      return createEllipsoidWithSurface(
         mesh, transform, nl, n_in, n_margin, n_end);
   }

   public static FemModel3d createEllipsoidWithSurface(PolygonalMesh mesh,
      RigidTransform3d alignment, int nl, int n_in, int n_margin, int n_end) {
      return createEllipsoidWithSurface(
         mesh, alignment, nl, n_in, n_margin, n_end, DEFAULT_BORDER_PERCENTAGE,
         DEFAULT_PROJECT_END_NODES);
   }

   public static FemModel3d createEllipsoidWithSurface(PolygonalMesh mesh,
      RigidTransform3d alignment, int nl, int n_in, int n_margin, int n_end,
      double borderPercentage, boolean projEndNodes) {
      BVTree bvh = mesh.getBVTree();
      Point3d[] edgePoints = getTightBox(mesh, alignment);

      Vector3d principalAxis_x = new Vector3d();
      Vector3d principalAxis_y = new Vector3d();
      Vector3d principalAxis_z = new Vector3d();
      alignment.R.getColumn(0, principalAxis_x);
      alignment.R.getColumn(1, principalAxis_y);
      alignment.R.getColumn(2, principalAxis_z);

      double l = edgePoints[0].distance(edgePoints[4]);
      double r_out = edgePoints[0].distance(edgePoints[1]) / 2;
      double r_in = r_out / (n_in + n_margin) * n_in;

      // Create template mesh
      FemModel3d fem =
         HybridFemFactory.createHybridMuscleEllipsoid_KeepNodes(
            l, r_in, r_out, nl, n_in, n_margin, n_end);

      Point3d pOnPlane = new Point3d();
      Point3d planeCentreStart = new Point3d();
      planeCentreStart.add(edgePoints[4]);
      planeCentreStart.add(edgePoints[5]);
      planeCentreStart.add(edgePoints[6]);
      planeCentreStart.add(edgePoints[7]);
      planeCentreStart.scale(1 / 4.0);
      Vector3d alongX = new Vector3d();
      alongX.sub(edgePoints[0], edgePoints[4]);
      planeCentreStart.scaledAdd(borderPercentage, alongX);
      alongX.scale((1 - 2 * borderPercentage) / nl);
      int[] nodesIdx;
      for (int i = 0; i <= nl; i++) {
         // Find a point on the plane
         pOnPlane.setZero();
         pOnPlane.scaledAdd(i, alongX, planeCentreStart);

         // Get nodes
         if (i < n_end) {
            nodesIdx = roundedBeamSliceNodes(n_end - i, i, nl, n_in, n_margin);
         } else if (i > nl - n_end) {
            nodesIdx =
               roundedBeamSliceNodes(i - (nl - n_end), i, nl, n_in, n_margin);
         } else {
            nodesIdx = roundedBeamSliceNodes(0, i, nl, n_in, n_margin);
         }

         if (nodesIdx != null) {
            FemNode3d[] nodes = new FemNode3d[nodesIdx.length];
            for (int j = 0; j < nodesIdx.length; j++) {
               if (nodesIdx[j] != -1) {
                  nodes[j] = fem.getNode(nodesIdx[j]);
               } else {
                  nodes[j] = new FemNode3d();
               }
            }
            mapNodesToSurfaceSlice(
               bvh, pOnPlane, principalAxis_z, principalAxis_x, nodes,
               4 * n_in);

            if (projEndNodes) {
               if (i == 0) {
                  projNodesToSurface(bvh, principalAxis_x, nodes);
               } else if (i == nl) {
                  Vector3d neg_principalAxis_x = new Vector3d(principalAxis_x);
                  neg_principalAxis_x.scale(-1);
                  projNodesToSurface(bvh, neg_principalAxis_x, nodes);
               }
            }
         }
      }

      fem.resetRestPosition();

      return removeUnwantedNodes(fem);
   }

   private static Point3d getEdgePoint(BVTree bvh, Point3d startPoint,
      Vector3d rayDir, Vector3d moveDir, double distLimit, int scale,
      double rayLimit, boolean endNode) {
      Point3d edgePoint = new Point3d();
      Point3d p = new Point3d(startPoint);
      Point3d pc = new Point3d();
      Point3d pc_p1 = new Point3d();
      Vector3d normal;
      Face face_ray;
      TriangleIntersector isect = new TriangleIntersector();
      BVFeatureQuery query = new BVFeatureQuery();
      Vector3d duv = new Vector3d();
      boolean origFlag, flag;
      double dist = 0;
      Point3d distPoint;

      //face_ray = bvh.intersect(p, rayDir, duv, isect);
      face_ray = query.nearestFaceAlongRay (
         null, duv, bvh, p, rayDir);      
      if (face_ray == null) {
         origFlag = false;
         flag = false;
      } else {
         origFlag = true;
         flag = true;
      }

      while (origFlag == flag && dist <= distLimit) {
         p.scaledAdd(scale, moveDir);
         //face_ray = bvh.intersect(p, rayDir, duv, isect);
         face_ray = query.nearestFaceAlongRay (
            null, duv, bvh, p, rayDir);
         if (face_ray == null)
            flag = false;
         else {
            edgePoint = projToSurface(p, rayDir, face_ray);
            distPoint = projToAxis(edgePoint, p, moveDir);
            if ((dist > distLimit / 4 && endNode)
               || distPoint.distance(edgePoint) <= rayLimit
               || dist > distLimit / 5) {
               flag = true;
               // drawVector(edgePoint, distPoint);
               // System.out.println("distPoint.dist="+distPoint.distance(edgePoint));
            }
         }

         dist = p.distance(startPoint);
      }

      if (origFlag == flag) {
         System.out.println("Warning: Can't find edge!");
         Vector2d duu = new Vector2d();
         face_ray = query.nearestFaceToPoint(edgePoint, duu, bvh, p);
      }
      if (flag == false) {
         p.sub(moveDir);
         p.scaledAdd(-scale, moveDir);
         //face_ray = bvh.intersect(p, rayDir, duv, isect);
         face_ray = query.nearestFaceAlongRay (
            null, duv, bvh, p, rayDir);
      }

      // Find projection
      if (face_ray != null) {
         normal = face_ray.getNormal();
         face_ray.computeCentroid(pc);
         pc_p1.set(pc);
         // pc-p1 (p1 = p)
         pc_p1.sub(p);
         // rayDir = (p2-p1)
         // [N.(pc-p1)/N.(p2-p1)](p2-p1)
         edgePoint.set(rayDir);
         edgePoint.scale(normal.dot(pc_p1) / normal.dot(rayDir));
         // p1 + [N.(pc-p1)/N.(p2-p1)](p2-p1)
         edgePoint.add(p);
      } else {
         return null;
      }
      return edgePoint;
   }

   private static void mapNodesToSurfaceSlice(BVTree bvh,
      Point3d pOnPlane, Vector3d vOnPlane, Vector3d normal,
      FemNode3d[] nodes, int n) {

      if ((nodes.length - 1) % n != 0) {
         System.out.println("Error: in mapNodesToSurfaceSlice(), " +
            "inconsistent number of nodes!");
         return;
      }
      int numNodePerAngle = nodes.length / n;
      //TriangleIntersector isect = new TriangleIntersector();
      BVFeatureQuery query = new BVFeatureQuery();
      Vector3d duv = new Vector3d();
      Face face;
      Point3d ps;

      // Find centre on the plane
      Point3d planeCentre = new Point3d(pOnPlane);
      planeCentre = getCentroid(bvh, planeCentre, vOnPlane, normal, n);

      double r = 0;
      double radStep = Math.toRadians(360 / n);
      double radStart = Math.toRadians(-45);
      double disStep;

      // Map centre node
      nodes[0].setPosition(planeCentre);

      // Map rest of the nodes
      for (int i = 0; i < n; i++) {
         // Find radius in each angle
         Vector3d ray = new Vector3d(vOnPlane);
         ray.transform(new RigidTransform3d(
            0, 0, 0, normal.x, normal.y, normal.z, radStart + i * radStep));
         //face = bvh.intersect(planeCentre, ray, duv, isect);
         face = query.nearestFaceAlongRay (
            null, duv, bvh, planeCentre, ray);         
         if (face != null) {
            ps = projToSurface(planeCentre, ray, face);
            r = ps.distance(planeCentre);
            // Map nodes according to this radius
            disStep = r / numNodePerAngle;
            for (int j = 0; j < numNodePerAngle; j++) {
               ps = new Point3d(planeCentre);
               ps.scaledAdd(-1 * disStep * (j + 1), ray);
               nodes[1 + i * numNodePerAngle + j].setPosition(ps);
            }
         } else {
            System.out.println("Warning: in mapNodesToSurfaceSlice(), " +
               "cannot project to surface!");
         }
      }

   }

   private static int[] roundedBeamSliceNodes(int startLayer, int depth,
      int nl, int n_in, int n_margin) {

      if (startLayer > (n_margin + ((int)n_in / 2) + n_in % 2)) {
         System.out.println("Error: in roundedBeamSliceNodes()," +
            "incorrect layer number!");
         return null;
      }

      int numLayer = n_margin + ((int)n_in / 2) + n_in % 2; // Not include
                                                            // centre
      int[] nodes = new int[(numLayer - startLayer) * 4 * n_in + 1];
      int[] nodeIdx;
      int count = 0;

      // Get centre node
      if (n_in % 2 != 0) {
         nodes[0] = -1;
      } else {
         nodeIdx = roundedBeamCircleNodes(numLayer, depth, nl, n_in, n_margin);
         nodes[0] = nodeIdx[0];
      }
      count++;

      for (int i = 0; i < 4 * n_in; i++) {
         for (int j = numLayer - 1; j >= startLayer; j--) {
            int layer_in = j - n_margin;
            nodeIdx =
               roundedBeamCircleNodesClockwise(j, depth, nl, n_in, n_margin);
            if (nodeIdx.length < 4 * n_in) {
               // Edge nodes
               // Top left
               if (i == 0) {
                  nodes[count] = nodeIdx[0];
                  // Top right
               } else if (i % n_in == 0 && i / n_in == 1) {
                  nodes[count] = nodeIdx[n_in - 2 * layer_in];
                  // Bottom right
               } else if (i % n_in == 0 && i / n_in == 2) {
                  nodes[count] = nodeIdx[2 * (n_in - 2 * layer_in)];
                  // Bottom left
               } else if (i % n_in == 0 && i / n_in == 3) {
                  nodes[count] = nodeIdx[3 * (n_in - 2 * layer_in)];
                  // Removed nodes
               } else if ((i / n_in == 0 || i / n_in == 2)
                  && (i % n_in <= layer_in || i % n_in >= n_in - layer_in)) {
                  nodes[count] = -1;
               } else if ((i / n_in == 1 || i / n_in == 3)
                  && (i % n_in <= layer_in || i % n_in >= n_in - layer_in)) {
                  nodes[count] = -1;
                  // Top nodes
               } else if (i / n_in == 0) {
                  nodes[count] = nodeIdx[i - layer_in];
                  // Right nodes
               } else if (i / n_in == 1) {
                  nodes[count] =
                     nodeIdx[i - n_in - layer_in + (n_in - 2 * layer_in)];
                  // Bottom nodes
               } else if (i / n_in == 2) {
                  nodes[count] =
                     nodeIdx[i - 2 * n_in - layer_in + (n_in - 2 * layer_in)
                        + (n_in - 1 - 2 * layer_in) + 1];
                  // Left nodes
               } else if (i / n_in == 3) {
                  nodes[count] =
                     nodeIdx[i - 3 * n_in - layer_in + 2
                        * (n_in - 2 * layer_in) + (n_in - 1 - 2 * layer_in) + 1];
               }
            } else {
               nodes[count] = nodeIdx[i];
            }
            count++;
         }
      }

      return nodes;
   }

   private static Point3d getCentroid(BVTree bvh, Point3d pOnPlane,
      Vector3d vOnPlane, Vector3d normal, double numStep) {
      return getCentroid(
         bvh, pOnPlane, vOnPlane, normal, numStep, Double.MAX_VALUE);
   }

   private static Point3d getCentroid(BVTree bvh, Point3d pOnPlane,
      Vector3d vOnPlane, Vector3d normal, double numStep, double prevDis) {
      
      //XXX I had to fill in zero for prevDis to prevent self-reference
      //    but no idea what it's for
      return getCentroid(
         bvh, pOnPlane, vOnPlane, normal, numStep, 0,DEFAULT_ACCURACY); 

   }

   private static Point3d getCentroid(BVTree bvh, Point3d pOnPlane,
      Vector3d vOnPlane, Vector3d normal, double numStep, double prevDis,
      double accuracyPercentage) {
      Vector3d ray = new Vector3d(vOnPlane);
      double radStep = Math.toRadians(360) / numStep;
      //TriangleIntersector isect = new TriangleIntersector();
      BVFeatureQuery query = new BVFeatureQuery();
      Vector3d duv = new Vector3d();
      Face face;
      Point3d centre = new Point3d();
      Point3d p;
      int count = 0;
      for (int i = 0; i < numStep; i++) {
         ray.transform(new RigidTransform3d(
            0, 0, 0, -normal.x, -normal.y, -normal.z, +i * radStep));
         //face = bvh.intersect(pOnPlane, ray, duv, isect);
         face = query.nearestFaceAlongRay (
            null, duv, bvh, pOnPlane, ray);         
         if (face != null) {
            p = projToSurface(pOnPlane, ray, face);
            centre.add(p);
            count++;
         }
      }
      if ((count < 1 || count < numStep / 2) && numStep < 360) {
         numStep *= 2;
         centre = getCentroid(bvh, pOnPlane, vOnPlane, normal, numStep);
      } else if (count < 1 && numStep >= 360) {
         System.out.println("Warning: in getCentroid()," +
            "cannot find centroid!");
         return null;
      } else {
         centre.scale(1.0 / count);
      }
      double newDis = centre.distance(pOnPlane);
      if ((prevDis - newDis) / prevDis > accuracyPercentage) {
         centre = getCentroid(bvh, centre, vOnPlane, normal, numStep, newDis);
      }
      return centre;
   }

   public static void projToCylinder(FemModel3d fem, int nl, int n_in,
      int n_margin) {
      int[] nodes;

      double y, z;
      double r;
      double radStep;
      double radStart;
      FemNode3d node;
      for (int layer = 0; layer < (n_margin + ((int)n_in / 2) + n_in % 2); layer++) {
         for (int i = 0; i <= nl; i++) {
            nodes = roundedBeamCircleNodes(layer, i, nl, n_in, n_margin);
            int count = 0;
            r =
               Math.sqrt(Math.pow(fem.getNode(nodes[0]).getPosition().z, 2)
                  + Math.pow(fem.getNode(nodes[0]).getPosition().z, 2));
            if (layer < n_margin) {
               radStep = Math.toRadians(90 / n_in);
               // Top
               radStart = Math.toRadians(-45);
               for (int j = 0; j < n_in + 1; j++) {
                  node = fem.getNode(nodes[count]);
                  y = r * Math.sin(radStart + radStep * j);
                  z = r * Math.cos(radStart + radStep * j);
                  node.setPosition(node.getPosition().x, y, z);
                  count++;
               }
               // Bottom
               radStart = Math.toRadians(225);
               for (int j = 0; j < n_in + 1; j++) {
                  node = fem.getNode(nodes[count]);
                  y = r * Math.sin(radStart - radStep * j);
                  z = r * Math.cos(radStart - radStep * j);
                  node.setPosition(node.getPosition().x, y, z);
                  count++;
               }
               // Right
               radStart = Math.toRadians(225) + radStep;
               for (int j = 0; j < n_in - 1; j++) {
                  node = fem.getNode(nodes[count]);
                  y = r * Math.sin(radStart + radStep * j);
                  z = r * Math.cos(radStart + radStep * j);
                  node.setPosition(node.getPosition().x, y, z);
                  count++;
               }
               // Left
               radStart = Math.toRadians(135) - radStep;
               for (int j = 0; j < n_in - 1; j++) {
                  node = fem.getNode(nodes[count]);
                  y = r * Math.sin(radStart - radStep * j);
                  z = r * Math.cos(radStart - radStep * j);
                  node.setPosition(node.getPosition().x, y, z);
                  count++;
               }
            } else {
               int layer_in = layer - n_margin;
               if ((n_in - 2 * layer_in) > 0) {
                  radStep = Math.toRadians(90 / (n_in - 2 * layer_in));
                  // Top
                  radStart = Math.toRadians(-45);
                  for (int j = 0; j <= n_in - 2 * layer_in; j++) {
                     node = fem.getNode(nodes[count]);
                     y = r * Math.sin(radStart + radStep * j);
                     z = r * Math.cos(radStart + radStep * j);
                     node.setPosition(node.getPosition().x, y, z);
                     count++;
                  }
                  // Bottom
                  radStart = Math.toRadians(225);
                  for (int j = 0; j <= n_in - 2 * layer_in; j++) {
                     node = fem.getNode(nodes[count]);
                     y = r * Math.sin(radStart - radStep * j);
                     z = r * Math.cos(radStart - radStep * j);
                     node.setPosition(node.getPosition().x, y, z);
                     count++;
                  }
                  // Right
                  radStart = Math.toRadians(225) + radStep;
                  for (int j = 0; j < n_in - 2 * layer_in - 1; j++) {
                     node = fem.getNode(nodes[count]);
                     y = r * Math.sin(radStart + radStep * j);
                     z = r * Math.cos(radStart + radStep * j);
                     node.setPosition(node.getPosition().x, y, z);
                     count++;
                  }
                  // Left
                  radStart = Math.toRadians(135) - radStep;
                  for (int j = 0; j < n_in - 2 * layer_in - 1; j++) {
                     node = fem.getNode(nodes[count]);
                     y = r * Math.sin(radStart - radStep * j);
                     z = r * Math.cos(radStart - radStep * j);
                     node.setPosition(node.getPosition().x, y, z);
                     count++;
                  }
               }
            }
         }
      }
      fem.resetRestPosition();
   }

   public static int[] roundedBeamCircleNodesClockwise(int layer, int depth,
      int nl, int n_in, int n_margin) {
      int[] nodesToReduceList =
         roundedBeamCircleNodes(layer, depth, nl, n_in, n_margin);
      int[] nodesRearranged = new int[nodesToReduceList.length];

      int count = 0;
      if (layer < n_margin) {
         // Top
         for (int j = n_in; j >= 0; j--) {
            nodesRearranged[count] = nodesToReduceList[j];
            count++;
         }
         // Right
         for (int j = n_in - 2; j >= 0; j--) {
            nodesRearranged[count] = nodesToReduceList[j + 2 * (n_in + 1)];
            count++;
         }
         // Bottom
         for (int j = 0; j < n_in + 1; j++) {
            nodesRearranged[count] = nodesToReduceList[j + (n_in + 1)];
            count++;
         }
         // Left
         for (int j = 0; j < n_in - 1; j++) {
            nodesRearranged[count] =
               nodesToReduceList[j + 2 * (n_in + 1) + (n_in - 1)];
            count++;
         }
      } else if (layer < (n_margin + ((int)n_in / 2) + n_in % 2)) {
         int layer_in = layer - n_margin;
         // Top
         for (int j = 0; j < (n_in + 1) - 2 * layer_in; j++) {
            nodesRearranged[count] =
               nodesToReduceList[(n_in + 1) - 2 * layer_in - 1 - j];
            count++;
         }
         // Right
         for (int j = 0; j < (n_in - 1) - 2 * layer_in; j++) {
            nodesRearranged[count] =
               nodesToReduceList[2 * ((n_in + 1) - 2 * layer_in) + (n_in - 1)
                  - 2 * layer_in - 1 - j];
            count++;
         }
         // Bottom
         for (int j = 0; j < (n_in + 1) - 2 * layer_in; j++) {
            nodesRearranged[count] =
               nodesToReduceList[j + (n_in + 1) - 2 * layer_in];
            count++;
         }
         // Left
         for (int j = 0; j < (n_in - 1) - 2 * layer_in; j++) {
            nodesRearranged[count] =
               nodesToReduceList[j + 2 * ((n_in + 1) - 2 * layer_in)
                  + ((n_in - 1) - 2 * layer_in)];
            count++;
         }
      } else if (n_in % 2 == 0) {
         nodesRearranged[0] = nodesToReduceList[0];
      }

      return nodesRearranged;
   }

   public static int[] roundedBeamCircleNodes(int layer, int depth, int nl,
      int n_in, int n_margin) {
      int numCenterNodes = (nl + 1) * (n_in + 1) * (n_in + 1);
      int numMarginNodes_TopBottom = (nl + 1) * (n_in + 1) * n_margin;
      int numMarginNodes_LeftRight = (nl + 1) * (n_in - 1) * n_margin;
      int[] nodesToReduceList = null;

      if (layer < n_margin) {
         nodesToReduceList = new int[4 * n_in];
         int count = 0;
         // Top
         for (int j = 0; j < n_in + 1; j++) {
            nodesToReduceList[count] =
               numCenterNodes + (nl + 1) * (n_in + 1) * (n_margin - 1 - layer)
                  + j * (nl + 1) + depth;
            count++;
         }
         // Bottom
         for (int j = n_in + 1; j < 2 * (n_in + 1); j++) {
            nodesToReduceList[count] =
               numCenterNodes + (nl + 1) * (n_in + 1) * (n_margin - 1 + layer)
                  + j * (nl + 1) + depth;
            count++;
         }
         // Right
         for (int j = 0; j < n_in - 1; j++) {
            nodesToReduceList[count] =
               numCenterNodes + 2 * numMarginNodes_TopBottom + (nl + 1) * layer
                  + j * (nl + 1) * n_margin + depth;
            count++;
         }
         // Left
         for (int j = 0; j < n_in - 1; j++) {
            nodesToReduceList[count] =
               numCenterNodes + 2 * numMarginNodes_TopBottom
                  + numMarginNodes_LeftRight + (nl + 1)
                  * (n_margin - 1 - layer) + j * (nl + 1) * n_margin + depth;
            count++;
         }
      } else if (layer < (n_margin + ((int)n_in / 2) + n_in % 2)) {
         int layer_in = layer - n_margin;
         nodesToReduceList = new int[4 * (n_in - 2 * layer_in)];
         int count = 0;
         // Top
         for (int j = layer_in; j <= n_in - layer_in; j++) {
            nodesToReduceList[count] =
               (n_in - layer_in) * (nl + 1) * (n_in + 1) + j * (nl + 1) + depth;
            count++;
         }
         // Bottom
         for (int j = layer_in; j <= n_in - layer_in; j++) {
            nodesToReduceList[count] =
               layer_in * (nl + 1) * (n_in + 1) + j * (nl + 1) + depth;
            count++;
         }
         // Right
         for (int j = layer_in + 1; j < n_in - layer_in; j++) {
            nodesToReduceList[count] =
               layer_in * (nl + 1) + j * (nl + 1) * (n_in + 1) + depth;
            count++;
         }
         // Left
         for (int j = layer_in + 1; j < n_in - layer_in; j++) {
            nodesToReduceList[count] =
               (n_in - layer_in) * (nl + 1) + j * (nl + 1) * (n_in + 1) + depth;
            count++;
         }
      } else if (n_in % 2 == 0) {
         nodesToReduceList = new int[1];
         nodesToReduceList[0] =
            (nl + 1) * (n_in + 1) * (n_in / 2) + (nl + 1) * (n_in / 2) + depth;
      }

      return nodesToReduceList;
   }

   public static FemModel3d reduceRoundedBeamCircle(FemModel3d fem, int layer,
      int depth, int nl, int n_in, int n_margin) {
      int[] nodesToReduceList =
         roundedBeamCircleNodes(layer, depth, nl, n_in, n_margin);
      FemModel3d newFem = reudceHexToWedgeAndTet(fem, nodesToReduceList);
      return removeWedge(newFem, nodesToReduceList);
   }

   public static FemModel3d removeUnwantedNodes(FemModel3d fem) {
      FemModel3d femResult = new FemModel3d();

      Hashtable<String,FemNode3d> hashedNodes =
         new Hashtable<String,FemNode3d>();
      Hashtable<String,FemNode3d> hashedNodesKeep =
         new Hashtable<String,FemNode3d>();

      String key;
      for (FemNode3d n : fem.getNodes()) {
         key =
            String.format(
               "%.5f_%.5f_%.5f", n.getPosition().x, n.getPosition().y,
               n.getPosition().z);
         FemNode3d n_duplicated = new FemNode3d(n.getPosition());
         hashedNodes.put(key, n_duplicated);
      }

      for (FemElement3d el : fem.getElements()) {
         if (el.numNodes() == 8) {
            FemNode3d[] el_nodes = new FemNode3d[8];
            int i = 0;
            for (FemNode3d n : el.getNodes()) {
               key =
                  String.format(
                     "%.5f_%.5f_%.5f", n.getPosition().x, n.getPosition().y,
                     n.getPosition().z);
               el_nodes[i] = hashedNodes.get(key);
               i++;
               if (!hashedNodesKeep.containsKey(key)) {
                  hashedNodesKeep.put(key, hashedNodes.get(key));
               }
            }
            femResult.addElement(new HexElement(el_nodes));
         } else if (el.numNodes() == 6) {
            FemNode3d[] el_nodes = new FemNode3d[6];
            int i = 0;
            for (FemNode3d n : el.getNodes()) {
               key =
                  String.format(
                     "%.5f_%.5f_%.5f", n.getPosition().x, n.getPosition().y,
                     n.getPosition().z);
               el_nodes[i] = hashedNodes.get(key);
               i++;
               if (!hashedNodesKeep.containsKey(key)) {
                  hashedNodesKeep.put(key, hashedNodes.get(key));
               }
            }
            femResult.addElement(new WedgeElement(el_nodes));
         } else if (el.numNodes() == 4) {
            FemNode3d[] el_nodes = new FemNode3d[4];
            int i = 0;
            for (FemNode3d n : el.getNodes()) {
               key =
                  String.format(
                     "%.5f_%.5f_%.5f", n.getPosition().x, n.getPosition().y,
                     n.getPosition().z);
               el_nodes[i] = hashedNodes.get(key);
               i++;
               if (!hashedNodesKeep.containsKey(key)) {
                  hashedNodesKeep.put(key, hashedNodes.get(key));
               }
            }
            femResult.addElement(new TetElement(
               el_nodes[0], el_nodes[1], el_nodes[2], el_nodes[3]));
         }
      }

      for (FemNode3d n : hashedNodesKeep.values()) {
         femResult.addNode(n);
      }

      return femResult;
   }

   public static FemModel3d
      removeWedge(FemModel3d fem, int[] nodesToReduceList) {
      FemModel3d femReduced = new FemModel3d();
      Hashtable<Integer,FemNode3d> hashedUndesiredNodes =
         new Hashtable<Integer,FemNode3d>();
      Hashtable<Integer,FemNode3d> hashedMapping =
         new Hashtable<Integer,FemNode3d>();

      for (int i : nodesToReduceList) {
         hashedUndesiredNodes.put(i, fem.getNode(i));
      }

      for (FemNode3d n : fem.getNodes()) {
         // if(!hashedUndesiredNodes.containsKey(n.getNumber())) {
         FemNode3d n_duplicated = new FemNode3d(n.getPosition());
         hashedMapping.put(n.getNumber(), n_duplicated);
         femReduced.addNode(n_duplicated);
         // }
      }
      int count, i;
      int[] nIdx = new int[8];
      int[] rIdx = new int[8];
      for (FemElement3d el : fem.getElements()) {
         count = 0;
         i = 0;
         for (FemNode3d n : el.getNodes()) {
            nIdx[i] = n.getNumber();
            if (hashedUndesiredNodes.containsKey(n.getNumber())) {
               rIdx[count] = i;
               count++;
            }
            i++;
         }

         if (!(count >= 2 && el.numNodes() == 6)) {
            if (el.numNodes() == 6) {
               FemNode3d[] el_WedgeNodes = new FemNode3d[6];
               for (int k = 0; k < 6; k++) {
                  el_WedgeNodes[k] = hashedMapping.get(nIdx[k]);
               }
               femReduced.addElement(new WedgeElement(el_WedgeNodes));
            } else if (el.numNodes() == 4) {
               FemNode3d[] el_TetNodes = new FemNode3d[4];
               for (int k = 0; k < 4; k++) {
                  el_TetNodes[k] = hashedMapping.get(nIdx[k]);
               }
               femReduced.addElement(new TetElement(
                  el_TetNodes[0],
                  el_TetNodes[1],
                  el_TetNodes[2],
                  el_TetNodes[3]
                  ));
            } else {
               FemNode3d[] el_HexNodes = new FemNode3d[8];
               int k = 0;
               for (int t : nIdx) {
                  el_HexNodes[k] = hashedMapping.get(t);
                  k++;
               }
               femReduced.addElement(new HexElement(el_HexNodes));
            }
         }
      }

      return femReduced;
   }

   public static FemModel3d reudceHexToWedgeAndTet(FemModel3d fem,
      int[] nodesToReduceList) {
      FemModel3d femReduced = new FemModel3d();
      Hashtable<Integer,FemNode3d> hashedUndesiredNodes =
         new Hashtable<Integer,FemNode3d>();
      Hashtable<Integer,FemNode3d> hashedMapping =
         new Hashtable<Integer,FemNode3d>();

      for (int i : nodesToReduceList) {
         hashedUndesiredNodes.put(i, fem.getNode(i));
      }

      for (FemNode3d n : fem.getNodes()) {
         // if(!hashedUndesiredNodes.containsKey(n.getNumber())) {
         FemNode3d n_duplicated = new FemNode3d(n.getPosition());
         hashedMapping.put(n.getNumber(), n_duplicated);
         femReduced.addNode(n_duplicated);
         // }
      }
      int count, i;
      int[] nIdx = new int[8];
      int[] rIdx = new int[8];
      for (FemElement3d el : fem.getElements()) {
         count = 0;
         i = 0;
         for (FemNode3d n : el.getNodes()) {
            nIdx[i] = n.getNumber();
            if (hashedUndesiredNodes.containsKey(n.getNumber())) {
               rIdx[count] = i;
               count++;
            }
            i++;
         }

         if (count == 2 && el.numNodes() == 8) {
            FemNode3d[] el_nodes = new FemNode3d[6];
            String caseCount = rIdx[0] + "_" + rIdx[1];

            if (caseCount.matches("0_1")) {
               int[] order = { 2, 6, 5, 3, 7, 4 };
               for (int x = 0; x < 6; x++) {
                  el_nodes[x] = hashedMapping.get(nIdx[order[x]]);
               }
               femReduced.addElement(new WedgeElement(el_nodes));
            } else if (caseCount.matches("1_2")) {
               int[] order = { 3, 7, 6, 0, 4, 5 };
               for (int x = 0; x < 6; x++) {
                  el_nodes[x] = hashedMapping.get(nIdx[order[x]]);
               }
               femReduced.addElement(new WedgeElement(el_nodes));
            } else if (caseCount.matches("2_3")) {
               int[] order = { 4, 7, 0, 5, 6, 1 };
               for (int x = 0; x < 6; x++) {
                  el_nodes[x] = hashedMapping.get(nIdx[order[x]]);
               }
               femReduced.addElement(new WedgeElement(el_nodes));
            } else if (caseCount.matches("4_5")) {
               int[] order = { 6, 1, 2, 7, 0, 3 };
               for (int x = 0; x < 6; x++) {
                  el_nodes[x] = hashedMapping.get(nIdx[order[x]]);
               }
               femReduced.addElement(new WedgeElement(el_nodes));
            } else if (caseCount.matches("5_6")) {
               int[] order = { 7, 2, 3, 4, 1, 0 };
               for (int x = 0; x < 6; x++) {
                  el_nodes[x] = hashedMapping.get(nIdx[order[x]]);
               }
               femReduced.addElement(new WedgeElement(el_nodes));
            } else if (caseCount.matches("6_7")) {
               int[] order = { 0, 4, 3, 1, 5, 2 };
               for (int x = 0; x < 6; x++) {
                  el_nodes[x] = hashedMapping.get(nIdx[order[x]]);
               }
               femReduced.addElement(new WedgeElement(el_nodes));
            } else if (caseCount.matches("0_3")) {
               int[] order = { 2, 7, 6, 1, 4, 5 };
               for (int x = 0; x < 6; x++) {
                  el_nodes[x] = hashedMapping.get(nIdx[order[x]]);
               }
               femReduced.addElement(new WedgeElement(el_nodes));
            } else if (caseCount.matches("4_7")) {
               int[] order = { 5, 0, 1, 6, 3, 2 };
               for (int x = 0; x < 6; x++) {
                  el_nodes[x] = hashedMapping.get(nIdx[order[x]]);
               }
               femReduced.addElement(new WedgeElement(el_nodes));
            } else if (caseCount.matches("0_4")) {
               int[] order = { 5, 6, 7, 1, 2, 3 };
               for (int x = 0; x < 6; x++) {
                  el_nodes[x] = hashedMapping.get(nIdx[order[x]]);
               }
               femReduced.addElement(new WedgeElement(el_nodes));
            } else if (caseCount.matches("1_5")) {
               int[] order = { 4, 6, 7, 0, 2, 3 };
               for (int x = 0; x < 6; x++) {
                  el_nodes[x] = hashedMapping.get(nIdx[order[x]]);
               }
               femReduced.addElement(new WedgeElement(el_nodes));
            } else if (caseCount.matches("2_6")) {
               int[] order = { 7, 4, 5, 3, 0, 1 };
               for (int x = 0; x < 6; x++) {
                  el_nodes[x] = hashedMapping.get(nIdx[order[x]]);
               }
               femReduced.addElement(new WedgeElement(el_nodes));
            } else if (caseCount.matches("3_7")) {
               int[] order = { 0, 2, 1, 4, 6, 5 };
               for (int x = 0; x < 6; x++) {
                  el_nodes[x] = hashedMapping.get(nIdx[order[x]]);
               }
               femReduced.addElement(new WedgeElement(el_nodes));
            } else {
               FemNode3d[] el_HexNodes = new FemNode3d[8];
               int k = 0;
               for (int t : nIdx) {
                  el_HexNodes[k] = hashedMapping.get(t);
                  if (hashedMapping.get(t) == null)
                     System.out.println("Can't find node: " + t);
                  k++;
               }
               femReduced.addElement(new HexElement(el_HexNodes));
            }
         } else if (count == 3 && el.numNodes() == 8) {
            FemNode3d[] el_nodes1 = new FemNode3d[4];
            FemNode3d[] el_nodes2 = new FemNode3d[4];
            String caseCount = rIdx[0] + "_" + rIdx[1] + "_" + rIdx[2];

            if (caseCount.matches("0_1_2")) {
               int[] order = { 3, 5, 7, 6, 7, 5, 3, 4 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("0_2_3")) {
               int[] order = { 5, 7, 1, 6, 1, 7, 5, 4 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("0_4_5")) {
               int[] order = { 1, 7, 2, 6, 2, 7, 1, 3 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("0_1_5")) {
               int[] order = { 4, 7, 2, 6, 2, 7, 4, 3 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("0_3_7")) {
               int[] order = { 4, 2, 5, 6, 5, 2, 4, 1 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("0_4_7")) {
               int[] order = { 3, 2, 5, 6, 5, 2, 3, 1 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("0_3_4")) {
               int[] order = { 7, 1, 6, 2, 6, 1, 7, 5 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("1_2_5")) {
               int[] order = { 7, 0, 6, 3, 6, 0, 7, 4 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));

            } else if (caseCount.matches("1_2_3")) {
               int[] order = { 0, 6, 4, 7, 4, 6, 0, 5 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("1_2_6")) {
               int[] order = { 3, 5, 4, 7, 4, 5, 3, 0 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("1_4_5")) {
               int[] order = { 0, 6, 3, 2, 3, 6, 0, 7 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("1_5_6")) {
               int[] order = { 4, 3, 2, 7, 2, 3, 4, 0 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("2_3_7")) {
               int[] order = { 6, 0, 5, 1, 5, 0, 6, 4 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("2_6_7")) {
               int[] order = { 3, 0, 5, 1, 5, 0, 3, 4 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("2_5_6")) {
               int[] order = { 1, 7, 0, 3, 0, 7, 1, 4 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("2_6_7")) {
               int[] order = { 3, 0, 5, 1, 5, 0, 3, 4 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("3_4_7")) {
               int[] order = { 6, 0, 1, 2, 1, 0, 6, 5 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("3_6_7")) {
               int[] order = { 2, 1, 4, 5, 4, 1, 2, 0 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("4_5_7")) {
               int[] order = { 6, 0, 2, 3, 2, 0, 6, 1 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("5_6_7")) {
               int[] order = { 4, 0, 2, 3, 2, 0, 4, 1 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("4_5_6")) {
               int[] order = { 7, 3, 1, 2, 1, 3, 7, 0 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("4_6_7")) {
               int[] order = { 5, 3, 1, 2, 1, 3, 5, 0 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else {
               FemNode3d[] el_HexNodes = new FemNode3d[8];
               int k = 0;
               for (int t : nIdx) {
                  el_HexNodes[k] = hashedMapping.get(t);
                  if (hashedMapping.get(t) == null)
                     System.out.println("Can't find node: " + t);
                  k++;
               }
               femReduced.addElement(new HexElement(el_HexNodes));
            }

         } else {
            if (el.numNodes() == 6 && count < 5) {
               FemNode3d[] el_WedgeNodes = new FemNode3d[6];
               for (int k = 0; k < 6; k++) {
                  el_WedgeNodes[k] = hashedMapping.get(nIdx[k]);
               }
               femReduced.addElement(new WedgeElement(el_WedgeNodes));
            } else if (el.numNodes() == 4 && count < 3) {
               FemNode3d[] el_TetNodes = new FemNode3d[4];
               for (int k = 0; k < 4; k++) {
                  el_TetNodes[k] = hashedMapping.get(nIdx[k]);
               }
               femReduced.addElement(new TetElement(
                  el_TetNodes[0],
                  el_TetNodes[1],
                  el_TetNodes[2],
                  el_TetNodes[3]
                  ));
            } else if (count < 4) {
               FemNode3d[] el_HexNodes = new FemNode3d[8];
               int k = 0;
               for (int t : nIdx) {
                  el_HexNodes[k] = hashedMapping.get(t);
                  k++;
               }
               femReduced.addElement(new HexElement(el_HexNodes));
            }
         }
      }

      return femReduced;
   }

   public static FemModel3d mergeNodes(FemModel3d fem, int[] nodelist0,
      int[] nodelist1) {
      if (nodelist0.length != nodelist1.length) {
         System.out
            .println("Warning: Cannot merge nodes, array size mismatch.");
         return null;
      }
      FemNode3d[] femNodelist0 = new FemNode3d[nodelist0.length];
      FemNode3d[] femNodelist1 = new FemNode3d[nodelist1.length];

      for (int i = 0; i < nodelist0.length; i++) {
         femNodelist0[i] = fem.getNode(nodelist0[i]);
         femNodelist1[i] = fem.getNode(nodelist1[i]);
      }
      return mergeNodes(fem, femNodelist0, femNodelist1);
   }

   public static FemModel3d mergeNodes(FemModel3d fem, FemNode3d[] nodelist0,
      FemNode3d[] nodelist1) {
      if (nodelist0.length != nodelist1.length) {
         System.out
            .println("Warning: Cannot merge nodes, array size mismatch.");
         return null;
      }

      FemModel3d femMerged = new FemModel3d();
      Hashtable<Integer,Integer> hashedPairs = new Hashtable<Integer,Integer>();
      Hashtable<Integer,Integer> hashedPairs_reverse =
         new Hashtable<Integer,Integer>();
      Hashtable<Integer,Point3d> hashedPairPosition =
         new Hashtable<Integer,Point3d>();
      Hashtable<Integer,FemNode3d> hashedMapping =
         new Hashtable<Integer,FemNode3d>();

      FemNode3d n0, n1;

      for (int i = 0; i < nodelist0.length; i++) {
         n0 = nodelist0[i];
         n1 = nodelist1[i];
         hashedPairs.put(n0.getNumber(), n1.getNumber());
         hashedPairs_reverse.put(n1.getNumber(), n0.getNumber());
         Point3d p = new Point3d();
         p.add(n0.getPosition(), n1.getPosition());
         p.scale(0.5);
         hashedPairPosition.put(n0.getNumber(), p);
      }

      for (FemNode3d n : fem.getNodes()) {
         if (!hashedPairs_reverse.containsKey(n.getNumber())) {
            FemNode3d n_duplicated;
            if (hashedPairs.containsKey(n.getNumber())) {
               n_duplicated =
                  new FemNode3d(hashedPairPosition.get(n.getNumber()));
               hashedMapping.put(hashedPairs.get(n.getNumber()), n_duplicated);
            } else {
               n_duplicated = new FemNode3d(n.getPosition());
            }
            femMerged.addNode(n_duplicated);
            hashedMapping.put(n.getNumber(), n_duplicated);

         }
      }

      for (FemElement3d el : fem.getElements()) {
         if (el.numNodes() == 8) {
            FemNode3d[] el_nodes = new FemNode3d[8];
            int i = 0;
            for (FemNode3d n : el.getNodes()) {
               el_nodes[i] = hashedMapping.get(n.getNumber());
               i++;
            }
            femMerged.addElement(new HexElement(el_nodes));
         } else if (el.numNodes() == 6) {
            FemNode3d[] el_nodes = new FemNode3d[6];
            int i = 0;
            for (FemNode3d n : el.getNodes()) {
               el_nodes[i] = hashedMapping.get(n.getNumber());
               i++;
            }
            femMerged.addElement(new WedgeElement(el_nodes));
         } else if (el.numNodes() == 4) {
            FemNode3d[] el_nodes = new FemNode3d[4];
            int i = 0;
            for (FemNode3d n : el.getNodes()) {
               el_nodes[i] = hashedMapping.get(n.getNumber());
               i++;
            }
            femMerged.addElement(new TetElement(
               el_nodes[0], el_nodes[1], el_nodes[2], el_nodes[3]));
         }
      }

      return femMerged;
   }

   public double getBorderPercentage() {
      return borderPercentage;
   }

   public void setBorderPercentage(double borderPercentage) {
      this.borderPercentage = borderPercentage;
   }

   public boolean getProjEndNodes() {
      return projEndNodes;
   }

   public void setProjEndNodes(boolean projEndNodes) {
      this.projEndNodes = projEndNodes;
   }

   public double getAccuracyPercentage() {
      return accuracyPercentage;
   }

   public void setAccuracyPercentage(double accuracyPercentage) {
      this.accuracyPercentage = accuracyPercentage;
   }

   public TemplateType getTemplateType() {
      return templateType;
   }

   public void setTemplateType(TemplateType templateType) {
      this.templateType = templateType;
   }

   public GeneratorType getType() {
      return GeneratorType.HYBRID;
   }
   
}

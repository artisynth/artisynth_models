package artisynth.models.palateV2;

import java.awt.Color;
import java.util.ArrayList;

import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import artisynth.core.femmodels.FemElement3dBase;
import artisynth.core.femmodels.FemMarker;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.materials.PeckAxialMuscle;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.Point;

public class SoftPalateMuscles 
{
   public static double muscleMaxForce = 3.0;
   public static double forceScaling = 1000.0;	// this seems better defined in probes!
   public static boolean randomColors = false;
   public static boolean allowExternal = false;
   
   public static String palatoglossus_L_name         = "LM_palatoglossus_L";
   public static String palatoglossus_R_name         = "LM_palatoglossus_R";
   public static String palatoglossus_post_L_name    = "LM_palatoglossus_post_L";
   public static String palatoglossus_post_R_name    = "LM_palatoglossus_post_R";
   public static String palatopharyngeus_L_name      = "LM_palatopharyngeus_L";
   public static String palatopharyngeus_R_name      = "LM_palatopharyngeus_R";
   public static String tensor_veli_palitini_L_name  = "LM_tensor-veli-palitini_L";
   public static String tensor_veli_palitini_R_name  = "LM_tensor-veli-palitini_R";
   public static String levator_veli_palitini_L_name = "LM_levator-veli-palitini_L";
   public static String levator_veli_palitini_R_name = "LM_levator-veli-palitini_R";
   public static String musculus_uvulae_L_name       = "LM_musculus-uvulae_L";
   public static String musculus_uvulae_R_name       = "LM_musculus-uvulae_R";
   
   // TODO: I need to add a simple way to define if a node should be: internalOnly, optionallyExternal, or alwayFixedNode
   
   public static MuscleBundle buildMuscleBundle(FemMuscleModel fem, String name, double[][] points, int[][] indices, boolean allowExternal, Color color)
   {
      MuscleBundle mb = new MuscleBundle();
      mb.setName(name);
      mb.setMaxForce(muscleMaxForce);
      fem.addMuscleBundle(mb);
      
      // create the fem markers, or optionally, external points
      int nPoints = points.length;
      int nFibres = indices.length;
      ArrayList<Point> bPoints = new ArrayList<Point>(nPoints);
      for (int a=0; a<nPoints; a++)
      {
	 Point3d pnt3d = new Point3d(points[a]);
	 Point point;

	 // add the marker to the model
	 FemElement3dBase elem = fem.findContainingElement (pnt3d);
	 //elem = null;
	 if ( (elem == null) && (allowExternal == false) ) 
	 {
	    // marker is outside of fem --> snap to surface
	    Point3d newLoc = new Point3d();
	    elem = fem.findNearestSurfaceElement (newLoc, pnt3d);
	    point = new FemMarker(elem, newLoc);
	    fem.addMarker((FemMarker)point);
	 }
	 else if ( (elem == null) && (allowExternal == true) )
	 {
	    // point is outside of fem --> create fem node
	    FemNode3d p = new FemNode3d(pnt3d);
	    p.setDynamic(false);
	    fem.addNode(p);
	    point = p;
	 }
	 else
	 {
	    // marker is inside the fem --> just add as it is
	    point = new FemMarker(elem, pnt3d);
	    fem.addMarker((FemMarker)point);
	 }
	 bPoints.add(point);
	 
      }
      
      // now construct the muscle bundles
      for (int a=0; a<nFibres; a++)
      {
	 Muscle fibre = new Muscle();
	 fibre.setFirstPoint  ( bPoints.get(indices[a][0]) );
	 fibre.setSecondPoint ( bPoints.get(indices[a][1]) );
	 PeckAxialMuscle mat = new PeckAxialMuscle();
	 mat.setForceScaling(forceScaling);		// is this needed? set universally for bundle?
	 mat.setMaxForce(muscleMaxForce);
	 double L = bPoints.get(indices[a][0]).distance(bPoints.get(indices[a][1]));
	 mat.setOptLength(1.0*L);
	 mat.setMaxLength(2.0*L);
	 if ( (bPoints.get(indices[a][0]).getClass() == FemNode3d.class) || (bPoints.get(indices[a][1]).getClass() == FemNode3d.class) )
	 {
	    // I only assign passive props to external muscles, otherwise the fem props are assumed to be contain the passive muscle props
	    mat.setPassiveFraction(2.0);
	 }
	 fibre.setMaterial(mat);
	 mb.addFibre (fibre);
      }
      
      if ( (color == null) || (randomColors == true) )
      {
	 Vector3d c = new Vector3d();
         c.setRandom (0.0, 1.0);
	 color = new Color((float)c.x,(float)c.y,(float)c.z);
      }
      mb.getRenderProps().setLineColor(color);
      
      mb.setFibresActive(true);
      return mb;
   }
   
   static double[][] ySymmetricPoints(double[][] points)
   {
      int nPoints = points.length;
      double[][] pOut = new double[nPoints][3];
      for (int a=0; a<points.length; a++)
      {
	 pOut[a][0] = points[a][0];
	 pOut[a][1] = points[a][1]*-1.0;
	 pOut[a][2] = points[a][2];
      }
      return pOut;
   }
   
   public static void addSoftPalateMuscles(FemMuscleModel fem)
   {
      palatoglossus_L(fem);
      palatoglossus_R(fem);
      palatoglossus_post_L(fem);
      palatoglossus_post_R(fem);
      palatopharyngeus_L(fem);
      palatopharyngeus_R(fem);
      tensor_veli_palitini_L(fem);
      tensor_veli_palitini_R(fem);
      levator_veli_palitini_L(fem);
      levator_veli_palitini_R(fem);
      musculus_uvulae_L(fem);
      musculus_uvulae_R(fem);
   }
   
   public static void palatoglossus_L(FemMuscleModel fem)
   {
      String name = palatoglossus_L_name;
      buildMuscleBundle(fem, name, palatoglossus_L_points(), palatoglossus_L_bundle(), true, Color.lightGray);
   }
   public static void palatoglossus_R(FemMuscleModel fem)
   {
      String name = palatoglossus_R_name;
      buildMuscleBundle(fem, name, palatoglossus_R_points(), palatoglossus_R_bundle(), true, Color.lightGray);
   }
   public static void palatoglossus_post_L(FemMuscleModel fem)
   {
      String name = palatoglossus_post_L_name;
      buildMuscleBundle(fem, name, palatoglossus_post_L_points(), palatoglossus_post_L_bundle(), true, Color.gray);
   }
   public static void palatoglossus_post_R(FemMuscleModel fem)
   {
      String name = palatoglossus_post_R_name;
      buildMuscleBundle(fem, name, palatoglossus_post_R_points(), palatoglossus_post_R_bundle(), true, Color.gray);
   }
   public static void palatopharyngeus_L(FemMuscleModel fem)
   {
      String name = palatopharyngeus_L_name;
      buildMuscleBundle(fem, name, palatopharyngeus_L_points(), palatopharyngeus_L_bundle(), true, Color.red);
   }
   public static void palatopharyngeus_R(FemMuscleModel fem)
   {
      String name = palatopharyngeus_R_name;
      buildMuscleBundle(fem, name, palatopharyngeus_R_points(), palatopharyngeus_R_bundle(), true, Color.red);
   }
   public static void tensor_veli_palitini_L(FemMuscleModel fem)
   {
      String name = tensor_veli_palitini_L_name;
      buildMuscleBundle(fem, name, tensor_veli_palitini_L_points(), tensor_veli_palitini_L_bundle(), true, Color.yellow);
   }
   public static void tensor_veli_palitini_R(FemMuscleModel fem)
   {
      String name = tensor_veli_palitini_R_name;
      buildMuscleBundle(fem, name, tensor_veli_palitini_R_points(), tensor_veli_palitini_R_bundle(), true, Color.yellow);
   }
   
   public static void levator_veli_palitini_L(FemMuscleModel fem)
   {
      String name = levator_veli_palitini_L_name;
      buildMuscleBundle(fem, name, levator_veli_palitini_L_points(), levator_veli_palitini_L_bundle(), true, Color.orange);
   }
   public static void levator_veli_palitini_R(FemMuscleModel fem)
   {
      String name = levator_veli_palitini_R_name;
      buildMuscleBundle(fem, name, levator_veli_palitini_R_points(), levator_veli_palitini_R_bundle(), true, Color.orange);
   }
   
   public static void musculus_uvulae_L(FemMuscleModel fem)
   {
      String name = musculus_uvulae_L_name;
      buildMuscleBundle(fem, name, musculus_uvulae_L_points(), musculus_uvulae_L_bundle(), true, Color.white);
   }
   public static void musculus_uvulae_R(FemMuscleModel fem)
   {
      String name = musculus_uvulae_R_name;
      buildMuscleBundle(fem, name, musculus_uvulae_R_points(), musculus_uvulae_R_bundle(), true, Color.white);
   }
   
   static double[][] palatoglossus_L_points()
   {
      double[][] points = {
	    
	    // anterior strand
	    {110.75,   0.00, 119.54},
	    {111.50,  -3.96, 119.53},
	    {112.25,  -8.24, 119.21},
	    {113.97, -13.34, 117.59},
	    {115.51, -18.11, 114.24},
	    {116.34, -22.34, 110.16},
	    {116.22, -24.24, 105.76},
	    {115.08, -25.25, 102.03},
	    {112.70, -25.00,  97.00},
	    {110.40, -24.30,  92.50},
	    //
	    {100.00, -19.00,  95.00},	// tongue attachment point
	    
	    
	    // posterior stand
	    {118.25,   0.00, 116.84},
	    {117.50,  -4.00, 116.97},
	    {117.55,  -7.35, 116.74},
	    {118.29, -11.28, 116.20},
	    {118.96, -16.75, 113.24},
	    {118.50, -21.50, 109.75},
	    {117.50, -24.00, 105.50},
	    {116.00, -24.80, 101.50},
	    {113.15, -24.85,  96.50},
	    {110.40, -24.30,  92.50},
	    
	    {101.00, -17.50, 92.00}};		// external tongue attachment point
      
      return points;
   }
   
   static double[][] palatoglossus_R_points()
   {
      return ySymmetricPoints(palatoglossus_L_points());
   }
   
   static int[][] palatoglossus_L_bundle()
   {
      int[][] conn = {
	    {0,1},
	    {1,2},
	    {2,3},
	    {3,4},
	    {4,5},
	    {5,6},
	    {6,7},
	    {7,8},
	    {8,9},
	    //
	    {9,10},	// external
	    
	    {11,12},
	    {12,13},
	    {13,14},
	    {14,15},
	    {15,16},
	    {16,17},
	    {17,18},
	    {18,19},
	    {19,20},
	    {20,21}};	// external
	    
      return conn;
   }
   
   static int[][] palatoglossus_R_bundle()
   {
      return palatoglossus_L_bundle();
   }
   
   static double[][] palatoglossus_post_L_points()
   {
      double[][] points = {
	    
	    {128.0,   0.00, 106.00},
	    {126.0,  -4.00, 109.0},
	    {125.0,  -7.50, 111.0},
	    {124.0, -12.00, 112.0},
	    {122.0, -16.75, 112.0},
	    {120.25, -21.25, 109.00},
	    {118.47, -23.78, 105.21},
	    {117.00, -24.34, 101.00},
	    {114.00, -24.70,  96.00},
	    {111.00, -24.50,  92.00},
	    
	    {106.00, -19.80,  91.00}};	// external attachment point
      

      return points;
   }
   
   static double[][] palatoglossus_post_R_points()
   {
      return ySymmetricPoints(palatoglossus_post_L_points());
   }
   
   static int[][] palatoglossus_post_L_bundle()
   {
      int[][] conn = {
	    {0,1},
	    {1,2},
	    {2,3},
	    {3,4},
	    {4,5},
	    {5,6},
	    {6,7},
	    {7,8},
	    {8,9},
	    {9,10}};
	    
      return conn;
   }
   
   static int[][] palatoglossus_post_R_bundle()
   {
      return palatoglossus_post_L_bundle();
   }

   static double[][] palatopharyngeus_L_points()
   {
      double[][] points = {
	    
	    {114.50,   0.00, 122.75},
	    {119.75, - 8.00, 120.85},
	    {123.50, -14.00, 117.05},
	    {127.25, -17.00, 110.40},
	    {130.25, -19.00, 102.80},
	    {132.13, -20.00,  95.20},
	    {133.25, -20.50,  88.55},
	    {134.00, -20.00,  83.80},
	    
	    {129.50,   0.00, 109.50},
	    {130.25,  -6.00, 108.00},
	    {131.00, -10.50, 104.50},
	    {132.00, -13.00, 101.50},
	    {133.00, -15.00,  97.50},
	    {134.00, -16.00,  93.49},
	    {134.00, -16.50,  88.55},
	    {134.00, -17.00,  83.80},
	    	    
	    {134.00, -19.00,  80.00}};	// an external "pharynx" point (old: {140.00, -18.00, 75.00}), {137.00, -18.00,  75.25}
      
      return points;
   }
   
   static double[][] palatopharyngeus_R_points()
   {
      return ySymmetricPoints(palatopharyngeus_L_points());
   }
   
   static int[][] palatopharyngeus_L_bundle()
   {
      int[][] conn = {
	    {0, 1},
	    {1, 2},
	    {2, 3},
	    {3, 4},
	    {4, 5},
	    {5, 6},
	    {6, 7},
	    {7, 16},
	    
	    {8, 9},
	    {9, 10},
	    {10, 11},
	    {11, 12},
	    {12, 13},
	    {13, 14},
	    {14, 15},
	    {15, 16}};
      return conn;
   }
   
   static int[][] palatopharyngeus_R_bundle()
   {
      return palatopharyngeus_L_bundle();
   }
   
   static int[] palatopharyngeus_L_fixedPoints()
   {
      int[] indices = {16};
      return indices;
   }
   
   static int[] palatopharyngeus_R_fixedPoints()
   {
      return palatopharyngeus_L_fixedPoints();
   }

   static double[][] tensor_veli_palitini_L_points()
   {
      double[][] points = {
	    
	    {106.37,  -1.00, 122.00},
	    {107.50,  -6.00, 121.50},
	    {109.00, -11.00, 120.00},
	    {110.50, -17.00, 118.00},
	    
	    {118.25,   0.00, 115.15},
	    {116.00,  -5.00, 116.10},
	    {114.50, -10.00, 116.10},
	    {113.00, -16.00, 116.00},
	    
	    {111.00, -22.50, 117.00},		// external "hamulus" point
	    {122.53, -19.19, 140.77}};		// external attachment point on skull
      
      // Some tensor points on the skull
      //    start of tensor (just below post-nasal spine): 	106.36919 -1.5530262 123.24639
      //    tensor at the hamulus: 				110.71474 -18.656095 117.19364
      //    tensor end point on skull: 				122.52522 -19.192179 140.76718
      
      return points;
   }
   
   static double[][] tensor_veli_palitini_R_points()
   {
      return ySymmetricPoints(tensor_veli_palitini_L_points());
   }
   
   static int[][] tensor_veli_palitini_L_bundle()
   {
      int[][] conn = {
	    {0, 1},
	    {1, 2},
	    {2, 3},
	    {3, 8},
	    
	    {4, 5},
	    {5, 6},
	    {6, 7},
	    {7, 8},
	    
	    {8, 9}};
      return conn;
   }
   
   static int[][] tensor_veli_palitini_R_bundle()
   {
      return tensor_veli_palitini_L_bundle();
   }
   
   static double[][] levator_veli_palitini_L_points()
   {
      double[][] points = {
	    
	    {113.00,   0.00, 120.85},
	    {115.25,  -6.22, 120.85},
	    {117.50, -10.84, 120.85},
	    {120.50, -15.10, 121.80},
	    {122.75, -17.60, 123.89},
	    {124.50, -18.80, 127.00},
	    
	    {122.75,   0.00, 116.10},
	    {122.00,  -6.00, 117.05},
	    {122.00, -13.00, 118.00},
	    {124.25, -15.50, 120.66},
	    {126.50, -17.30, 122.75},
	    {127.50, -18.50, 124.10},
	    
	    {128.64, -19.84, 141.63}};			// levator attachment point on the skull
      
      return points;
   }
   
   static double[][] levator_veli_palitini_R_points()
   {
      return ySymmetricPoints(levator_veli_palitini_L_points());
   }
   
   static int[][] levator_veli_palitini_L_bundle()
   {
      int[][] conn = {
	    {0, 1},
	    {1, 2},
	    {2, 3},
	    {3, 4},
	    {4, 5},
	    {5, 12},
	    
	    {6, 7},
	    {7, 8},
	    {8, 9},
	    {9, 10},
	    {10, 11},
	    {11, 12}};
      return conn;
   }
   
   static int[][] levator_veli_palitini_R_bundle()
   {
      return levator_veli_palitini_L_bundle();
   }
   
   static double[][] musculus_uvulae_L_points()
   {
      double[][] points = {
	    
	    {105.88, -2.00, 122.28},
	    {109.55, -2.00, 122.09},
	    {113.00, -2.00, 121.52},
	    {116.45, -2.00, 120.38},
	    {119.60, -2.00, 119.23},
	    {122.00, -2.00, 117.52},
	    {124.40, -2.00, 115.15},
	    {126.73, -2.00, 111.83},
	    {129.13, -2.00, 108.50},
	    {131.45, -2.00, 104.70}};
      
      return points;
   }
   
   static double[][] musculus_uvulae_R_points()
   {
      return ySymmetricPoints(musculus_uvulae_L_points());
   }

   static int[][] musculus_uvulae_L_bundle()
   {
      int[][] conn = {
	    {0, 1},
	    {1, 2},
	    {2, 3},
	    {3, 4},
	    {4, 5},
	    {5, 6},
	    {6, 7},
	    {7, 8},
	    {8, 9}};
      return conn;
   }
   
   static int[][] musculus_uvulae_R_bundle()
   {
      return musculus_uvulae_L_bundle();
   }

}

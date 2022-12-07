package artisynth.models.jawTongue;

import java.awt.Color;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.mechmodels.CollisionResponse;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.PointList;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.probes.Probe;
import artisynth.core.util.ArtisynthPath;
import artisynth.models.dynjaw.JawModel;
import maspack.geometry.PolygonalMesh;
import maspack.interpolation.Interpolation.Order;
import maspack.render.RenderProps;
import maspack.util.ReaderTokenizer;

public class BadinJawHyoidTongueContact extends BadinJawHyoidTongue
{
   
   public static final boolean DEFAULT_OC_MARKERS_VISIBLE_VALUE = true;
   
   /** The oral cavity rigid body. */
   protected RigidBody myOralCavity;

   /** A reference to the oral cavity's front region frame markers. */
   protected PointList<FrameMarker> myFrontMarkers;

   /** A reference to the oral cavity's mid region frame markers. */
   protected PointList<FrameMarker> myMidMarkers;

   /** A reference to the oral cavity's back region frame markers. */
   protected PointList<FrameMarker> myBackMarkers;

   /** A reference to the oral cavity's lateral left region frame markers. */
   protected PointList<FrameMarker> myLateralLeftMarkers;

   /** A reference to the oral cavity's lateral right region frame markers. */
   protected PointList<FrameMarker> myLateralRightMarkers;

   /**
    * A reference to names of the "medial" frame markers of the oral cavity's
    * front and mid region markers.
    */
   protected List<String> myCoronalMarkerNames;

   /**
    * Color to use when rendering the frame markers in the front region of the
    * oral cavity.
    */
   public static final Color FRONT_MARKERS_COLOR = new Color (0, 100, 200); // Blue

   /**
    * Color to use when rendering the frame markers in the mid region of the
    * oral cavity.
    */
   public static final Color MID_MARKERS_COLOR = new Color (0, 127, 0); // Green

   /**
    * Color to use when rendering the frame markers in the back region of the
    * oral cavity.
    */
   public static final Color BACK_MARKERS_COLOR = new Color (200, 100, 0); // Orange

   /**
    * Color to use when rendering the frame markers in the lateral regions of
    * the oral cavity.
    */
   public static final Color LATERAL_MARKERS_COLOR = new Color (127, 0, 0); // Red

   /**
    * Color to use when rendering the frame markers in the coronal region of the
    * oral cavity.
    */
   public static final Color CORONAL_MARKERS_COLOR = new Color (51, 0, 102); // Purple

   /** Small value to compare two doubles for equality. */
   protected static double EPSILON = 0.0001;
   
   /**
    * The distance monitor which measures (and can variously render) the
    * distance between the tongue and the oral cavity.
    */
   protected DistanceMonitor myDistanceMonitor;
   
   public BadinJawHyoidTongueContact() {
      super();
   }
   
   public DistanceMonitor getDistanceMonitor () {
      return myDistanceMonitor;
   }
   
   /**
    * Returns the oral cavity.
    * 
    * @return the oral cavity
    */
   public RigidBody getOralCavity () {
      return myOralCavity;
   }
   
   /**
    * Returns the tongue.
    * 
    * @return the tongue
    */
   public FemMuscleModel getTongue () {
      return tongue;
   }

   /**
    * Returns the list of frame markers belonging to the front region of the
    * oral cavity.
    * 
    * @return the list of frame markers belonging to the front region of the
    * oral cavity
    */
   public PointList<FrameMarker> getFrontOralCavityMarkers () {
      return myFrontMarkers;
   }

   /**
    * Returns the list of frame markers belonging to the mid region of the oral
    * cavity.
    * 
    * @return the list of frame markers belonging to the mid region of the oral
    * cavity
    */
   public PointList<FrameMarker> getMidOralCavityMarkers () {
      return myMidMarkers;
   }

   /**
    * Returns the list of frame markers belonging to the back region of the oral
    * cavity.
    * 
    * @return the list of frame markers belonging to the back region of the oral
    * cavity
    */
   public PointList<FrameMarker> getBackOralCavityMarkers () {
      return myBackMarkers;
   }

   /**
    * Returns the list of frame markers belonging to the lateral left region of
    * the oral cavity.
    * 
    * @return the list of frame markers belonging to the lateral left region of
    * the oral cavity
    */
   public PointList<FrameMarker> getLateralLeftOralCavityMarkers () {
      return myLateralLeftMarkers;
   }

   /**
    * Returns the list of frame markers belonging to the lateral right region of
    * the oral cavity.
    * 
    * @return the list of frame markers belonging to the lateral right region of
    * the oral cavity
    */
   public PointList<FrameMarker> getLateralRightOralCavityMarkers () {
      return myLateralRightMarkers;
   }

   /**
    * Returns the list of the names of the frame markers belonging to the medial
    * part of the front and mid regions of the oral cavity.
    * 
    * @return the list of the names of the frame markers belonging to the medial
    * part of the front and mid regions of the oral cavity
    */
   public List<String> getCoronalFrontAndMidMarkerNames () {
      return myCoronalMarkerNames;
   }

   /**
    * Returns a list of frame markers belonging to all regions of the oral
    * cavity.
    * 
    * @return a list of frame markers belonging to all regions of the oral
    * cavity
    */
   public PointList<FrameMarker> getAllOralCavityMarkers () {
      PointList<FrameMarker> list =
         new PointList<FrameMarker> (FrameMarker.class);

      // No need to add coronal markers, since they are just references
      // to some of the front and mid markers.
      for (FrameMarker mkr : myFrontMarkers) {
         list.add (mkr);
      }
      for (FrameMarker mkr : myMidMarkers) {
         list.add (mkr);
      }
      for (FrameMarker mkr : myBackMarkers) {
         list.add (mkr);
      }
      for (FrameMarker mkr : myLateralLeftMarkers) {
         list.add (mkr);
      }
      for (FrameMarker mkr : myLateralRightMarkers) {
         list.add (mkr);
      }

      return list;
   }
   
   @Override
   public void build (String[] args) throws IOException {
      super.build (args); // Create StaticJawHyoidTongue model.

      addOralCavityToModel ("geometry/oralCavityMesh.obj");
      addOralCavityFrameMarkersToModel();
      
      myDistanceMonitor =
         new DistanceMonitor (this, 754, DistanceMonitor.DEFAULT_RENDER_TYPE);
      addMonitor (myDistanceMonitor);

      // addMonitor (new RunUntilSettledScaled (tongue));
   }
   
   /**
    * Creates the oral cavity rigid body from the mesh in the specified file,
    * and adds the body to the model.
    * 
    * @param filenameRelToClass
    * file containing the mesh, specified relative to the location of the
    * {@link StaticOralCavityTongue} class.
    * @throws IOException
    */
   protected void addOralCavityToModel (String filenameRelToClass)
      throws IOException {
      // Create oral cavity rigid body from mesh.
      PolygonalMesh oralCavityMesh =
         new PolygonalMesh (
            ArtisynthPath.getSrcRelativeFile (
               BadinJawHyoidTongueContact.class, filenameRelToClass));
      myOralCavity =
         RigidBody.createFromMesh ("oralCavity", oralCavityMesh, 1.0, 1.0);
      myOralCavity.setDynamic (false);
      myJawModel.addRigidBody (myOralCavity);

      // Enable tongue-oralCavity collisions, and disable tongue-jaw-maxilla
      // collisions.
      RigidBody jaw = myJawModel.rigidBodies ().get ("jaw");
      RigidBody maxilla = myJawModel.rigidBodies ().get ("maxilla");
      myJawModel.setCollisionBehavior (tongue, myOralCavity, true, 0.0);
      myJawModel.setCollisionBehavior (myOralCavity, jaw, false);
      myJawModel.setCollisionBehavior (myOralCavity, maxilla, false);
      myJawModel.setCollisionBehavior (jaw, tongue, false);
      myJawModel.setCollisionBehavior (maxilla, tongue, false);
      
      myJawModel.setCollisionResponse (tongue, myOralCavity);
      RigidBody.setVisible(myOralCavity, false);
   }
   

   /**
    * Creates all 96 oral cavity frame markers from the files in the
    * frameMarkers folder of this package, and groups the frame markers by
    * region, placing them in lists.
    * 
    * @throws IOException
    */
   protected void addOralCavityFrameMarkersToModel () throws IOException {
      myFrontMarkers =
         new PointList<FrameMarker> (FrameMarker.class, "frontMarkers");
      myMidMarkers =
         new PointList<FrameMarker> (FrameMarker.class, "midMarkers");
      myBackMarkers =
         new PointList<FrameMarker> (FrameMarker.class, "backMarkers");
      myLateralLeftMarkers =
         new PointList<FrameMarker> (FrameMarker.class, "lateralLeftMarkers");
      myLateralRightMarkers =
         new PointList<FrameMarker> (FrameMarker.class, "lateralRightMarkers");
      myCoronalMarkerNames = new ArrayList<String> ();

      addOralCavityFrameMarkersToListOrChangeMarkerNamesForCoronalMarkers (
         "frameMarkers/ocMarkers_front.txt", myFrontMarkers, "oralCavity_f_",
         FRONT_MARKERS_COLOR, false);
      addOralCavityFrameMarkersToListOrChangeMarkerNamesForCoronalMarkers (
         "frameMarkers/ocMarkers_mid.txt", myMidMarkers, "oralCavity_m_",
         MID_MARKERS_COLOR, false);
      addOralCavityFrameMarkersToListOrChangeMarkerNamesForCoronalMarkers (
         "frameMarkers/ocMarkers_back.txt", myBackMarkers, "oralCavity_b_",
         BACK_MARKERS_COLOR, false);
      addOralCavityFrameMarkersToListOrChangeMarkerNamesForCoronalMarkers (
         "frameMarkers/ocMarkers_lat_left.txt", myLateralLeftMarkers,
         "oralCavity_ll_", LATERAL_MARKERS_COLOR, false);
      addOralCavityFrameMarkersToListOrChangeMarkerNamesForCoronalMarkers (
         "frameMarkers/ocMarkers_lat_right.txt", myLateralRightMarkers,
         "oralCavity_lr_", LATERAL_MARKERS_COLOR, false);
      addOralCavityFrameMarkersToListOrChangeMarkerNamesForCoronalMarkers (
         "frameMarkers/coronalMarkers.txt", null, "oralCavity_c_",
         CORONAL_MARKERS_COLOR, true);
      
      for (FrameMarker mkr : myFrontMarkers) {
         if (mkr.getName ().contains ("oralCavity_c_")) {
            myCoronalMarkerNames.add (mkr.getName ());
         }
      }
      for (FrameMarker mkr : myMidMarkers) {
         if (mkr.getName ().contains ("oralCavity_c_")) {
            myCoronalMarkerNames.add (mkr.getName ());
         }
      }

      // Don't add coronal markers to mech model, since they are just references
      // to markers already added through myFrontMarkers and myMidMarkers.
      myJawModel.add (myFrontMarkers);
      myJawModel.add (myMidMarkers);
      myJawModel.add (myBackMarkers);
      myJawModel.add (myLateralLeftMarkers);
      myJawModel.add (myLateralRightMarkers);
   }

   /**
    * Creates oral cavity frame markers at the locations specified in the given
    * file, and places them in the given list.
    * <p>
    * The format of the file is as follows:
    * 
    * <pre>
    * N
    * mkr1.x mkr1.y mkr1.z
    * mkr2.x mkr2.y mkr2.z
    * ...
    * mkrN.x mkrN.y mkrN.z
    * </pre>
    * 
    * @param filenameRelToClass
    * file containing the marker locations, specified relative to the
    * {@link StaticOralCavityTongue} class.
    * @param list
    * list in which to place the frame markers
    * @param name
    * a string name to identify the frame markers for this region of the oral
    * cavity, e.g. "f" or "front" for the front region
    * @param color
    * the color used when rendering the frame markers for this region
    * @throws IOException
    */
   protected void addOralCavityFrameMarkersToListOrChangeMarkerNamesForCoronalMarkers (
      String filenameRelToClass, PointList<FrameMarker> list, String name,
      Color color, boolean isCoronalMarkersList) throws IOException {
      File mkrFile =
         ArtisynthPath.getSrcRelativeFile (
            BadinJawHyoidTongueContact.class, filenameRelToClass);
      ReaderTokenizer rtok = new ReaderTokenizer (new FileReader (mkrFile));
      rtok.nextToken (); // Number of frame markers in file.
      int numMkrs = (int)rtok.nval;
      for (int i = 0; i < numMkrs; i++) {
         FrameMarker mkr = createFrameMarkerFromFile (rtok);
         if (isCoronalMarkersList) {
            FrameMarker coronalMkr =
               markerToAddToCoronalList (mkr, myFrontMarkers);
            if (coronalMkr == null) {
               coronalMkr = markerToAddToCoronalList (mkr, myMidMarkers);
            }
            if (coronalMkr == null) {
               System.out.println ("Warning: Coronal marker " + mkr
               + " not found in amongst front or mid markers!");
            }
            mkr = coronalMkr;
         }
         else {
            list.add (mkr);
         }
         if (mkr != null) {
            mkr.setName (name + (i + 1));
            RenderProps.setPointColor (mkr, color);
            RenderProps.setVisible (mkr, DEFAULT_OC_MARKERS_VISIBLE_VALUE);
         }
      }
   }

   /**
    * Reads the next three tokens from the file associated with {@code rtok},
    * treating them as {@code double} values, {@code x}, {@code y}, and
    * {@code z}. Creates a new frame marker on the oral cavity at the given
    * {@code (x, y, z)} location.
    * 
    * @param rtok
    * the reader used to read from a file
    * @return a new frame marker on the oral cavity
    * @throws IOException
    */
   protected FrameMarker createFrameMarkerFromFile (ReaderTokenizer rtok)
      throws IOException {
      rtok.nextToken (); // x coordinate of current frame marker
      double x = rtok.nval;
      rtok.nextToken (); // y coordinate of current frame marker
      double y = rtok.nval;
      rtok.nextToken (); // z coordinate of current frame marker
      double z = rtok.nval;

      return new FrameMarker (myOralCavity, x, y, z);
   }

   /**
    * Returns a reference to the frame marker in {@code crossCheckList} whose
    * position is equal to that of {@code mkrFromCoronalFile} within an error
    * tolerance of {@link #EPSILON}, or {@code null} if no such marker is found.
    * 
    * @param mkrFromCoronalFile
    * marker whose position should be equal to some frame marker in
    * {@code crossCheckList}
    * @param crossCheckList
    * list which should contain a marker whose position is equal to that of
    * {@code mkrFromCoronalFile}
    * @return a reference to the marker in {@code crossCheckList} whose position
    * is equal to that of {@code mkrFromCoronalFile}, or {@code null} if none
    * exists
    */
   protected FrameMarker markerToAddToCoronalList (
      FrameMarker mkrFromCoronalFile, PointList<FrameMarker> crossCheckList) {
      for (FrameMarker mkr : crossCheckList) {
         double[] mkrPos = new double[3];
         double[] mkrFromCoronalFilePos = new double[3];
         mkr.getPosition ().get (mkrPos);
         mkrFromCoronalFile.getPosition ().get (mkrFromCoronalFilePos);
         boolean equalMkrPos = true;
         for (int i = 0; i < mkrPos.length; i++) {
            equalMkrPos =
               equalMkrPos
               && Math.abs (mkrPos[i] - mkrFromCoronalFilePos[i]) < EPSILON;
         }
         if (equalMkrPos) {
            return mkr;
         }
      }
      return null;
   }

   public void addExciterProbe(String exciterName, double maxExcitation) {
    if (getInputProbes().get (exciterName + " exciter probe") == null) {
    NumericInputProbe nip =
       new NumericInputProbe(this, "models/jawmodel/models/tongue/exciters/" + exciterName
       + ":excitation", 0, 0.5);
    nip.addData (
       new double[] { 0.00, 0.0,
                      0.03, 0.0,
                      0.40, maxExcitation,
                      0.50, maxExcitation
                    }, NumericInputProbe.EXPLICIT_TIME);
    nip.setName (exciterName + " exciter probe");
    nip.setInterpolationOrder (Order.CubicStep);
    addInputProbe (nip);
    System.out.println("adding probe");
    System.out.println(exciterName + " " + maxExcitation);
 }
}

   public void removeExciterProbe(String exciterName) {
      Probe p = getInputProbes().get(exciterName + " exciter probe");
      if (p != null) {
         removeInputProbe(p);
      }
   }
   
   /**
    * Is the tongue in contact with the oralCavity?
    * 
    * @return {@code true} if tongue-oralCavity contact exists, {@code false}
    * otherwise
    */
   public boolean isTongueInContact () {
      CollisionResponse resp = 
         myJawModel.getCollisionResponse(tongue, myOralCavity);
      return resp.inContact();
   }
   
   /**
    * Returns the jaw model, which is this root model's MechModel.
    * 
    * @return the jaw model, which is this root model's MechModel
    */
   public JawModel getJawModel () {
      return myJawModel;
   }

}

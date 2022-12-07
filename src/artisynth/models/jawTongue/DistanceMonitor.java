package artisynth.models.jawTongue;

import java.awt.Color;
import java.awt.Component;
import java.util.AbstractMap.SimpleEntry;
import java.util.ArrayList;
import java.util.Collections;

import maspack.geometry.BVFeatureQuery;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import maspack.render.color.RainbowColorMap;
import maspack.widgets.EnumSelector;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemMarker;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.PointList;
import artisynth.core.modelbase.MonitorBase;

/**
 * This class monitor's the {@link StaticOCTPlayer} model or any subclass of the
 * model that includes the tongue and the oral cavity. At each time step, the
 * distance between some or all points on the oral cavity and the tongue is
 * calculated, and the color used to render these points is (possibly) changed,
 * according to these distances.
 */
public class DistanceMonitor extends MonitorBase {

   /** The root model to which this monitor has been added. */
   protected BadinJawHyoidTongueContact myRoot;

   /**
    * A {@link BVFeatureQuery} used to find the minimum distances between
    * various points on different polygonal meshes.
    */
   protected BVFeatureQuery myQuery;

   /**
    * A marker to represent the nearest point on the oral cavity to a given
    * point ({@link #myNodeMarker}) on the tongue.
    */
   protected FrameMarker myNodeNearestFrameMarker;

   /**
    * A marker to represent the point on the tongue for which we would like to
    * find the nearest point on the oral cavity.
    */
   protected FemMarker myNodeMarker;

   /**
    * A color map used to color the oral cavity if
    * <code>{@link #myRenderType} == {@link #RenderType}.ORAL_CAVITY_SURFACE_DISTANCE_MAP</code>
    * .
    */
   protected RainbowColorMap myColors;

   /**
    * A list of entries, one per frame marker on the oral cavity, where the
    * {@link Double} value of the entry represents the distance between said
    * frame marker and the nearest point on the tongue surface mesh.
    */
   protected ArrayList<SimpleEntry<FrameMarker,Double>> myDistanceMappingList;

   /**
    * The type of distance rendering performed by this monitor during the
    * current time step.
    */
   protected RenderType myRenderType;

   /**
    * {@code true} if, and only if, the value of {@link #myRenderType} has not
    * changed since the previous time step.
    */
   protected boolean myRenderTypeValid;

   /**
    * The maximum distance between a frame marker and the nearest point on the
    * tongue for which it is assumed that the two are "in contact".
    */
   public static final double CONTACT_THRESHOLD = 0.5;

   /**
    * An enum type representing all the different ways in which this monitor can
    * calculate and render the distance between the oral cavity and the tongue.
    */
   public static enum RenderType {
      /**
       * For each oral cavity frame marker, find the nearest point on the tongue
       * and record the Euclidean distance between these points. Also, brighten
       * the frame markers for which the distance calculated is less than
       * {@link DistanceMonitor#CONTACT_THRESHOLD}.
       */
      FRAME_MARKERS_DISTANCE_MAP,

      /** Not very useful, and not really working at this point. */
      NODE_NEAREST_POINT,

      /** Do nothing; no calculations or rendering. */
      NONE,

      /**
       * Changes the color each vertex of the oral cavity based on its distance
       * to the nearest point on the tongue, relative to this distance for every
       * other node.
       */
      ORAL_CAVITY_SURFACE_DISTANCE_MAP;
   }

   /** The default {@link RenderType} used by this monitor. */
   public static final RenderType DEFAULT_RENDER_TYPE =
      RenderType.FRAME_MARKERS_DISTANCE_MAP;

   static PropertyList myProps =
      new PropertyList (DistanceMonitor.class, MonitorBase.class);

   @Override
   public PropertyList getAllPropertyInfo () {
      return myProps;
   }

   static {
      myProps.add (
         "renderType", "the type of distance rendering to perform",
         DEFAULT_RENDER_TYPE);
   }

   public RenderType getRenderType () {
      return myRenderType;
   }

   public void setRenderType (RenderType type) {
      myRenderTypeValid = (myRenderType == type);
      myRenderType = (type == null) ? DEFAULT_RENDER_TYPE : type;

      // Synchronizes the control panel widget with the actual property
      // setting, in case this method is called directly by the user.
      for (Component c : myRoot
         .getControlPanels ().get ("OCT Controls").getPropertyPanel ()
         .getWidgets ()) {
         if ("renderTypeWidget".equals (c.getName ())) {
            ((EnumSelector)c).setValue (myRenderType);
         }
      }
   }

   /**
    * Creates a new {@link DistanceMonitor}.
    * 
    * @param rootModel
    * the model to which this monitor has been or will be added
    * @param tongueNodeIdx
    * the index of the tongue node to use when {@link #myRenderType} is set to
    * {@link RenderType}.NODE_NEAREST_POINT
    * @param type
    * the initial {@link RenderType} to use for distance rendering
    */
   public DistanceMonitor (BadinJawHyoidTongueContact rootModel, int tongueNodeIdx,
   RenderType type) {
      myRoot = rootModel;
      myQuery = new BVFeatureQuery ();
      myRenderType = (type == null) ? DEFAULT_RENDER_TYPE : type;
      myRenderTypeValid = true;
      setName ("DistanceMonitor");

      setUpOralCavitySurfaceDistanceMap ();
      setUpNodeNearestPoint (tongueNodeIdx);
      setUpFrameMarkersDistanceMap ();
   }

   protected void setUpOralCavitySurfaceDistanceMap () {
      myColors = new RainbowColorMap ();
   }

   protected void setUpNodeNearestPoint (int tongueNodeIdx) {
      // Create and set properties for myNodeMarker
      FemNode3d myNode = myRoot.getTongue ().getNode (tongueNodeIdx);
      FemElement3d myElement =
         myRoot.getTongue ().getElementNeighbors (myNode).get (0);
      myNodeMarker = new FemMarker (myElement, myNode.getPosition ());
      myNodeMarker.setName ("TongueNodeMarker");
      myRoot.getJawModel ().addRenderable (myNodeMarker);
      if (myNodeMarker.getRenderProps () == null) {
         myNodeMarker.setRenderProps (myNodeMarker.createRenderProps ());
      }
      RenderProps.setPointColor (myNodeMarker, Color.GREEN);
      RenderProps.setPointRadius (myNodeMarker, 0.5);
      RenderProps.setVisible (myNodeMarker, false);

      // Create and set properties for myNodeNearestFrameMarker
      myNodeNearestFrameMarker = new FrameMarker ("NodeNearestMarker");
      myNodeNearestFrameMarker.setFrame (myRoot.getOralCavity ());
      myRoot.getJawModel ().addFrameMarker (myNodeNearestFrameMarker);
      RenderProps.setPointColor (myNodeNearestFrameMarker, Color.GREEN);
      RenderProps.setPointRadius (myNodeNearestFrameMarker, 0.5);
      RenderProps.setVisible (myNodeNearestFrameMarker, false);
   }

   protected void setUpFrameMarkersDistanceMap () {
      PointList<FrameMarker> list = myRoot.getAllOralCavityMarkers ();
      myDistanceMappingList =
         new ArrayList<SimpleEntry<FrameMarker,Double>> (list.size ());
      for (FrameMarker mkr : list) {
         myDistanceMappingList
            .add (new SimpleEntry<FrameMarker,Double> (mkr, -1.0));
      }
   }

   /**
    * Returns a list of entries mapping the oral cavity frame markers to the
    * distance between each frame marker and the nearest point on the oral
    * cavity.
    * 
    * @return a list mapping frame markers to tongue distances
    */
   public ArrayList<SimpleEntry<FrameMarker,Double>> getDistanceMappingList () {
      return myDistanceMappingList;
   }

   /**
    * Convenience method. Returns a list of entries, one for each entry,
    * {@code E}, in the given {@code distanceMappingList}, where - instead of
    * recording a distance associated with {@code E}'s frame marker - a boolean
    * value is recorded, which is {@code true} if, and only if,
    * {@code E.getValue()} is no greater than {@code contactThreshold}.
    * 
    * @param distanceMappingList
    * a list of entries, one per frame marker, which records the minimum
    * distance between said frame marker and the tongue
    * @param contactThreshold
    * the threshold distance by which a frame may be considered in contact with
    * the tongue
    * @return a list of entries, one per frame marker, which records whether
    * said frame marker is considered to be in contact with the tongue
    */
   public static ArrayList<SimpleEntry<FrameMarker,Boolean>> generateContactMappingList (
      ArrayList<SimpleEntry<FrameMarker,Double>> distanceMappingList,
      double contactThreshold) {
      ArrayList<SimpleEntry<FrameMarker,Boolean>> toRet =
         new ArrayList<SimpleEntry<FrameMarker,Boolean>> (
            distanceMappingList.size ());
      for (SimpleEntry<FrameMarker,Double> entry : distanceMappingList) {
         toRet.add (
            new SimpleEntry<FrameMarker,Boolean> (
               entry.getKey (), entry.getValue () <= contactThreshold));
      }
      return toRet;
   }

   @Override
   public void apply (double t0, double t1) {
      frameMarkersDistanceMap ();
   }

   protected void concealPreviousRenderTypeRendering () {
      // Revert oral cavity mesh back to texture rendering
      myRoot.getOralCavity ().getMesh ().clearColors();
      myRoot.getOralCavity ().getMesh ().setFixed (true);

      // Hide the node-nearest markers
      RenderProps.setVisible (myNodeMarker, false);
      RenderProps.setVisible (myNodeNearestFrameMarker, false);

      // Remove highlight from all oral cavity frame markers
      for (FrameMarker mkr : myRoot.getFrontOralCavityMarkers ()) {
         mkr.getRenderProps ().setPointColor (
            BadinJawHyoidTongueContact.FRONT_MARKERS_COLOR);
      }
      for (FrameMarker mkr : myRoot.getMidOralCavityMarkers ()) {
         mkr.getRenderProps ().setPointColor (
            BadinJawHyoidTongueContact.MID_MARKERS_COLOR);
      }
      for (FrameMarker mkr : myRoot.getBackOralCavityMarkers ()) {
         mkr.getRenderProps ().setPointColor (
            BadinJawHyoidTongueContact.BACK_MARKERS_COLOR);
      }
      for (FrameMarker mkr : myRoot.getLateralLeftOralCavityMarkers ()) {
         mkr.getRenderProps ().setPointColor (
            BadinJawHyoidTongueContact.LATERAL_MARKERS_COLOR);
      }
      for (FrameMarker mkr : myRoot.getLateralRightOralCavityMarkers ()) {
         mkr.getRenderProps ().setPointColor (
            BadinJawHyoidTongueContact.LATERAL_MARKERS_COLOR);
      }
   }

   protected void oralCavitySurfaceDistanceMap () {
      PolygonalMesh meshOC = myRoot.getOralCavity ().getMesh ();
      PolygonalMesh meshTongue = myRoot.getTongue ().getSurfaceMesh ();

      // Find the range of distances given the current tongue position
      double smallest = Double.POSITIVE_INFINITY;
      double largest = 0;
      for (Vertex3d v : meshOC.getVertices ()) {
         Vertex3d near =
            myQuery.nearestVertexToPoint (meshTongue, v.getPosition ());
         double dist = v.getPosition ().distance (near.getPosition ());
         if (dist < smallest) {
            smallest = dist;
         }
         if (dist > largest) {
            largest = dist;
         }
      }
      double range = largest - smallest + /* EPSILON= */0.0001;

      meshOC.setVertexColoringEnabled ();
      // Color all vertices according to distance relative to range
      for (Vertex3d v : meshOC.getVertices ()) {
         Vertex3d near =
            myQuery.nearestVertexToPoint (meshTongue, v.getPosition ());
         double dist = v.getPosition ().distance (near.getPosition ());
         double colorValue = (dist - smallest) / range;
         meshOC.setColor (v.getIndex (), myColors.getColor (colorValue));
      }

      meshOC.setFixed (false);
   }

   protected void nodeNearestPoint () {
      Point3d pnt = new Point3d ();
      Point3d nearPnt = new Point3d ();
      myNodeMarker.getPosition (pnt);
      myQuery.nearestFaceToPoint (
         nearPnt, null, myRoot.getOralCavity ().getMesh (), pnt);
      myNodeNearestFrameMarker.setPosition (nearPnt);

      if (!myNodeMarker.getRenderProps ().isVisible ()) {
         RenderProps.setVisible (myNodeMarker, true);
      }
      if (!myNodeNearestFrameMarker.getRenderProps ().isVisible ()) {
         RenderProps.setVisible (myNodeNearestFrameMarker, true);
      }
   }

   protected void frameMarkersDistanceMap () {
      for (FrameMarker mkr : myRoot.getFrontOralCavityMarkers ()) {
         update (mkr, BadinJawHyoidTongueContact.FRONT_MARKERS_COLOR);
      }
      for (FrameMarker mkr : myRoot.getMidOralCavityMarkers ()) {
         update (mkr, BadinJawHyoidTongueContact.MID_MARKERS_COLOR);
      }
      for (FrameMarker mkr : myRoot.getBackOralCavityMarkers ()) {
         update (mkr, BadinJawHyoidTongueContact.BACK_MARKERS_COLOR);
      }
      for (FrameMarker mkr : myRoot.getLateralLeftOralCavityMarkers ()) {
         update (mkr, BadinJawHyoidTongueContact.LATERAL_MARKERS_COLOR);
      }
      for (FrameMarker mkr : myRoot.getLateralRightOralCavityMarkers ()) {
         update (mkr, BadinJawHyoidTongueContact.LATERAL_MARKERS_COLOR);
      }
      
//      double min = 1000000;
//      for (SimpleEntry<FrameMarker,Double> entry : myDistanceMappingList) {
//         if (entry.getValue () < min) {
//            min = entry.getValue ();
//         }
//      }
//      System.out.println(min);
   }

   protected void update (FrameMarker mkr, Color color) {
      double d = distanceFromTongue (mkr);
      updateMarkerColor (mkr, d, color);
      updateMyDistanceMappingList (mkr, d);
   }

   protected double distanceFromTongue (FrameMarker mkr) {
      Point3d nearPnt = new Point3d ();
      myQuery.nearestFaceToPoint (
         nearPnt, null, myRoot.getTongue ().getSurfaceMesh (),
         mkr.getPosition ());
      return mkr.getPosition ().distance (nearPnt);
   }

   protected void updateMyDistanceMappingList (FrameMarker mkr, double newDistance) {
      for (SimpleEntry<FrameMarker,Double> entry : myDistanceMappingList) {
         if (entry.getKey().getName().equals(mkr.getName())) {
            entry.setValue(newDistance);
         }
      }
   }

   protected void updateMarkerColor (
      FrameMarker mkr, double distance, Color noContactColor) {
      if (distance <= CONTACT_THRESHOLD) {
         Color contactColor = noContactColor.brighter ().brighter ();
         if (!contactColor.equals (mkr.getRenderProps ().getPointColor ())) {
            mkr.getRenderProps ().setPointColor (contactColor);
         }
      }
      else {
         if (!noContactColor.equals (mkr.getRenderProps ().getPointColor ())) {
            mkr.getRenderProps ().setPointColor (noContactColor);
         }
      }
   }
}

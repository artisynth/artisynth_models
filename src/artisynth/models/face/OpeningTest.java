package artisynth.models.face;

import java.awt.Color;
import java.awt.Rectangle;
import java.awt.Robot;
import java.awt.image.BufferedImage;
import java.awt.image.BufferedImageOp;
import java.awt.image.LookupOp;
import java.awt.image.ShortLookupTable;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import javax.imageio.ImageIO;

import artisynth.core.driver.Main;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.modelbase.MonitorBase;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.AxisAlignedRotation;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.GL.GLViewer;

public class OpeningTest extends RootModel {
   
   Color openingColor = new Color (0f,1f,0f);


   @Override
   public void build (String[] args) throws IOException {
      super.build (args);
      
      MechModel mech = new MechModel("mech");
      addModel (mech);
      
      MeshComponent torus = new MeshComponent ();
      torus.setMesh (MeshFactory.createTorus (4,  2, 128, 128));
      RenderProps.setFaceColor (torus, Color.RED);

      
      MeshComponent plane = new MeshComponent ();
      plane.setMesh (MeshFactory.createPlane (4,4));
      RenderProps.setFaceColor (plane, openingColor);
      
      mech.add (torus);
      mech.add (plane);
      
      addMonitor (new OpeningMeasurer ());
      
   }
   
   private static Rectangle getViewerBounds(GLViewer viewer) {
      Rectangle area = new Rectangle();
      area.width = viewer.getCanvas().getWidth();
      area.height = viewer.getCanvas().getHeight();
      area.x = viewer.getCanvas().getLocationOnScreen().x;
      area.y = viewer.getCanvas().getLocationOnScreen().y;
      return area;
   }


   /**
    * Method to get the distance between this pixel's color and the passed color
    * @param testColor the color to compare to
    * @return the distance between this pixel's color and the passed color
    */
   public double colorDistance(Color testColor, Color color)
   {
      float[] c = new float[3];
      float[] t = new float[3];
      color.getColorComponents (c);
      testColor.getColorComponents (t);
      
      return Math.sqrt (
         (c[0]-t[0])*(c[0]-t[0])+
         (c[1]-t[1])*(c[1]-t[1])+
         (c[2]-t[2])*(c[2]-t[02]));
   }


   public synchronized void grabScreenShot () {
      BufferedImage screenshot = null;
      ArrayList<Color> pixelColors = new ArrayList<> ();
      ArrayList<Integer> pixelCnts = new ArrayList<> ();
      try {
         screenshot = (new Robot()).
            createScreenCapture(getViewerBounds(getMainViewer ()));
         System.out.println("total pixels = "+screenshot.getWidth ()*screenshot.getHeight ());
        
         int cnt = 0;
         for (int i = 0; i < screenshot.getWidth (); i++) {
            for (int j = 0; j < screenshot.getHeight (); j++) {
               Color pixelColor = new Color(screenshot.getRGB (i, j));
               if (pixelColor.getGreen ()>200) {
                  if (!pixelColors.contains (pixelColor)) {
               
                  pixelColors.add (pixelColor);
                  pixelCnts.add (new Integer (0));
                  }
               
               int idx = pixelColors.indexOf (pixelColor);
               int c = pixelCnts.get (idx) + 1;
               pixelCnts.set (idx, c); 
//               cnt++;
               }
               
               if (colorDistance (openingColor, pixelColor) < 0.5) {
                  cnt++;
               }
            }
         }
         System.out.println("opening pixels = "+cnt);
         for (int i = 0; i < pixelColors.size (); i++) {
            System.out.println("pixel color = "+pixelColors.get (i)+" "+pixelCnts.get (i));
         }
         System.out.println("opening color = "+openingColor+" matching colors = "+cnt);
         double distPerPixel = getMainViewer().distancePerPixel (Vector3d.ZERO);
         double openingArea = distPerPixel*distPerPixel*cnt;
         System.out.println("dist-per-pixel = "+distPerPixel+"; opening area = "+openingArea);
         
         System.out.println("----");
         System.out.println("----");
         System.out.println("----");

         
      } catch (Exception e) {
         e.printStackTrace();
      }
            
//      short[] threshold = new short[256];
//      for (int i = 0; i < 256; i++)
//         threshold[i] = (i < 128) ? (short)0 : (short)255;
//         BufferedImageOp thresholdOp =
//            new LookupOp(new ShortLookupTable(0, threshold), null);
//         BufferedImage destination = thresholdOp.filter(screenshot, null);
      
         try {
            ImageIO.write(screenshot, "png", new File("test.png"));
         }
         catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
         }
   }


   
   public void attach (DriverInterface di) {
      getMainViewer().setOrthographicView(true);
      getMainFrame ().setSize (553, 613); // 1000 x 1000 viewer 
      getMainViewer().setAxialView (AxisAlignedRotation.X_Y);
//      getMainViewer().setView
      setViewerCenter (new Point3d(0, 0, 0));
      setViewerEye (new Point3d(0, 0, 10));
      getMainViewer ().zoom (1);
   }

   public class OpeningMeasurer extends MonitorBase {

      @Override
      public void apply (double t0, double t1) {
         measureArea ();
      }
      
      public synchronized double measureArea () {
         double openingArea = -1;
         BufferedImage screenshot = null;
         ArrayList<Color> pixelColors = new ArrayList<> ();
         ArrayList<Integer> pixelCnts = new ArrayList<> ();
         try {
            screenshot = (new Robot()).
               createScreenCapture(getViewerBounds(getMainViewer ()));
            System.out.println("total pixels = "+screenshot.getWidth ()*screenshot.getHeight ());
           
            int cnt = 0;
            for (int i = 0; i < screenshot.getWidth (); i++) {
               for (int j = 0; j < screenshot.getHeight (); j++) {
                  Color pixelColor = new Color(screenshot.getRGB (i, j));
                  if (pixelColor.getGreen ()>200) {
                     if (!pixelColors.contains (pixelColor)) {
                  
                     pixelColors.add (pixelColor);
                     pixelCnts.add (new Integer (0));
                     }
                  
                  int idx = pixelColors.indexOf (pixelColor);
                  int c = pixelCnts.get (idx) + 1;
                  pixelCnts.set (idx, c); 
//                  cnt++;
                  }
                  
                  if (colorDistance (openingColor, pixelColor) < 0.5) {
                     cnt++;
                  }
               }
            }
            System.out.println("opening pixels = "+cnt);
            for (int i = 0; i < pixelColors.size (); i++) {
               System.out.println("pixel color = "+pixelColors.get (i)+" "+pixelCnts.get (i));
            }
            System.out.println("opening color = "+openingColor+" matching colors = "+cnt);
            double distPerPixel = getMainViewer().distancePerPixel (Vector3d.ZERO);
            openingArea = distPerPixel*distPerPixel*cnt;
            System.out.println("dist-per-pixel = "+distPerPixel+"; opening area = "+openingArea);
            
            System.out.println("----");
            System.out.println("----");
            System.out.println("----");

            
         } catch (Exception e) {
            e.printStackTrace();
         }
               
//         short[] threshold = new short[256];
//         for (int i = 0; i < 256; i++)
//            threshold[i] = (i < 128) ? (short)0 : (short)255;
//            BufferedImageOp thresholdOp =
//               new LookupOp(new ShortLookupTable(0, threshold), null);
//            BufferedImage destination = thresholdOp.filter(screenshot, null);
         
            try {
               ImageIO.write(screenshot, "png", new File("test.png"));
            }
            catch (IOException e) {
               // TODO Auto-generated catch block
               e.printStackTrace();
            }
            return openingArea;
      }

   }
   
}




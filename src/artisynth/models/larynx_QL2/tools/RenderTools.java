package artisynth.models.larynx_QL2.tools;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Toolkit;
import java.util.HashMap;

import com.jogamp.opengl.GLAutoDrawable;
import com.jogamp.opengl.GLEventListener;

import com.jogamp.graph.curve.opengl.RenderState;

import artisynth.core.driver.Main;
import artisynth.core.driver.ViewerManager;
import artisynth.core.femmodels.FemMeshComp;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.femmodels.MuscleElementDesc;
import artisynth.core.gui.Timeline;
import artisynth.core.inverse.TrackingController;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.CollisionManager;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MechSystemModel;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.Particle;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.Controller;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.RenderableComponentBase;
import artisynth.models.larynx_QL2.VocalTractBase;
import artisynth.models.larynx_QL2.components.AbstractBody;
import artisynth.models.larynx_QL2.components.Diarthrotic;
import artisynth.models.larynx_QL2.components.HardBody;
import artisynth.models.larynx_QL2.components.Ligamentous;
import artisynth.models.larynx_QL2.components.PointBody;
import artisynth.models.larynx_QL2.components.SoftBody;
import artisynth.models.larynx_QL2.components.Structures;
import artisynth.models.larynx_QL2.components.Tissue;
import com.jogamp.graph.curve.opengl.TextRegionUtil;
//import com.jogamp.graph.curve.opengl.TextRendererImpl01;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;
import maspack.properties.PropertyMode;
import maspack.render.Dragger3d.DraggerType;
import maspack.render.FaceRenderProps;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.Renderer.LineStyle;
import maspack.render.Renderer.PointStyle;
import maspack.render.GL.GLClipPlane;
import maspack.render.GL.GLViewer;
import maspack.render.GL.GL3.GL3Viewer;
/**
 *  <p>
 *   @author Scott Moisik (Scott.Moisik@ntu.edu.sg), 2017, Nanyang Technological University, Singapore. 
 *   <p>
 *   If you use this model in your research, please cite: 
 *   Moisik, S. R., & Gick, B. (2017). The quantal larynx: the stable regions of laryngeal biomechanics and implications for speech production. Journal of Speech Language and Hearing Research, 60(3), 540-560. https://doi.org/10.1044/2016_JSLHR-S-16-0019

 *  **/
public class RenderTools {

               public static void setGlobalVisibility(MechModel mech, boolean visibleFlag) {

                      for (FrameMarker fm : mech.frameMarkers()) {
                             RenderProps.setVisible(fm, visibleFlag);
                      }

                      for (RigidBody rb : mech.rigidBodies()) {
                             RenderProps.setVisible(rb, visibleFlag);
                      }

                      for (AxialSpring as : mech.axialSprings()) {
                             RenderProps.setVisible(as, visibleFlag);
                      }

                      for (Particle p : mech.particles()) {
                             RenderProps.setVisible(p, visibleFlag);
                      }

               }

               public static void centerCamera(Point3d point) {
                       ViewerManager vm = Main.getMain().getViewerManager();
                       for (int i = 0; i < vm.numViewers(); i++) {
                               GL3Viewer viewer = (GL3Viewer) vm.getViewer(i);
                               viewer.setCenter(point);
                       }
               }


               public static void centerCamera(FemMuscleModel fem) {
                       centerCamera(fem.getSurfaceMesh());
               }
               
               public static void centerCamera(RigidBody rb) {
                       centerCamera(rb.getMesh());
               }
               
               public static void centerCamera(AbstractBody body) {
                       if (body instanceof HardBody) {
                               centerCamera(((HardBody) body).getMesh());
                       }
                       else if (body instanceof SoftBody) {
                               centerCamera(((SoftBody) body).getMesh());
                       }
                       else if (body instanceof PointBody) {
                               centerCamera(((PointBody) body).getPosition());
                       }
               }
               
               public static void centerCamera(PolygonalMesh mesh) {
                      Point3d centroid = new Point3d();
                      mesh.computeCentroid(centroid);

                      Point3d pmin = new Point3d();
                      Point3d pmax = new Point3d();

                      mesh.updateBounds(pmin, pmax);

                      Vector3d vdiag = new Vector3d();
                      vdiag.sub (pmax, pmin);
                      double r = vdiag.norm() / 2;
                      double far = 40 * r;
                      double near = far / 1000;

                      ViewerManager vm = Main.getMain().getViewerManager();
                      RigidTransform3d TEW = new RigidTransform3d();
                      for (int i = 0; i < vm.numViewers(); i++) {
                             GL3Viewer viewer = (GL3Viewer) vm.getViewer(i);
                             viewer.setCenter(centroid);
                             Vector3d zdir = new Vector3d();
                             viewer.getEyeToWorld(TEW);
                             TEW.R.getColumn(2, zdir);

                             double d = r / Math.sin(Math.toRadians(viewer.getVerticalFieldOfView()) / 2);
                             TEW.p.scaledAdd(d, zdir, centroid);
                             viewer.setEyeToWorld(TEW);
                             viewer.setPerspective(viewer.getVerticalFieldOfView(), near, far);
                      }
               }


               public static double getModelExtent(MechModel mech) {
                      double extent = 0.0;
                      for (RigidBody rb : mech.rigidBodies()) {
                             if (rb.getMesh() != null) {
                                     double size = getCharacteristicBigSize(rb.getMesh());
                                     extent = (extent >= size ? extent : size);
                             }
                      }
                      
                      if (extent == 0.0) {
                              extent = 1.0;
                              System.err.println("A characteristic size based on rigid bodies could not be determined. A value of 1.0 will be used and this will affect the radius all rendered point objects.");
                      }

                      for (MechSystemModel model : mech.models()) {
                             if (model instanceof FemModel3d) {
                                    double size = getCharacteristicBigSize(((FemModel3d) model).getSurfaceMesh());
                                    extent = (extent >= size ? extent : size);
                             }
                      }

                      for (Particle p : mech.particles()) {
                             double size = p.getPosition().norm();
                             extent = (extent >= size ? extent : size);
                      }

                      return extent;
               }

               public static double getCharacteristicSmallSize(PolygonalMesh mesh) {
                      Vector3d min = new Vector3d();
                      Vector3d max = new Vector3d();
                      mesh.getWorldBounds(min, max);
                      Vector3d diff = new Vector3d(max);
                      diff.sub(min);
                      diff.absolute();
                      return (diff.x < diff.y ? (diff.x < diff.z ? diff.x : diff.z) : (diff.y < diff.z ? diff.y : diff.z)); 
               }

               public static double getCharacteristicBigSize(PolygonalMesh mesh) {
                      Vector3d min = new Vector3d();
                      Vector3d max = new Vector3d();
                      mesh.getWorldBounds(min, max);
                      Vector3d diff = new Vector3d(max);
                      diff.sub(min);
                      diff.absolute();
                      return (diff.x > diff.y ? (diff.x > diff.z ? diff.x : diff.z) : (diff.y > diff.z ? diff.y : diff.z)); 
               }
               
               public static void setupRenderProperties(FemMuscleModel fem, Color elementColor, Color nodeColor) {
                      double scale = getCharacteristicBigSize(fem.getSurfaceMesh())*5e-3;

                      //Setup render properties for this deformable body
                      RenderProps.setVisible(fem.getMuscleBundles(), true);
                      RenderProps.setVisible(fem.getNodes(), false);
                      RenderProps.setVisible(fem.getElements(), true);
                      RenderProps.setVisible(fem.markers(), false);
                      RenderProps.setLineWidth(fem, 1);
                      RenderProps.setPointRadius(fem.getNodes(), scale);
                      RenderProps.setPointStyle(fem.getNodes(), PointStyle.SPHERE);
                      RenderProps.setPointColor(fem.getNodes(), nodeColor);
                      fem.setDirectionRenderLen(scale*1000);
                      
                      fem.setElementWidgetSize(1.0);
                      fem.setSurfaceRendering(SurfaceRender.None);

                      for (FemMeshComp fm : fem.getMeshComps()) {
                              RenderProps.setVisible(fm, false);
                      }
                      
                      for (FemNode3d n : fem.getNodes()) {
                             RenderProps.setVisibleMode(n, PropertyMode.Inherited);
                      }

                      for (MuscleBundle mb : fem.getMuscleBundles()) {
                             for (Muscle fiber : mb.getFibres()) {
                                    RenderProps.setVisible(fiber, true);
                                    RenderProps.setLineStyle(fiber, LineStyle.SPINDLE);
                                    RenderProps.setLineColor(fiber, Color.red);
                                    RenderProps.setLineRadius(fiber, scale*0.5);
                             }
                             
                             for (MuscleElementDesc mec : mb.getElements()) {
                                     RenderProps.setVisible(mec, false);
                                     RenderProps.setFaceColor(mec, elementColor);
                             }
                      }

                      RenderProps.setFaceColor(fem, elementColor);
                      RenderProps.setLineWidth(fem.getElements(), 0);
                      RenderProps.setLineColor(fem, new Color(.2f, .2f, .2f));

               }

               public static void setupCollisionRenderProperties(MechModel mech) {
                      CollisionManager collisions = mech.getCollisionManager();
                      collisions.setContactNormalLen(-0.02);
                      collisions.setDrawContactNormals(true);
                      RenderProps.setLineColor(collisions, new Color(0.2f, 0.2f, 0.5f));
                      RenderProps.setVisible(collisions, true);
               }

               /** Shifts the timeline to the right monitor and places it within the lower half of the screen space. **/
               public static void reshapeTimeLine() {

                      int h = Main.getMain().getMainFrame().getHeight();

                      Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();
                      Timeline timeLine = Main.getMain().getTimeline();
                      timeLine.setLocation(screenSize.width, screenSize.height - h);
                      timeLine.setSize(screenSize.width, 400);

               }

               /** Shifts the timeline to the right monitor and places it within the lower half of the screen space. **/
               public static void returnTimeLineToDefaultLocation() {

                      int h = Main.getMain().getMainFrame().getHeight();

                      Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();
                      Timeline timeLine = Main.getMain().getTimeline();
                      timeLine.setLocation(0, screenSize.height - h);

               }


               public static void setupGlobalRenderProperties(MechModel mech, double pointSize) {
                      final RenderProps props = mech.createRenderProps();

                      // Particle RenderProps
                      props.setPointRadius(pointSize);
                      props.setPointStyle(Renderer.PointStyle.SPHERE);
                      props.setPointColor(Color.PINK);

                      // Line RenderProps
                      props.setLineRadius(pointSize);
                      //props.setLineSlices(8);
                      props.setLineWidth(1);
                      props.setLineStyle(Renderer.LineStyle.LINE);
                      props.setLineColor(Color.WHITE);

                      // Mesh RenderProps
                      props.setShading(Renderer.Shading.FLAT);
                      props.setFaceColor(new Color(1f, 0.8f, 0.6f));
                      props.setFaceStyle(Renderer.FaceStyle.FRONT_AND_BACK);

                      mech.setRenderProps(props);

                      //Scale components to characteristic dimensions
                      for (Tissue tissue : Structures.getTissues()) {
                              if (tissue instanceof Ligamentous) {
                                      tissue.setRenderRadius(pointSize*0.5);
                              }
                              else if (tissue instanceof Diarthrotic) {
                                      ((Diarthrotic) tissue).setRenderProperties(pointSize*2, new Color(192, 192, 192));
                              }
                              else {
                                      tissue.setRenderRadius(pointSize);
                              }
                             
                      }

                      for (RigidBody body : mech.rigidBodies()) {
                             RenderProps.setFaceColor(body, new Color(0.55f, 0.55f, 0.65f));
                             body.setAxisLength(0.0);
                      }

                      for (Particle particle : mech.particles()) {
                             RenderProps.setPointRadius(particle, pointSize);
                      }

                      for (MechSystemModel msm : mech.models()) {
                             if (msm instanceof FemMuscleModel) {
                                    RenderProps.setPointRadius(((FemMuscleModel) msm).getNodes(), pointSize*0.75);

                                    for (MuscleBundle mb : ((FemMuscleModel) msm).getMuscleBundles()) {
                                           for (Muscle m : mb.getFibres()) {
                                                  RenderProps.setLineRadius((AxialSpring) m, pointSize*0.5);
                                           }
                                    }
                             }
                      }
               }

               public static void setTargetPointSize(VocalTractBase vtBase) {
                      //Set render properties for target points (if any exist)
                      for (Controller tc : vtBase.getControllers()) {
                             if (tc instanceof TrackingController) {
                                    for (ModelComponent tp : ((TrackingController) tc).getMotionTargetTerm().getTargets()) {
                                           if (tp instanceof Point) {
                                                  RenderProps.setPointRadius((Point) tp, vtBase.characteristicPointSize);
                                           }
                                    }   
                             }
                      }


               }
        public static class RenderPlane {

                public static enum PlaneOrientation {
                        FRONTAL_ANTERIOR(new Vector3d(1.0, 0.0, 0.0), Math.PI),
                        FRONTAL_POSTERIOR(new Vector3d(1.0, 0.0, 0.0), 0.0),
                        MIDSAGITTAL_LEFT(new Vector3d(0.0, 1.0, 0.0), Math.PI*0.5),
                        MIDSAGITTAL_RIGHT(new Vector3d(0.0, 1.0, 0.0), -Math.PI*0.5),
                        AXIAL_SUPERIOR(new Vector3d(1.0, 0.0, 0.0), Math.PI*0.5),
                        AXIAL_INFERIOR(new Vector3d(1.0, 0.0, 0.0), -Math.PI*0.5);

                        double angle;
                        Vector3d axis;

                        PlaneOrientation(Vector3d axis, double angle) {
                                this.angle = angle;
                                this.axis = axis;
                        }
                }

                public static enum PlaneType {
                        CLIP,
                        SLICE,
                        GRID,
                        CLIPGRID,
                        SLICEGRID;
                }

                public static void createBasicSlicePlane(MechModel mech, PlaneOrientation orientation, boolean showDragger) {
                        RenderPlane.createPlane(PlaneType.SLICE, (GL3Viewer) Main.getMain().getViewer(), new Point3d(), orientation, getModelExtent(mech), showDragger);
                }
                
                public static void createBasicClipPlane(MechModel mech, PlaneOrientation orientation, boolean showDragger) {
                        RenderPlane.createPlane(PlaneType.CLIP, (GL3Viewer) Main.getMain().getViewer(), new Point3d(), orientation, getModelExtent(mech), showDragger);
                }

                public static void createBasicClipPlane(MechModel mech, PlaneOrientation orientation) {
                        RenderPlane.createPlane(PlaneType.CLIP, (GL3Viewer) Main.getMain().getViewer(), new Point3d(), orientation, getModelExtent(mech), true);
                }

                public static void createPlane(PlaneType type, GL3Viewer viewer, Point3d position, PlaneOrientation orientation, double minSize, boolean useDraggerFlag) {
                        createPlane(type, viewer, position, orientation.axis, orientation.angle, minSize, useDraggerFlag);
                }

                /** Adds angle to the default orientation axis-angle representation for the specified orientation plane. **/
                public static void createPlane(PlaneType type, GL3Viewer viewer, Point3d position, PlaneOrientation orientation, double angle, double minSize, boolean useDraggerFlag) {
                        createPlane(type, viewer, position, orientation.axis, orientation.angle + angle, minSize, useDraggerFlag);
                }

                public static void createPlane(PlaneType type, GL3Viewer viewer, Point3d position, Vector3d axis, double angle, double minSize, boolean useDraggerFlag) {
                        GLClipPlane plane = new GLClipPlane();
                        viewer.addClipPlane(plane);
                        plane.setOrientation(new AxisAngle(axis, angle));
                        plane.setPosition(position);
                        plane.setDragger((useDraggerFlag ? DraggerType.Transrotator : DraggerType.None));
                        plane.setMinSize(minSize);

                        switch (type) {
                        case CLIP:
                                plane.setClippingEnabled(true);
                                plane.setGridVisible(false);
                                break;
                        case CLIPGRID:
                                plane.setClippingEnabled(true);
                                plane.setGridVisible(true);
                                break;
                        case GRID:
                                plane.setGridVisible(true);
                                break;
                        case SLICE:
                                plane.setClippingEnabled(false);
                                plane.setSlicingEnabled(true);
                                plane.setGridVisible(false);
                                break;
                        case SLICEGRID:
                                plane.setClippingEnabled(false);
                                plane.setSlicingEnabled(true);
                                plane.setGridVisible(true);
                                break;
                        default:
                                break;

                        }

                        viewer.rerender();

                }
        }


        /** Creates an openGL visualization of a set of instructions. The instructions must be added in order starting from last to first to ensure that the first instruction will appear at the top. Each instruction must be associated with a unique keyword. **/ 
        public static class Instructions {
                public GL3Viewer viewer;
                public int fontSize = 12;
                public int xPosition = 10;
                public int yPosition = 10;
                public int yOffset = 15;
                public HashMap<String, InstructionLine> instructions = new HashMap<String, InstructionLine>();
                public Instructions(GL3Viewer viewer) {
                        this.viewer = viewer;
                }
                public Instructions(GL3Viewer viewer, int fontSize, int xPosition, int yPosition, int yOffset) {
                        this(viewer);
                        this.fontSize = fontSize;
                        this.xPosition = xPosition;
                        this.yPosition = yPosition;
                        this.yOffset = yOffset;
                }
                
                public void add(String keyword, String fixedInstruction, String variableInstruction) {
                        if (!instructions.containsKey(keyword)) {
                                instructions.put(keyword, new InstructionLine(viewer, fixedInstruction, variableInstruction, fontSize, xPosition, yPosition += yOffset));       
                        }
                        else {
                                throw new IllegalArgumentException("Instruction keywords must be unique");
                        }
                }
                
                public void change(String keyword, String variableInstruction) {
                       InstructionLine il = instructions.get(keyword);
                       if (il != null) {
                              il.update(variableInstruction);
                       }
                       else {
                              System.err.println("Instruction line with keyword '" + keyword + "' not found.");
                       }
                }
                
                public static class InstructionLine {
                        public TextFeeder2 textFeeder;
                        public String fixedInstruction;
                        public String variableInstruction;
                        
                        public InstructionLine(GL3Viewer viewer, String fixedInstruction, String variableInstruction, int fontSize, int xPosition, int yPosition) {
                                this.fixedInstruction = fixedInstruction;
                                this.variableInstruction = variableInstruction;
                                textFeeder = new TextFeeder2(fixedInstruction + variableInstruction, fontSize, xPosition, yPosition);
                                
                        }
                        public void update(String newVariableInstruction) {
                                variableInstruction = newVariableInstruction;
                                textFeeder.setText(fixedInstruction + variableInstruction);
                        }
                }
        }
        
//         public class TextWrapper extends TextRendererImpl01 {
// 
//                 public TextWrapper(RenderState arg0, int arg1) {
//                         super(arg0, arg1);
//                         // TODO Auto-generated constructor stub
//                 }
//                 
//         }
        
        /*
        public static class TextFeeder implements GLEventListener {
                private String text = "";
                private Color textColor = Color.white;
                private int x, y;
                private Font font;
                private TextFeeder2 textRenderer;

                public TextFeeder(GL3Viewer viewer, String text, int x, int y) { this(viewer, text, Color.white, "SansSerif", Font.BOLD, 12, x, y); }
                public TextFeeder(GL3Viewer glViewer, String text, int fontSize, int x, int y) { this(glViewer, text, Color.white, "SansSerif", Font.BOLD, fontSize, x, y); }
                public TextFeeder(GL3Viewer glViewer, String text, Color textColor, String fontType, int fontStyle, int fontSize, int x, int y) {this(glViewer, text, new Font("SansSerif", fontStyle, fontSize), textColor, x, y); }
                public TextFeeder(GL3Viewer viewer, String text, Font font, Color textColor, int x, int y) { 
                        this.text = text;
                        this.textColor = textColor;
                        this.x = x;
                        this.y = y;
                        //this.font = new Font("SansSerif", Font.BOLD, 512); //font;
                        
                        textRenderer = new TextFeeder2(font);
                        
                        viewer.getCanvas().addGLEventListener(this);
                }

                public void setLocation(int x, int y) { this.x = x; this.y = y; }
                public void setText(String text) { this.text = text; }
                public String getText() { return text; }

                @Override
                public void display(GLAutoDrawable drawable) {

                                //textRenderer.begin(drawable.getGL());
                                //textRenderer.drawText(font, text, new float[]{(float) 0.0, (float) 0.0, (float) 0.0}, (float) 0.001);
                                //textRenderer.end(drawable.getGL());
                                //int i = 1;
                                
                        textRenderer.beginRendering(drawable.getWidth(), drawable.getHeight());
                        textRenderer.setColor(textColor);
                        textRenderer.draw(text, x, y);
                        textRenderer.endRendering();
                        

                }

                @Override
                public void init(GLAutoDrawable drawable) {}
                @Override
                public void reshape(GLAutoDrawable arg0, int arg1, int arg2, int arg3, int arg4) {}
                @Override
                public void dispose(GLAutoDrawable arg0) {}
        }
	*/
        
        public static class TextFeeder2 extends RenderableComponentBase {
           private String text = "";
           private Color textColor = Color.white;
           private int x, y;
           private float em;
           private Font font;

           public static PropertyList myProps =
           new PropertyList (TextFeeder2.class, RenderableComponentBase.class);
           static {
              myProps.add ("renderProps", "render properties", null);
           }

           public PropertyList getAllPropertyInfo() {
              return myProps;
           }

           public TextFeeder2(String text, int x, int y) { this(text, Color.white, "SansSerif", Font.BOLD, 12, x, y); }
           public TextFeeder2(String text, int fontSize, int x, int y) { this(text, Color.white, "SansSerif", Font.BOLD, fontSize, x, y); }
           public TextFeeder2(String text, Color textColor, String fontType, int fontStyle, int fontSize, int x, int y) {this(text, new Font("SansSerif", fontStyle, fontSize), fontSize, textColor, x, y); }
           public TextFeeder2(String text, Font font, float emSize, Color textColor, int x, int y) { 
              this.text = text;
              this.textColor = textColor;
              this.x = x;
              this.y = y;
              this.em = emSize;
              this.font = font;
              //this.font = new Font("SansSerif", Font.BOLD, 512); //font;
              RenderProps.setVisible(this, true);
              VocalTractBase.getInstance().scheduleRenderableAdd(this);
           }

           @Override
           public RenderProps createRenderProps() {
              return new FaceRenderProps();
           }

           public void setLocation(int x, int y) { this.x = x; this.y = y; }
           public void setText(String text) { this.text = text; }
           public String getText() { return text; }

           @Override
           public void render(Renderer renderer, int flags) {
              renderer.pushModelMatrix();
              renderer.setModelMatrix2d(0, renderer.getScreenWidth(), 0, renderer.getScreenHeight());
              float[] pos = new float[]{x,y,0};
              renderer.setColor(textColor);
              renderer.drawText(font, text, pos, em);
              renderer.popModelMatrix();
           }
           
           @Override
           public int getRenderHints() {
              return TRANSPARENT | TWO_DIMENSIONAL;
           }
        }

}

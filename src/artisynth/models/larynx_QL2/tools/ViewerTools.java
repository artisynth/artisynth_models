package artisynth.models.larynx_QL2.tools;

import java.awt.Component;
import java.awt.Dimension;
import java.awt.GraphicsDevice;
import java.awt.GraphicsEnvironment;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.Point;
import java.awt.Rectangle;
import java.awt.Robot;
import java.awt.event.ComponentEvent;
import java.awt.event.ComponentListener;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.awt.image.BufferedImage;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;

import javax.imageio.ImageIO;
import javax.media.opengl.GL2;
import javax.media.opengl.GL3;
import javax.media.opengl.GLAutoDrawable;
import javax.media.opengl.GLEventListener;
import javax.swing.BoxLayout;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JSplitPane;
import javax.swing.JToolBar;

import com.jogamp.common.nio.Buffers;
import com.jogamp.opengl.FBObject;

import maspack.geometry.PolygonalMesh;
import maspack.matrix.AxisAlignedRotation;
import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.RotationMatrix3d;
import maspack.render.GL.GLViewer;
import maspack.render.GL.GL2.GL2Viewer;
import maspack.render.GL.GL3.GL3Viewer;
import maspack.render.GL.GLViewerFrame;
import maspack.render.GL.GLViewerPanel;
import maspack.render.GL.GL3.GL3Viewer;
import maspack.widgets.ViewerToolBar;
import artisynth.core.driver.Main;
import artisynth.core.driver.SchedulerListener;
import artisynth.core.driver.ViewerManager;
import artisynth.core.driver.Scheduler.Action;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.moviemaker.MakeMovieFromData;
import artisynth.core.moviemaker.MovieMaker;
import artisynth.core.workspace.RenderProbe;
import artisynth.core.workspace.RootModel;
import artisynth.models.larynx_QL2.VocalTractBase;
import artisynth.models.larynx_QL2.components.Names.Files;
import artisynth.models.larynx_QL2.tools.AuxTools.Timer;
import artisynth.models.larynx_QL2.tools.FemTools.ForgedFem;

/**
 *  <p>
 *   @author Scott Moisik (Scott.Moisik@ntu.edu.sg), 2017, Nanyang Technological University, Singapore. 
 *   <p>
 *   If you use this model in your research, please cite: 
 *   Moisik, S. R., & Gick, B. (2017). The quantal larynx: the stable regions of laryngeal biomechanics and implications for speech production. Journal of Speech Language and Hearing Research, 60(3), 540–560. https://doi.org/10.1044/2016_JSLHR-S-16-0019

 *  **/
public class ViewerTools {

	public static void showOnScreen(int screen, JFrame frame) {
		GraphicsEnvironment ge = GraphicsEnvironment.getLocalGraphicsEnvironment();
		GraphicsDevice[] gd = ge.getScreenDevices();
		if( screen > -1 && screen < gd.length ) {
			frame.setLocation(gd[screen].getDefaultConfiguration().getBounds().x, frame.getY());
			//gd[screen].setFullScreenWindow(frame);
			frame.setAlwaysOnTop(true);
			frame.setExtendedState(frame.getExtendedState() | JFrame.MAXIMIZED_BOTH);
		} else if( gd.length > 0 ) {
			frame.setLocation(gd[0].getDefaultConfiguration().getBounds().x, frame.getY());
		} else {
			throw new RuntimeException( "No Screens Found" );
		}
	}
	
	public static class OffscreenRenderer extends RenderProbe {
		public ArrayList<ViewerHandler> viewerHandlers = new ArrayList<ViewerHandler>();

		public int width, height;
		public String filePath;
		public int frameCount = 0;
		public double frameRate;
		public boolean frameGrabEnabledFlag = true;

		public OffscreenRenderer(String filePath, int width, int height, ArrayList<GLViewer> viewers, double interval) {
			super(Main.getMain(), interval);
			this.frameRate = interval;
			this.filePath = filePath;

			for (GLViewer viewer : viewers) {
				ViewerHandler vh = new ViewerHandler(viewer);
				viewer.getCanvas().addGLEventListener(vh);
				viewerHandlers.add(vh);
			}

			this.width = width;
			this.height = height;

		}

		public void reset() {
			frameGrabEnabledFlag = true;
			frameCount = 0;
			for (ViewerHandler vh : viewerHandlers) {
				vh.images.clear();	
			}
		}

		public boolean isGrabOngoing() {
			for (ViewerHandler vh : viewerHandlers) {
				if (vh.isGrabbing) {
					return true;
				}
			}
			return false;
		}

		public BufferedImage grabImage() {
			if (frameGrabEnabledFlag) {
				if (Main.getMain() != null) {
					Main.getMain().rerender();
				}

				System.out.println ("Image maker: Grabbing image.");
				try {
					for (ViewerHandler vh : viewerHandlers) {
						vh.isGrabbing = true;
						vh.viewer.repaint();
					}
					
					//Wait for grabbing to complete
					while (isGrabOngoing()) {
						try {
							Thread.sleep(1000);
						}
						catch (Exception e){
						}
					}
					
					//Compose the image from the viewers
					BufferedImage frame = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);

					for (ViewerHandler vh : viewerHandlers) {
						int[] rgbData = vh.images.get(0).getRGB(0, 0, vh.width, vh.height, null, 0, vh.width);
						frame.setRGB(vh.x, vh.y, vh.width - 1, vh.height - 1, rgbData, 0, vh.width);
					}
					
					return frame;
				}
				catch (Exception e) {
					e.printStackTrace();
					System.out.println ("ERROR grabbing movie frame");
				}
			}
			return null;
		}
		
		public String[] assembleFrames() {
			//Wait for grabbing to complete
			while (isGrabOngoing()) {
				try {
					Thread.sleep(1000);
				}
				catch (Exception e){
				}
			}

			int leastFrames = 0;
			for (ViewerHandler vh : viewerHandlers) {
				if (leastFrames == 0 || vh.images.size() < leastFrames) {
					leastFrames = vh.images.size();
				}
			}
			
			ArrayList<String> fileNames = new ArrayList<String>();
			if ((viewerHandlers.size() > 0 ? (viewerHandlers.get(0).images.size() > 0) : false)) {
			       
				ArrayList<BufferedImage> movieFrames = new ArrayList<BufferedImage>();
				for (int i = 0; i < leastFrames; i++) {
					BufferedImage frame = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);

					for (ViewerHandler vh : viewerHandlers) {
						int[] rgbData = vh.images.get(i).getRGB(0, 0, vh.width, vh.height, null, 0, vh.width);
						frame.setRGB(vh.x, vh.y, vh.width - 1, vh.height - 1, rgbData, 0, vh.width);
					}

					movieFrames.add(frame);

					try {
						String fileName = filePath + "offscreen_frame_" + i + ".jpg";
						fileNames.add(fileName);
						ImageIO.write(frame, "JPG", new File(fileName));
					} catch (IOException e) {
						e.printStackTrace();
					}
				}

				//Create the file name array
				String[] fileNameArray = new String[fileNames.size()];
				for (int i = 0; i < fileNames.size(); i++) {
					fileNameArray[i] = fileNames.get(i);
				}

				//Write the info file required by the movie maker system
				try {
					BufferedWriter bw = new BufferedWriter(new FileWriter(filePath + "\\info.txt"));
					bw.write("Number of frames: " + String.valueOf(movieFrames.size()));
					bw.newLine();
					bw.write("Width: " + String.valueOf(width));
					bw.newLine();
					bw.write("Height: " + String.valueOf(height));
					bw.newLine();
					bw.write("Frame rate: " + String.valueOf(frameRate));
					bw.flush();
					bw.close();
				} catch (IOException e) {
					e.printStackTrace();
				} 

				return fileNameArray;
			}
			else {
				System.err.println("No frames were captured. Movie could not be made.");
			}
			return null;
		}

		public void apply(double t) {
			if (frameGrabEnabledFlag) {
				if (Main.getMain() != null) {
					Main.getMain().rerender();
				}

				frameCount++;
				System.out.println ("Movie maker: Grabbing frame #" + frameCount + " at t = " + String.format("%.2f", t) + " s");
				try {
					for (ViewerHandler vh : viewerHandlers) {
						vh.isGrabbing = true;
						vh.viewer.repaint();
					}
				}
				catch (Exception e) {
					e.printStackTrace();
					System.out.println ("ERROR grabbing movie frame");
				}
			}
		}

		public static class ViewerHandler implements GLEventListener {
			public ArrayList<BufferedImage> images = new ArrayList<BufferedImage>();
			public FBObject fbo;
			public GLViewer viewer;
			public int x, y, width, height;
			public boolean isGrabbing = false;

			public ViewerHandler(GLViewer viewer) {
				fbo = new FBObject();
				this.viewer = viewer;
				x = viewer.getCanvas().getX();
				y = viewer.getCanvas().getY();
				width = viewer.getCanvas().getWidth();
				height = viewer.getCanvas().getHeight();
			}

			@Override
			public void display(GLAutoDrawable drawable) {
				GL3 gl = drawable.getGL().getGL3();
				fbo.bind(gl);

				if (isGrabbing && viewer instanceof GL3Viewer) {
					int flags = viewer.getRenderFlags();
					((GL3Viewer)viewer).display(drawable, flags);

					images.add(capture(gl));
					isGrabbing = false;
				}
				fbo.unbind(gl);
			}

			@Override
			public void init(GLAutoDrawable drawable) {
				GL3 gl = drawable.getGL().getGL3();
				fbo.reset(gl, width, height);

				fbo.attachColorbuffer(gl, 0, false);
				fbo.attachRenderbuffer(gl, FBObject.Attachment.Type.DEPTH, -1);
			}

			@Override
			public void reshape(GLAutoDrawable arg0, int arg1, int arg2, int arg3, int arg4) {}
			@Override
			public void dispose(GLAutoDrawable drawable) {
				GL3 gl = drawable.getGL().getGL3();
				fbo.detachAll(gl);
				fbo.destroy(gl);
			}


			private BufferedImage capture(GL3 gl) {
				BufferedImage image = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);

				// Get the ARGB pixels as integers.
				int[] pixelsARGB = getPixelsARGB(gl);

				image.setRGB (0, 0, width, height, pixelsARGB, 0, width);
				return image;
			}

			/**
			 * Uses the current GL object to get the RGBA pixels as bytes
			 * and converts them to ARGB pixels as integers.
			 * 
			 * The RGBA bytes from OpenGL must be converted to the ARGB integers 
			 * that Java's ImageIO class is expecting. Also, the y-axis must be 
			 * flipped because OpengGl has the origin at the bottom left of a canvas
			 * and the Java AWT at the top left.
			 * 
			 * @param gl      the current GL object.
			 * @param width   the width of the canvas.
			 * @param height  the height of the canvas.
			 * @return  The ARGB pixels as integers.
			 */
			private int[] getPixelsARGB(GL3 gl) {
				// Get the canvas RGB pixels as bytes and set up counters.
				int size = width * height * 4; // 4 bytes per RGBA pixel
				ByteBuffer pixelsBGRA = Buffers.newDirectByteBuffer(size);
				gl.glReadPixels(0, 0, width, height, GL2.GL_BGRA, GL2.GL_UNSIGNED_BYTE, pixelsBGRA);

				int byteRow = width * height * 4;
				int currentByte = byteRow;
				int byteRowWidth = width * 4;

				int[] pixelsARGB = new int[width * height];
				
				float[] bkColor = new float[4];
				gl.glGetFloatv(GL2.GL_COLOR_CLEAR_VALUE, bkColor,0);
				float bkAlpha = bkColor[3];

				// Convert RGBA bytes to ARGB integers.
				for (int row = 0, currentInt = 0; row < height; ++row) {
					byteRow -= byteRowWidth;
					currentByte = byteRow;

					for (int column = 0; column < width; ++column) {
						int blue = pixelsBGRA.get (currentByte++);
						int green = pixelsBGRA.get (currentByte++);
						int red = pixelsBGRA.get (currentByte++);
						int alpha = pixelsBGRA.get (currentByte++);


						// Set alpha to be completely opaque or completely invisible.
						//(alpha != 0) ? 0xff000000 : 0x00000000;

					        // compute final dest alpha
					        float fAlpha = 1-(1-alpha/255f)*(1-bkAlpha);
					        alpha = (int)(fAlpha*bkAlpha*255);
					            
						// allow alpha to be anything
						pixelsARGB[currentInt++] = (alpha << 24)
								| ((red & 0x000000ff) << 16)
								| ((green & 0x000000ff) << 8)
								| (blue & 0x000000ff);
					}
				}

				return pixelsARGB;
			}
		}
	}

	public static JPanel getContentPane() {
		//Obtain the split pane which contains the main GL drawing context and a scroll pane for the model hierarchy
		for (Component comp : Main.getMain().getMainFrame().getContentPane().getComponents()) {
			if (comp instanceof JSplitPane) {
				return (JPanel) ((JSplitPane) comp).getRightComponent();
			}
		}

		System.err.println("Could not locate main content pane.");
		return null;
	}
	public static class Movie {
		private MovieMaker movieMaker = Main.getMain().getMovieMaker();
		private static int offset = 6;

		/** Creates a movie maker object that is controlled programmatically. **/
		public Movie(String outputDirectory) {

			//Add a resize listener for the main frame
			Main.getMain().getMainFrame().addComponentListener(new ComponentListener() {
				public void componentShown(ComponentEvent e) {}
				public void componentResized(ComponentEvent e) {
					setCanvasSize(movieMaker);
				}
				public void componentMoved(ComponentEvent e) {}
				public void componentHidden(ComponentEvent e) {}
			});
			movieMaker.setFormat("jpg");
			setCanvasSize(movieMaker);

			movieMaker.setDataPath(outputDirectory);
		}

		public MovieMaker getMovieMaker() { return movieMaker; }
		public static void takeScreenShot(String outputDirectory, String fileName) {
			try {
				Movie movie = new Movie(outputDirectory);
				ImageIO.write((new Robot()).createScreenCapture(movie.getMovieMaker().getCaptureArea()), Files.SCREENSHOT.getKeyword(), new File(movie.getMovieMaker().getDataPath() + fileName + ".png"));
			} catch (Exception e) {
				e.printStackTrace();
			} 
		}


		public static void addScreenShotSaver(final char triggerKey, final VocalTractBase vtBase, final String filePath) {
			final class ScreenShotCounter {
				int screenShotCount = 1;
			}

			final ScreenShotCounter ssc = new ScreenShotCounter();

			//Define an inline key adapter to handle custom keyboard input
			VocalTractBase.addCustomListener(new KeyAdapter() {
				public void keyTyped(KeyEvent e) {

					if (e.getKeyChar() == triggerKey) {
						Movie.takeScreenShot(filePath, "ScreenShot_" + ssc.screenShotCount++);
						System.out.println("Screen shot saved as " + filePath + "ScreenShot_" + ssc.screenShotCount + Files.SCREENSHOT.getSuffix());
					}
				}
			});

		}

		public static JPanel setCanvasSize(MovieMaker movieMaker) {
			JPanel canvas = getContentPane();

			if (canvas != null) {
				int width = canvas.getWidth();
				int height = canvas.getHeight();
				Point pos = canvas.getLocationOnScreen();

				Rectangle area = new Rectangle(pos.x + offset, pos.y, width - offset, height);
				movieMaker.setCaptureArea(area, null, false);//new Dimension(width, height), true);

			} else {
				System.err.println("Could not locate a suitable panel to render as frame in movie.");
			}

			return canvas;
		}

		public void grabFrame() { 
			if (!movieMaker.isGrabbing()) {
				movieMaker.setGrabbing(true);
			}

			try {
				movieMaker.grab();

			} catch (Exception e) {
				e.printStackTrace();
			}
		}

		public void finishMovie() {
			SimpleDateFormat dateformatJava = new SimpleDateFormat("dd-MM-yyyy");
			String dateString = dateformatJava.format(new Date());
			try {
				movieMaker.close();
				movieMaker.render("Movie_" + dateString);
				movieMaker.clean();
			} catch (Exception e) {
				e.printStackTrace();
			}
		}

		/** Creates a movie maker that becomes active when the user runs a simulation. Calling this method will produce a movie maker that does not stop grabbing frames until the simulation is stopped/paused by the user or hits a break point set elsewhere. <b>Currently, this must be applied prior to splitting the screen, otherwise the capture area will be incorrect.</b> *** 
		 *  **/
		public static void createScheduledMovieMaker(final RootModel root, String fileName, String outputDirectory, double frameRate) {
                   createScheduledMovieMaker(root, outputDirectory, fileName, 100.0, frameRate, Main.getMain().getMainFrame().getWidth(), Main.getMain().getMainFrame().getHeight(), true);
		}


		/** Creates a movie maker that becomes active when the user runs a simulation.  
		 *  **/
		public static void createScheduledMovieMaker(final RootModel root, final String outputDirectory, final String movieName, double stopTime, double frameRate) {
			createScheduledMovieMaker(root, outputDirectory, movieName, stopTime, frameRate, Main.getMain().getMainFrame().getWidth(), Main.getMain().getMainFrame().getHeight(), true);
		}

		/** Creates a movie maker that becomes active when the user runs a simulation.  
		 * @param movieName This is the file name used for the movie. If <b>null</b>, then a generic date-stamp name will be used.
		 * @param height 
		 * @param width **/
		public static void createScheduledMovieMaker(final RootModel root, final String outputDirectory, final String movieName, double stopTime, final double frameRate, int width, int height, final boolean cleanFilesFlag) {
			reshapeMainFrame(width, height);
			Main.getMain().getMainFrame().setResizable(false);
			root.addBreakPoint(stopTime);

			//Add a resize listener for the main frame
			Main.getMain().getMainFrame().addComponentListener(new ComponentListener() {
				public void componentShown(ComponentEvent e) {}
				public void componentResized(ComponentEvent e) {
					//setCanvasSize(movieMaker);
				}
				public void componentMoved(ComponentEvent e) {}
				public void componentHidden(ComponentEvent e) {}
			});

			//Remove any suffixed slashes as the movie maker class adds them
			final String finalizedOutputDirectory = (outputDirectory.endsWith("\\") ? outputDirectory.substring(0, outputDirectory.length() - 1) : outputDirectory); 

			JPanel panel = getContentPane();
			final OffscreenRenderer offscreenRenderer = initializeOffscreenRendering(panel.getWidth(), panel.getHeight(), finalizedOutputDirectory, frameRate);
			Main.getMain().getScheduler().setRenderProbe(offscreenRenderer);

			class Assistant {
				public boolean renderMovieFlag = true;
			}
			final Assistant assistant = new Assistant();

			Main.getMain().getScheduler().addListener(new SchedulerListener() {
				public void schedulerActionPerformed(Action action) {
					if (assistant.renderMovieFlag && (action == Action.Stopped || action == Action.Pause)) {
						assistant.renderMovieFlag = false;
						String[] fileNames = offscreenRenderer.assembleFrames();

						if (fileNames != null) {
							String finalMovieName;
							if (movieName == null) {
								SimpleDateFormat dateformatJava = new SimpleDateFormat("dd-MM-yyyy");
								String dateString = dateformatJava.format(new Date());
								finalMovieName = "Movie_" + dateString + Files.MOVIE.getSuffix();
							}
							else {
								finalMovieName = movieName + (!movieName.endsWith(Files.MOVIE.getSuffix()) ? Files.MOVIE.getSuffix() : "");
							}

							try {
								new MakeMovieFromData(fileNames, finalizedOutputDirectory, finalMovieName);
							} catch (Exception e) {
								e.printStackTrace();
							}

							Main.getMain().getMainFrame().setResizable(true);

							if (cleanFilesFlag) {
								for (int i = 0; i < fileNames.length; i++) {
									File tmpFile = new File(fileNames[i]);
									tmpFile.delete();
								}
							}
						}
					}
				}
			});
		}

		public static OffscreenRenderer initializeOffscreenRendering(int width, int height, String filePath, double frameRate) {
			ArrayList<GLViewer> viewers = new ArrayList<GLViewer>();
			for (int i = 0; i < Main.getMain().getViewerManager().numViewers(); i++) {
				viewers.add(Main.getMain().getViewerManager().getViewer(i));
			}

			Main.getMain().setFrameRate(frameRate);
			return new OffscreenRenderer(filePath, width, height, viewers, 1/Main.getMain().getFrameRate());
		}
	}

	public static enum ModelViews {
		FRONT(new Point3d(0.0, 0.0, -1.0), new Point3d(0.0, 0.0, 0.0)),
		BACK(new Point3d(0.0, 0.0, 1.0), new Point3d(0.0, 0.0, 0.0)),
		LEFT(new Point3d(-1.0, 0.0, 0.0), new Point3d(0.0, 0.0, 0.0)),
		RIGHT(new Point3d(1.0, 0.0, 0.0), new Point3d(0.0, 0.0, 0.0)),
		TOP(new Point3d(0.0, 1.0, 0.0), new Point3d(0.0, 0.0, 0.0)),
		BOTTOM(new Point3d(0.0, -1.0, 0.0), new Point3d(0.0, 0.0, 0.0)),

		OBLIQUE_LEFT_FRONT(new Point3d(-1.0, 0.0, -1.0), new Point3d(0.0, 0.0, 0.0)), 
		OBLIQUE_SUPERIOR_BACK(new Point3d(0.0, 1.0, 1.0), new Point3d(0.0, 0.0, 0.0));

		Point3d eye;
		Point3d center;
		ModelViews(Point3d eye, Point3d center) {
			eye.normalize();
			this.eye = eye;
			this.center = center;
		}
	}

	public static class CustomViewerLayout {

		private GLViewerPanel mainPanel;//, topSubPanel, bottomSubPanel;
		private JToolBar topToolBar, bottomToolBar;
		private GL3Viewer mainViewer, topSubViewer, bottomSubViewer;
		public JPanel toolBarPanel, contentPane;
		public Component[] defaultToolBars;


		CustomViewerLayout() {};

		public GL3Viewer getMainViewer() { return (GL3Viewer) mainPanel.getViewer(); }
		public GL3Viewer getTopSubViewer() { return topSubViewer; }
		public GL3Viewer getBottomSubViewer() { return bottomSubViewer; }

		/** Sets the center of the view to the origin (0.0, 0.0, 0.0) and sets the eye to the specified value with the specified zoom (smaller values move the eye closer to the orgin). **/
		public void setViews(ModelViews mainView, double mainZoom, Point3d mainCenter, ModelViews topSubView, double topZoom, Point3d topCenter, ModelViews bottomSubView, double bottomZoom, Point3d bottomCenter) {
			Point3d mainEye = mainView.eye;
			mainEye.scale(mainZoom);
			mainViewer.setEye(mainEye);
			mainViewer.setCenter(mainCenter);

			if (topSubViewer != null) {
				Point3d topEye = topSubView.eye;
				topEye.scale(topZoom);
				topSubViewer.setEye(topEye);
				topSubViewer.setCenter(topCenter);
			}

			if (bottomSubViewer != null) {
				Point3d bottomEye = bottomSubView.eye;
				bottomEye.scale(bottomZoom);
				bottomSubViewer.setEye(bottomSubView.eye);
				bottomSubViewer.setCenter(bottomCenter);
			}

			Main.getMain().getViewerManager().render();
		}

		/** Sets the center of the view to the origin (0.0, 0.0, 0.0) and sets the eye to the specified value with the specified zoom (smaller values move the eye closer to the orgin). **/
		public void setViews(ModelViews mainView, double mainZoom, ModelViews topSubView, double topZoom, ModelViews bottomSubView, double bottomZoom) {
			setViews(mainView, mainZoom, mainView.center, topSubView, topZoom, topSubView.center, bottomSubView, bottomZoom, bottomSubView.center);
		}

		public void setViews(ModelComponent focusComp, ModelViews mainView, double mainZoom, Point3d mainCenter, ModelViews topSubView, double topZoom, Point3d topCenter, ModelViews bottomSubView, double bottomZoom, Point3d bottomCenter) {
			setViews(mainView, mainZoom, topSubView, topZoom, bottomSubView, bottomZoom);

			Point3d center = new Point3d();
			if (focusComp instanceof FemMuscleModel) {
				((FemMuscleModel) focusComp).getSurfaceMesh().computeCentroid(center);
			}
			else if (focusComp instanceof RigidBody) {
				((RigidBody) focusComp).getMesh().computeCentroid(center);
			}
			else if (focusComp instanceof artisynth.core.mechmodels.Point) {
				center = ((artisynth.core.mechmodels.Point) focusComp).getPosition();
			}

			Point3d mainCenterAdjusted = new Point3d(mainCenter);
			mainCenterAdjusted.add(center);
			mainViewer.setCenter(mainCenterAdjusted);
			Point3d newEye = new Point3d(mainViewer.getEye());
			newEye.add(mainCenterAdjusted);
			mainViewer.setEye(newEye);

			if (topSubViewer != null) {
				Point3d topCenterAdjusted = new Point3d(topCenter);
				topCenterAdjusted.add(center);
				topSubViewer.setCenter(topCenterAdjusted);
				Point3d newTopEye = new Point3d(topSubViewer.getEye());
				newTopEye.add(topCenterAdjusted);
				topSubViewer.setEye(newTopEye);
			}
			if (bottomSubViewer != null) {
				Point3d bottomCenterAdjusted = new Point3d(bottomCenter);
				bottomCenterAdjusted.add(center);
				bottomSubViewer.setCenter(bottomCenterAdjusted);
				Point3d newBottomEye = new Point3d(bottomSubViewer.getEye());
				newBottomEye.add(bottomCenterAdjusted);
				bottomSubViewer.setEye(newBottomEye);
			}
		}


		public void setViews(ModelComponent focusComp, ModelViews mainView, double mainZoom, ModelViews topSubView, double topZoom, ModelViews bottomSubView, double bottomZoom) {
			setViews(mainView, mainZoom, topSubView, topZoom, bottomSubView, bottomZoom);

			Point3d center = new Point3d();
			if (focusComp instanceof FemMuscleModel) {
				((FemMuscleModel) focusComp).getSurfaceMesh().computeCentroid(center);
			}
			else if (focusComp instanceof RigidBody) {
				((RigidBody) focusComp).getMesh().computeCentroid(center);
			}
			else if (focusComp instanceof artisynth.core.mechmodels.Point) {
				center = ((artisynth.core.mechmodels.Point) focusComp).getPosition();
			}

			mainViewer.setCenter(center);
			Point3d newEye = new Point3d(mainViewer.getEye());
			newEye.add(center);
			mainViewer.setEye(newEye);

			if (topSubViewer != null) {
				topSubViewer.setCenter(center);
				Point3d newTopEye = new Point3d(topSubViewer.getEye());
				newTopEye.add(center);
				topSubViewer.setEye(newTopEye);
			}
			if (bottomSubViewer != null) {
				bottomSubViewer.setCenter(center);
				Point3d newBottomEye = new Point3d(bottomSubViewer.getEye());
				newBottomEye.add(center);
				bottomSubViewer.setEye(newBottomEye);
			}
		}


		public static enum ProjectionType {
			ORTHOGRAPHIC, PERSPECTIVE;
		}


		public void setAllToPerspective() {
			setProjectionType(ProjectionType.PERSPECTIVE, ProjectionType.PERSPECTIVE, ProjectionType.PERSPECTIVE);
		}
		public void setAllToOrthographic() {
			setProjectionType(ProjectionType.ORTHOGRAPHIC, ProjectionType.ORTHOGRAPHIC, ProjectionType.ORTHOGRAPHIC);
		}
		public void setProjectionType(ProjectionType mainProjType, ProjectionType topSubProjType, ProjectionType bottomSubProjType) {
			mainViewer.setOrthographicView(mainProjType == ProjectionType.ORTHOGRAPHIC);

			if (topSubViewer != null) {
				topSubViewer.setOrthographicView(topSubProjType == ProjectionType.ORTHOGRAPHIC);
			}

			if (bottomSubViewer != null) {
				bottomSubViewer.setOrthographicView(bottomSubProjType == ProjectionType.ORTHOGRAPHIC);
			}

			Main.getMain().getViewerManager().render();
		}

		public static CustomViewerLayout splitMainView(VocalTractBase vocalTractBase) {
			return splitMainView(vocalTractBase, AxisAlignedRotation.X_Y, AxisAlignedRotation.X_Z, AxisAlignedRotation.X_NZ);
		}

		public static CustomViewerLayout splitMainView(VocalTractBase vocalTractBase, AxisAlignedRotation mainPanelView, AxisAlignedRotation topRightPanelView, AxisAlignedRotation bottomRightPanelView) {
			CustomViewerLayout cvl = new CustomViewerLayout();
			int w = Main.getMain().getMainFrame().getWidth();
			int h = Main.getMain().getMainFrame().getHeight();

			for (Component comp : Main.getMain().getMainFrame().getContentPane().getComponents()) {
				if (comp instanceof JPanel) {
					for (Component panelComp : ((JPanel) comp).getComponents()) {
						if (panelComp instanceof JToolBar) {
							cvl.toolBarPanel = (JPanel) comp;
							cvl.defaultToolBars = cvl.toolBarPanel.getComponents();
							break;
						}
					}
				}
			}

			//TODO the main panel appears to be overlapped by the subpanels, and this has consequences for offscreen movie rendering (as the overlap is visible); insets are thus temporarily set to 0 so that this is not the case
			Insets insets = new Insets(0, 0, 0, 0);
			int both = GridBagConstraints.BOTH;
			int center = GridBagConstraints.CENTER;

			cvl.mainPanel = Main.getMain().getMainFrame().getGLPanel();
			cvl.mainViewer = (GL3Viewer) cvl.mainPanel.getViewer();
			cvl.mainViewer.getCanvas().setPreferredSize(new Dimension((2*w)/3, h/2));
			cvl.mainViewer.getCanvas().setMinimumSize(new Dimension(w/3, h/2));

			cvl.topSubViewer = (GL3Viewer) CustomViewerLayout.createSubViewer(w/3, h/2, w/6, h/4);
			cvl.bottomSubViewer = (GL3Viewer) CustomViewerLayout.createSubViewer(w/3, h/2, w/6, h/4);

			cvl.topToolBar = new ViewerToolBar(cvl.topSubViewer, true);
			cvl.bottomToolBar = new ViewerToolBar(cvl.bottomSubViewer, true);

			cvl.topToolBar.setOrientation(JToolBar.VERTICAL);
			cvl.bottomToolBar.setOrientation(JToolBar.VERTICAL);

			cvl.toolBarPanel.add(cvl.topToolBar);
			cvl.toolBarPanel.add(cvl.bottomToolBar);

			for (Component comp : Main.getMain().getMainFrame().getContentPane().getComponents()) {
				if (comp instanceof JSplitPane) {
					JSplitPane splitPanel = (JSplitPane) comp;

					JPanel panel = new JPanel(new GridBagLayout());
					panel.add(cvl.mainPanel, AuxTools.defineConstraints(both, center, 0, 0, 1, 2, 0, 0, 1.0, 1.0, insets));
					panel.add(cvl.topSubViewer.getCanvas(), AuxTools.defineConstraints(both, center, 1, 0, 1, 1, 0, 0, 0.5, 1.0, insets));
					panel.add(cvl.bottomSubViewer.getCanvas(), AuxTools.defineConstraints(both, center, 1, 1, 1, 1, 0, 0, 0.5, 1.0, insets));

					cvl.contentPane = panel;

					splitPanel.setRightComponent(panel);
					splitPanel.validate();
					splitPanel.repaint();
					break;
				}
			}

			//Reposition toolbars in the panel
			Component[] toolBars = cvl.toolBarPanel.getComponents();
			cvl.toolBarPanel.removeAll();
			cvl.toolBarPanel.setLayout(new BoxLayout(cvl.toolBarPanel, BoxLayout.PAGE_AXIS));

			for (Component comp : toolBars) {
				cvl.toolBarPanel.add(comp);
			}

			cvl.toolBarPanel.validate();
			return cvl;
		}

		public void resetViewers() {
			if (topSubViewer != null && bottomSubViewer != null) {
				ViewerManager vm = Main.getMain().getViewerManager();
				//Ensure that the main viewer is the only viewer (which may not be the case in the event that a splitViewer was selected by a simulation)
				for (Component comp : Main.getMain().getMainFrame().getContentPane().getComponents()) {
					if (comp instanceof JSplitPane) {
						JSplitPane splitPanel = (JSplitPane) comp;

						vm.clearRenderables();
						//vm.removeViewer(mainPanel.getViewer());
						vm.removeViewer(topSubViewer);
						vm.removeViewer(bottomSubViewer);

						mainPanel.getViewer().clearRenderables();
						mainPanel.getViewer().clearDraggers();

						//GLViewerPanel newMainPanel = getMain().getMainFrame().createNewGLPanel(mainPanel.getWidth(), mainPanel.getHeight());

						splitPanel.setRightComponent(mainPanel);

						//Reset the toolbar panel
						toolBarPanel.removeAll();
						toolBarPanel.setLayout(new BoxLayout(toolBarPanel, BoxLayout.PAGE_AXIS));

						for (Component toolBar : defaultToolBars) {
							toolBarPanel.add(toolBar);
						}

						toolBarPanel.validate();
					}
				}
			}

			//Remove all clip planes
			GLViewer mainViewer = Main.getMain().getViewer();
			for (int i = 0; i < mainViewer.getNumClipPlanes(); i++) {
				mainViewer.removeClipPlane(mainViewer.getClipPlane(i));
			}
		}


		public static GLViewer createSubViewer(int width, int height, int minWidth, int minHeight) {
                   GLViewer viewer = null;
                   GLViewer.GLVersion vers = Main.getMain().getGLVersion();
                   switch (vers) {
                      case GL2:
                         viewer = new GL2Viewer (width, height);
                         break;
                      case GL3:
                         viewer = new GL3Viewer (width, height);
                         break;
                      default:
                         throw new UnsupportedOperationException (
                            "Unimplemented viewer type: " + vers);
                   }
			viewer.getCanvas().setPreferredSize(new Dimension(width, height));
			viewer.getCanvas().setMinimumSize(new Dimension(minWidth, minHeight));


			Main.getMain().getViewerManager().addViewer(viewer);
			AxisAngle REW = AxisAngle.ROT_X_90;
			viewer.setDefaultAxialView(AxisAlignedRotation.getNearest(new RotationMatrix3d(REW)));
			Main.getMain().initializeViewer(viewer, REW);

			viewer.setAxialView(AxisAlignedRotation.getNearest(new RotationMatrix3d(REW)));
			viewer.setOrthographicView(true);

			return viewer;
		}
	}


	/** Sets up the viewer. If <b>focusComp</b> is null then the origin is used as the center of the view. For zoom, use smaller values to zoom in, and larger ones to zoom out. **/
	public static void setView(GL3Viewer viewer, ModelViews view, ModelComponent focusComp, double zoom) {
		Point3d center = new Point3d();
		if (focusComp != null) {
			if (focusComp instanceof FemMuscleModel) {
				((FemMuscleModel) focusComp).getSurfaceMesh().computeCentroid(center);
			}
			else if (focusComp instanceof RigidBody) {
				((RigidBody) focusComp).getMesh().computeCentroid(center);
			}
			else if (focusComp instanceof artisynth.core.mechmodels.Point) {
				center = ((artisynth.core.mechmodels.Point) focusComp).getPosition();
			}
		}

		Point3d mainEye = view.eye;
		mainEye.scale(zoom);
		viewer.setEye(mainEye);

		Point3d centerAdjusted = new Point3d(center);
		centerAdjusted.add(view.center);
		viewer.setCenter(centerAdjusted);
		Point3d newEye = new Point3d(viewer.getEye());
		newEye.add(centerAdjusted);
		viewer.setEye(newEye);

	}

	public static void setView(GLViewer viewer, ModelViews view, Point3d center, double zoom) {
		Point3d mainEye = view.eye;
		mainEye.scale(zoom);
		viewer.setEye(mainEye);
		viewer.setCenter(center);

		Point3d centerAdjusted = new Point3d(center);
		centerAdjusted.add(center);
		viewer.setCenter(centerAdjusted);
		Point3d newEye = new Point3d(viewer.getEye());
		newEye.add(centerAdjusted);
		viewer.setEye(newEye);

	}


	public static void createViewerFrame(int x, int y, int w, int h, Point3d eyePos, Point3d centerPos, double zoom, double near, double far) {
		GLViewerFrame vf = Main.getMain().createViewerFrame();
		GLViewer v = vf.getViewer();

		vf.setSize(w, h);
		vf.setLocation(x, y);

		v.setOrthogonal(zoom, near, far);

		v.setEye(eyePos);
		v.setCenter(centerPos);
	}

	public static void reshapeMainFrame(int width, int height) {
		Main.getMain().getMainFrame().setSize(new Dimension(width, height));
		Main.getMain().getMainFrame().revalidate();
		Main.getMain().getMainFrame().repaint();
	}

	public static void reshapeContentPane(int width, int height) {
		JPanel contentPanel = getContentPane();
		contentPanel.setSize(new Dimension(width, height));
	}


	public static void addScreenShotSaver(final char triggerKey, final String filePath) {
		final class ScreenShotCounter {
			int screenShotCount = 1;
		}

		final ScreenShotCounter ssc = new ScreenShotCounter();

		//Define an inline key adapter to handle custom keyboard input
		VocalTractBase.addCustomListener(new KeyAdapter() {
			public void keyTyped(KeyEvent e) {

				if (e.getKeyChar() == triggerKey) {
					Movie.takeScreenShot(filePath, "ScreenShot_" + ssc.screenShotCount++);
					System.out.println("Screen shot saved as " + filePath + "ScreenShot_" + ssc.screenShotCount + Files.SCREENSHOT.getSuffix());
				}
			}
		});
	}

}

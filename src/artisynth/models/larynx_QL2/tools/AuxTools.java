package artisynth.models.larynx_QL2.tools;

import java.awt.GridBagConstraints;
import java.awt.Insets;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import artisynth.core.mechmodels.RigidBody;
import artisynth.models.larynx_QL2.components.HardBody;
import artisynth.models.larynx_QL2.components.Names;
import artisynth.models.larynx_QL2.components.SoftBody;
import artisynth.models.larynx_QL2.components.SoftBody.FemMethods.FemIOMethods;
import artisynth.models.larynx_QL2.components.Structures;
import artisynth.models.larynx_QL2.components.Tissue;

import java.util.Collections;
import java.util.Comparator;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.Map.Entry;
/**
 *  <p>
 *   @author Scott Moisik (Scott.Moisik@ntu.edu.sg), 2017, Nanyang Technological University, Singapore. 
 *   <p>
 *   If you use this model in your research, please cite: 
 *   Moisik, S. R., & Gick, B. (2017). The quantal larynx: the stable regions of laryngeal biomechanics and implications for speech production. Journal of Speech Language and Hearing Research, 60(3), 540-560. https://doi.org/10.1044/2016_JSLHR-S-16-0019

 *  **/
public class AuxTools {

	public static class Timer {
		long startTime, stopTime;
		public String timeeName;
		boolean timeReported = false;
		
		public Timer() {
		       this(true);
		}
		public Timer(String timee) {
		       this(true, timee);
		}
		
		public Timer(boolean startImmediatelyFlag) {
		       if (startImmediatelyFlag) {
		              start();
		       }
		}
		public Timer(boolean startImmediatelyFlag, String timeeName) {
		       this(startImmediatelyFlag);
			this.timeeName= timeeName;
		}

		public void start() {
			timeReported = false;
			startTime = System.currentTimeMillis();
		}

		
		public void stop() {
                     if (!timeReported) {
                            stop("Execution time: ", true);
                            timeReported = true;
                     }
              }
		
		public void stop(String customMessage) {
                     if (!timeReported) {
                            stop(customMessage, true);
                            timeReported = true;
                     }
              }
              
		
		public void stop(boolean reportOnNewLineFlag) {
			if (!timeReported) {
				stop("Execution time: ", reportOnNewLineFlag);
				timeReported = true;
			}
		}

		public void stop(String customMessage, boolean reportOnNewLineFlag) {
			if (!timeReported) {
				if (reportOnNewLineFlag) {
				       System.out.println("\n" + customMessage + createTimeMessage());
				}
				else {
				       System.out.print(customMessage + createTimeMessage());
				}
				timeReported = true;
			}
		}
		
		public String createTimeMessage() {
			String timeMessage = "";
			stopTime = System.currentTimeMillis();

			long durationMiliseconds = (stopTime - startTime);
			float durationSeconds = (float) durationMiliseconds / (float) 1000;
			float durationMinutes = durationSeconds / 60f;
			timeMessage = durationMiliseconds + " ms (" + String.format("%.3f", durationSeconds) + " s; " + String.format("%.3f", durationMinutes) + " min)";
			return timeMessage;
		}
		
		public double getDurationInMiliseconds() {
                     return (stopTime - startTime);
              }
		
		public double getDurationInSeconds() {
		       return getDurationInMiliseconds() / 1000;
		}
		
		public double getDurationInMinutes() {
                     return getDurationInSeconds() / 60;
              }

	}


	/** Allows for all model component files to be resaved to the specified path. This is useful if the user wishes to store the results of a particular transform
	 * so that it does not need to be applied everytime the model is loaded. ***Note*** that this model will overwrite any similarly named files located in <b>filePath</b>.**/
	public static void saveAllModelFiles(String filePath, Structures structs) {
		//Save all hard bodies to OBJ files
		for (HardBody hb : structs.getHardBodies()) {
			saveMeshToOBJ(filePath + hb.getName() + Names.Files.RIGIDBODY.getSuffix(), hb.getName(), hb.getMesh());

			//Save the collision box if one exists
			int boxNumber = 1;
			for (RigidBody collisionBox : hb.getCollisionBoxes()) {
				saveMeshToOBJ(filePath + hb.getName() + Names.Files.COLLISION_BOX.getNumberedSuffix(boxNumber++), hb.getName() + " collision box", collisionBox.getMesh());
			}
		}

		//Save all tissues to "tissues.csv" file
		Tissue.writeTissueDataToCSV(filePath + Names.Files.TISSUES, structs, false);

		//Save all soft bodies to CSV files and save all FEM muscles to "[fem_name]_muscles.csv" files
		for (SoftBody sb : structs.getSoftBodies()) {
		       if (sb.isUsed()) {
		              FemIOMethods.writeFemToCSV(sb.getFem(), filePath + sb.getName() + Names.Files.FEM.getSuffix());
		              FemIOMethods.writeMusclesToCSV(sb.getFem(), filePath + sb.getName() + Names.Files.FEM_MUSCLES.getSuffix());
		              FemIOMethods.writeAttachmentsToCSV(sb.getFem(), filePath + sb.getName() + Names.Files.ATTACHMENTS.getSuffix());
		              FemIOMethods.writeElementColorsToCSV(sb.getFem(), filePath + sb.getName() + Names.Files.COLORS.getSuffix());
		              saveMeshToOBJ(filePath + sb.getName() + Names.Files.SURFACE_MESH.getSuffix(), sb.getName(), sb.getMesh());

		              //Save collision subsurfaces if they exist
		              int surfaceNumber = 1;
		              for (PolygonalMesh subsurface : sb.getCollisionSubsurfaces()) {
		                     int idx = subsurface.getName().lastIndexOf("_");
		                     String subsurfaceName = subsurface.getName().substring(idx + 1, subsurface.getName().length());
		                     saveMeshToOBJ(filePath + sb.getName() + Names.Files.COLLISION_SUBSURFACE.getNumberedNamedSuffix(surfaceNumber++, subsurfaceName), sb.getName() + " collision subsurface " + subsurfaceName, subsurface);
		              }
		       }
		}
	}



	/** Applies the world transform of this mesh and writes to an OBJ file as outlined in {@link maspack.geometry.PolygonalMesh#write write}. **/
	public static void saveMeshToOBJ(String fileName, String meshName, PolygonalMesh mesh) {
		try {
			BufferedWriter bw = new BufferedWriter(new FileWriter(new File(fileName), false));

			//Write the transformed vertex data
			for (Vertex3d v : mesh.getVertices()) {
				Point3d pos = v.getWorldPoint();
				bw.write("v " + pos.x + " " + pos.y + " " + pos.z + "\n");	
			}

			for (Face face : mesh.getFaces()) {
				String faceLine = "f";
				int[] indices = face.getVertexIndices();
				for (int i : indices) {
					faceLine += " " + (i + 1);	
				}

				bw.write(faceLine + "\n");
			}


			bw.close();
			System.out.println("Mesh '" + meshName + "' saved to " + fileName);

			//mesh.write(new PrintWriter(fileName), "%g");
			//System.out.println("Mesh '" + meshName + "' saved to " + fileName);
		} catch (FileNotFoundException e) {
			System.err.println("Failed to write mesh " + meshName + " to file " + fileName);
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	public static PolygonalMesh loadPolygonalMesh(String fileName) {
		try {
			PolygonalMesh pm =  new PolygonalMesh(new File(fileName));
			return pm;

		} catch (IOException e) {
			System.out.println("Failed to load polygonal mesh from file " + fileName);
			e.printStackTrace();
		}

		return null;
	}

	public static LinkedHashMap<Object, Double> sortByComparator(Map<Object, Double> unsortedMap) {
              List<Map.Entry<Object, Double>> list = new LinkedList<Map.Entry<Object, Double>>(unsortedMap.entrySet());

              Collections.sort(list, new Comparator<Map.Entry<Object, Double>>() {
                     @Override
                     public int compare(Entry<Object, Double> o1, Entry<Object, Double> o2) {
                            return ((Map.Entry<Object, Double>) (o1)).getValue().compareTo(((Map.Entry<Object, Double>) (o2)).getValue());
                     }
              });

              LinkedHashMap<Object, Double> sortedMap = new LinkedHashMap<Object, Double>();
              for (Iterator<Entry<Object, Double>> it = list.iterator(); it.hasNext();) {
                     Map.Entry<Object, Double> entry = (Map.Entry<Object, Double>) it.next();
                     sortedMap.put(entry.getKey(), entry.getValue());
              } 

              return sortedMap;
       }

/*	
	public static LinkedHashMap<Integer, Double> sortByComparator(Map<Integer, Double> unsortedMap) {
		List<Map.Entry<Integer, Double>> list = new LinkedList<Map.Entry<Integer, Double>>(unsortedMap.entrySet());

		Collections.sort(list, new Comparator<Map.Entry<Integer, Double>>() {
			@Override
			public int compare(Entry<Integer, Double> o1, Entry<Integer, Double> o2) {
				return ((Map.Entry<Integer, Double>) (o1)).getValue().compareTo(((Map.Entry<Integer, Double>) (o2)).getValue());
			}
		});

		LinkedHashMap<Integer, Double> sortedMap = new LinkedHashMap<Integer, Double>();
		for (Iterator<Entry<Integer, Double>> it = list.iterator(); it.hasNext();) {
			Map.Entry<Integer, Double> entry = (Map.Entry<Integer, Double>) it.next();
			sortedMap.put(entry.getKey(), entry.getValue());
		} 

		return sortedMap;
	}
*/
	/** 
	 * Routine for laying out controls according to GridBag Layout Manager. 
	 * **/
	public static GridBagConstraints defineConstraints(int fillType, int anchorDir, int xLoc, int yLoc, int gridWidth, int gridHeight, int ipadx, int ipady, double weightx, double weighty, Insets inset) {
		//Declare 'GridBagLayout' constraint
		GridBagConstraints c = new GridBagConstraints();

		c.anchor = anchorDir;
		c.fill = fillType;
		c.ipadx = ipadx;
		c.ipady = ipady;
		c.weightx = weightx;
		c.weighty = weighty;
		c.gridx = xLoc;
		c.gridy = yLoc;
		c.gridwidth = gridWidth;
		c.gridheight = gridHeight;
		c.insets = inset;

		return c;
	}

	public static String pointArrayAsString(Point3d[] points, boolean flipRightFlag) {
		String string = "";
		int count = 1;
		for (Point3d point : points) {
			string += (flipRightFlag ? -point.x : point.x) + ", " + point.y + ", " + point.z + (count < points.length ? ", " : "");
			count++;
		}

		return string;
	}

	public static String pointAsString(Point3d point, int precision) {
		String format = "%." + precision + "f";
		return String.format(format, point.x) + ", " + String.format(format, point.y)  + ", " + String.format(format, point.z);
	}

	public static ArrayList<String[]> readDataFromCSV(String fileName) {
		ArrayList<String[]> rawTextLines = new ArrayList<String[]>();

		try {
			//Create a text file reader and parse the imported file for numeric data
			String line = "";
			String csvSeparator = ",";
			BufferedReader br;

			br = new BufferedReader(new FileReader(fileName));

			while ((line = br.readLine()) != null) {
				String[] currentLine = line.split(csvSeparator);

				for (int i = 0; i < currentLine.length; i++) {
					currentLine[i] = currentLine[i].trim();
				}

				rawTextLines.add(currentLine);
			}

			br.close();
		} catch (FileNotFoundException e) {
			System.err.println("Could not locate file " + fileName);
		} catch (IOException e) {
			e.printStackTrace();
		}

		return rawTextLines;
	}


	public static enum FlipAxis {
		X, Y, Z, ALL; 
	}
	public static Point3d flipValue(Point3d point, FlipAxis flipAxis) {
		switch (flipAxis) {
		case ALL:
			return new Point3d(-point.x, -point.y, -point.z);
		case X:
			return new Point3d(-point.x, point.y, point.z);
		case Y:
			return new Point3d(point.x, -point.y, point.z);
		case Z:
			return new Point3d(point.x, point.y, -point.z);
		default:
			return new Point3d(point);
		}
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

	public static Point3d getAveragePosition(Point3d ...points) {
		Point3d averagePoint = new Point3d();
		double count = 0;
		for (Point3d point : points) {
			averagePoint.add(point);
			count++;
		}
		
		if (count > 0) {
			averagePoint.scale(1/count);
		}
		else {
			System.err.println("Average position cannot be defined: no input points");
		}
		return averagePoint;
	}
	
	
	public static double getAbsMaxValues(double[] data) {
		double maxValue = Math.abs(data[0]);
		
		for (int i = 0; i < data.length; i++){
			if (Math.abs(data[i]) > maxValue){
				maxValue = Math.abs(data[i]);
			}
		} 
		
		return maxValue;
	}



	public static boolean isNaN(Vector3d dist) {
		return ((Double.isNaN(dist.x)) || (Double.isNaN(dist.y)) || (Double.isNaN(dist.z)));
		
	}
	
}

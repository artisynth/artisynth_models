package artisynth.models.larynx_QL2.components;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Point;
import artisynth.models.larynx_QL2.components.Structures.HasCommonProperties;
import artisynth.models.larynx_QL2.tools.AuxTools;
import maspack.matrix.Point3d;
/**
 *  <p>
 *   @author Scott Moisik (Scott.Moisik@ntu.edu.sg), 2017, Nanyang Technological University, Singapore. 
 *   <p>
 *   If you use this model in your research, please cite: 
 *   Moisik, S. R., & Gick, B. (2017). The quantal larynx: the stable regions of laryngeal biomechanics and implications for speech production. Journal of Speech Language and Hearing Research, 60(3), 540-560. https://doi.org/10.1044/2016_JSLHR-S-16-0019

 *  **/
public abstract class Tissue extends StructureBase implements HasCommonProperties {
	public String type;
	public String name;
	public String side;
	public boolean pairedFlag = false;

	public String originName;
	public String insertionName;

	public AbstractBody originBody;
	public AbstractBody insertionBody;

	public Point originPoint;
	public Point insertionPoint;
	public Point[] points;

	public Point3d originLoc;
	public Point3d insertionLoc;
	public double restLength;

	public int numberOfPoints;
	public Point3d[] locs;

	public Tissue pair = null;
	private boolean dataWrittenFlag = false;
	protected boolean saveableFlag = true;


	Tissue(String name, String type, String side, AbstractBody[] bodies, Point3d[] locs) {
		this.name = name;
		this.type = type;
		this.side = side;
		
		if (!side.equals("")) {
			pairedFlag = true;
		}

		originBody = bodies[0];
		insertionBody = bodies[1];

		originName = originBody.getName();
		insertionName = insertionBody.getName();

		this.locs = locs;
		numberOfPoints = locs.length;
		originLoc = locs[0];
		insertionLoc = locs[numberOfPoints - 1];
	}

	public abstract void setVisible(boolean visibleFlag);

	public String getName() { return name; }
	public abstract void setName(String name);
	public String getSide() { return side; }
	public void setSide(String side) { this.side = side; }
	public abstract void setLeftRightNames();
	public abstract void setRenderRadius(double size);
	public abstract void removeComponents(MechModel mech);

	public void setDynamic(boolean dynamicFlag) {}
	public boolean isSaveable() { return saveableFlag; }
	public String print() {
		return type + ", " + side + ", " + name + ", ";
	}

	public void setPoints(Point ... points) { this.points = points; }
	public Point3d[] getLocations() {
		Point3d[] locs = new Point3d[points.length];
		int count = 0;
		for (Point point : points) {
			locs[count++] = point.getPosition();
		}
		return locs;
	}

	public static String createDataLine(Tissue t) {
		if (t instanceof Contractile) {
			Contractile m = (Contractile) t;
			return m.type + ", " + m.name + ", " + m.side  + ", " + m.originName + ", " + m.insertionName + ", "
			+ m.numberOfPoints + ", " + AuxTools.pointArrayAsString(m.getLocations(), false) + ", "
			+ m.getDamping() + ", " + m.getPassiveFraction() + ", " + m.getTendonRatio() + ", " + m.getForceScaling() + "\n";
		}
		else if (t instanceof Ligamentous) {
			Ligamentous lig = (Ligamentous) t;
			return lig.type + ", " + lig.name + ", " + lig.side  + ", " + lig.originName + ", " + lig.insertionName + ", " 
			+ lig.numberOfPoints + ", " + AuxTools.pointArrayAsString(lig.getLocations(), false) + ", " 
			+ lig.getStiffness() + ", " + lig.getDamping() + ", " + lig.getRestLength() + "\n";  

		}
		else if (t instanceof Diarthrotic) {
			Diarthrotic dia = (Diarthrotic) t;
			if (dia.type.equals("joint")) {
				return dia.type + ", " + dia.name + ", " + dia.side  + ", " + dia.originName + ", " + dia.insertionName + ", " 
				+ dia.numberOfPoints + ", " + AuxTools.pointArrayAsString(dia.getLocations(), false) + ", " 
				+ dia.getLinearCompliance() + ", " + dia.getRotatryCompliance()  + ", " + dia.getMinAngle() + ", " + dia.getMaxAngle() + "\n";
			}
			else if (dia.type.equals("plane")) {
				return dia.type + ", " + dia.name + ", " + dia.side  + ", " + dia.originName + ", " + dia.insertionName + ", " 
				+ dia.numberOfPoints + ", " + AuxTools.pointArrayAsString(dia.getLocations(), false) + ", " 
				+ dia.getAngle() + ", " + dia.getSize() + ", " + dia.getLinearCompliance() + "\n";
			}  
		}
		return "";
	}

	public static String createDataLine(Tissue t, Tissue sym) {
		if (t instanceof Contractile) {
			Contractile m = (Contractile) t;
			return m.type + ", " + m.name + ", " + m.side  + ", " + m.originName + ", " + m.insertionName + ", "
			+ m.numberOfPoints + ", " + AuxTools.pointArrayAsString(sym.getLocations(), true) + ", "
			+ m.getDamping() + ", " + m.getPassiveFraction() + ", " + m.getTendonRatio() + ", " + m.getForceScaling() + "\n";
		}
		else if (t instanceof Ligamentous) {
			Ligamentous lig = (Ligamentous) t;
			return lig.type + ", " + lig.name + ", " + lig.side  + ", " + lig.originName + ", " + lig.insertionName + ", " 
			+ lig.numberOfPoints + ", " + AuxTools.pointArrayAsString(sym.getLocations(), true) + ", " 
			+ lig.getStiffness() + ", " + lig.getDamping() + ", " + lig.getRestLength() + "\n";  

		}
		else if (t instanceof Diarthrotic) {
			Diarthrotic dia = (Diarthrotic) t;
			if (dia.type.equals("joint")) {
				return dia.type + ", " + dia.name + ", " + dia.side  + ", " + dia.originName + ", " + dia.insertionName + ", " 
				+ dia.numberOfPoints + ", " + AuxTools.pointArrayAsString(sym.getLocations(), true) + ", " 
				+ dia.getLinearCompliance() + ", " + dia.getRotatryCompliance()  + ", " + dia.getMinAngle() + ", " + dia.getMaxAngle() + "\n";
			}
			else if (dia.type.equals("plane")) {
				return dia.type + ", " + dia.name + ", " + dia.side  + ", " + dia.originName + ", " + dia.insertionName + ", " 
				+ dia.numberOfPoints + ", " + AuxTools.pointArrayAsString(sym.getLocations(), true) + ", " 
				+ dia.getAngle() + ", " + dia.getSize() + ", " + dia.getLinearCompliance() + "\n";
			}
		}
		
		return "";
	}

	public static void writeTissueDataToCSV(String fileName, Structures structs, boolean makeLeftMatchRightFlag) {
		//Reset data write flags
		for (Tissue t : Structures.getTissues()) {
			t.dataWrittenFlag = false;
		}

		try {
			BufferedWriter bw = new BufferedWriter(new FileWriter(new File(fileName), false));

			for (Tissue t : Structures.getTissues()) {
				if (!t.dataWrittenFlag && t.saveableFlag && !(t.side.equals("left") && makeLeftMatchRightFlag)) {
					bw.write(createDataLine(t));
					t.dataWrittenFlag = true;

					if (makeLeftMatchRightFlag && t.side.equals("right")) {
						if (!t.pair.dataWrittenFlag) {
							bw.write(createDataLine(t.pair, t));
							t.pair.dataWrittenFlag = true;
						}
					}					
				}
			}

			bw.close();
			System.out.println("Current " + (makeLeftMatchRightFlag ? "right-symmetrized version of " : "") + "tissue data (for muscles and ligaments) saved to " + fileName);
		} catch (IOException e) {
			System.out.println("An error occurred reading file " + fileName + "; if the file is open, close it and try again.");
		} catch (NullPointerException e) {
			System.out.println("An unknown error occurred when attempting to save tissue data.");
		}
	}

	public static void readTissuesFromCSV(MechModel mech, String fileName, boolean supressWarnings) {
		ArrayList<String[]> tissueData = AuxTools.readDataFromCSV(fileName);

		for (String[] td : tissueData) {
			String type = td[0];
			String name = td[1];
			String side = td[2];
			AbstractBody origin = Structures.getBodyByName(td[3]);
			AbstractBody insertion = Structures.getBodyByName(td[4]);

			if (origin != null && insertion != null) {
				if (origin.isUsed() && insertion.isUsed()) {
					AbstractBody[] bodies = new AbstractBody[]{origin, insertion};

					int numberOfPoints = Integer.valueOf(td[5]);
					Point3d[] points = new Point3d[numberOfPoints];
					for (int i = 0; i < numberOfPoints; i++) {
						int idx = i*3 + 6;
						double x = Double.valueOf(td[idx]);
						double y = Double.valueOf(td[idx + 1]);
						double z = Double.valueOf(td[idx + 2]);
						points[i] = new Point3d(x, y, z);
					}

					int j = 6 + 3*numberOfPoints;

					if (type.equals("muscle")) {
						double damping = Double.valueOf(td[j]);
						double passiveFraction = Double.valueOf(td[j + 1]);
						double tendoRatio = Double.valueOf(td[j + 2]);
						double forceScaling = Double.valueOf(td[j + 3]);
						Structures.addTissue(new Contractile(name, side, mech, bodies, points, damping, passiveFraction, tendoRatio, forceScaling));
					}
					else if (type.equals("ligament")) {
						double stiffness = Double.valueOf(td[j]);
						double damping = Double.valueOf(td[j + 1]);
						double restLength = ((td.length > j + 2) ? Double.valueOf(td[j + 2]) : -1.0); 
						Structures.addTissue(new Ligamentous(name, side, mech, bodies, points, stiffness, damping, (restLength == 0.0 ? true : false)));
					}
					else if (type.equals("joint")) {
						double linCompliance = Double.valueOf(td[j]);
						double rotCompliance = Double.valueOf(td[j + 1]);
						double minAngle = Double.valueOf(td[j + 2]);
						double maxAngle = Double.valueOf(td[j + 3]);
						Structures.addTissue(new Diarthrotic(name, side, mech, bodies, points, linCompliance, rotCompliance, minAngle, maxAngle));
					}
					else if (type.equals("plane")) {
						double angle = Double.valueOf(td[j]);
						double size = Double.valueOf(td[j + 1]);
						double compliance = Double.valueOf(td[j + 2]);
						
						Structures.addTissue(new Diarthrotic(name, side, bodies, points, angle, size, compliance));
					}
				}
			}
			else {
				if (!supressWarnings) {
					System.err.println("Could not load tissue '" + name + "': " + (origin == null ? " Origin '" + td[3] + "' could not be found. " : "") + (insertion == null ? " Insertion '" + td[4] + "' could not be found. " : ""));
				}
			}
		}

		//Find paired structures and associate them
		int antiSymmetryCount = 0;
		for (Tissue t1 : Structures.getTissues()) {
			if (t1.pairedFlag && t1.pair == null) {
				for (Tissue t2 : Structures.getTissues()) {
					if (t1 != t2 && t2.name.equals(t1.name)) {
						t1.pair = t2;
						t2.pair = t1;

						if (!t1.type.equals("plane") && !t2.type.equals("plane") && !t1.name.contains("anti-symmetric") && !(t2.name.contains("anti-symmetric"))) {
						       //Do symmetry check
						       if (t1.originLoc.x != -t2.originLoc.x || t1.originLoc.y != t2.originLoc.y || t1.originLoc.z != t2.originLoc.z) {
						              //System.err.println("Origin locations of tissue pair " + t1.name + " are not symmetric about the midsagittal plane.");
						              antiSymmetryCount++;
						       }
						       if (t1.insertionLoc.x != -t2.insertionLoc.x || t1.insertionLoc.y != t2.insertionLoc.y || t1.insertionLoc.z != t2.insertionLoc.z) {
						              //System.err.println("Insertion locations of tissue pair " + t1.name + " are not symmetric about the midsagittal plane.");
						              antiSymmetryCount++;
						       }
						}
					}
				}
			}
		}
		
		if (antiSymmetryCount == 0) {
		       System.out.println("Tissues are symmetric");
		}
	}






}

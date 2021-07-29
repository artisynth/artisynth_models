package artisynth.models.larynx_QL2.components;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import artisynth.core.mechmodels.PlanarJoint;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ModelComponent;
import artisynth.models.larynx_QL2.components.Names.Files;
import artisynth.models.larynx_QL2.components.Names.VocalTractType;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.AffineTransform3dBase;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
/**
 *  <p>
 *   @author Scott Moisik (Scott.Moisik@ntu.edu.sg), 2017, Nanyang Technological University, Singapore. 
 *   <p>
 *   If you use this model in your research, please cite: 
 *   Moisik, S. R., & Gick, B. (2017). The quantal larynx: the stable regions of laryngeal biomechanics and implications for speech production. Journal of Speech Language and Hearing Research, 60(3), 540-560. https://doi.org/10.1044/2016_JSLHR-S-16-0019

 *  **/
public class HardBody extends AbstractBody {
	private RigidBody body;
	private ArrayList<RigidBody> collisionBoxes = new ArrayList<RigidBody>();

	public HardBody(MechModel mech, VocalTractType vtModel, String name) {
		super(name, true, true, true, true, 1.0, new RigidTransform3d());
		addToModel(mech, vtModel.getFullFileName(name + Files.RIGIDBODY.getSuffix()));
	}
	

       public HardBody(MechModel mech, VocalTractType vtModel, Names name, HardBodyConstraint hbc) {
              super(name.getName(), true, true, true, true, 1.0, new RigidTransform3d());
              addToModel(mech, vtModel.getFullFileName(name + Files.RIGIDBODY.getSuffix()));
              switch (hbc) {
                     case FULL_PLANAR_JOINT_X:
                            PlanarJoint fpj = new PlanarJoint(
                               body, null, Point3d.ZERO, new Vector3d(1.0, 0.0, 0.0));
                            fpj.setName(name + " mid-sagittal constraint plane");
                            fpj.setRenderProps(new RenderProps());
                            fpj.getRenderProps().setVisible(false);
                            mech.addBodyConnector(fpj);
              }
       }
       

	public HardBody(MechModel mech, VocalTractType vtModel, String name, boolean dynamicFlag) {
		super(name, true, true, dynamicFlag, true, 1.0, new RigidTransform3d());
		addToModel(mech, vtModel.getFullFileName(name + Files.RIGIDBODY.getSuffix()));
	}
	
	public HardBody(MechModel mech, VocalTractType vtModel, String name, boolean dynamicFlag, boolean visibleFlag) {
		super(name, true, true, dynamicFlag, visibleFlag, 1.0, new RigidTransform3d());
		addToModel(mech, vtModel.getFullFileName(name + Files.RIGIDBODY.getSuffix()));
	}

	public HardBody(MechModel mech, VocalTractType vtModel, String name, boolean addComponentFlag, boolean addMusclesFlag, boolean dynamicFlag, boolean visibleFlag, double scale, RigidTransform3d worldTransform) {
		super(name, addComponentFlag, addMusclesFlag, dynamicFlag, visibleFlag, scale, worldTransform);
		addToModel(mech, vtModel.getFullFileName(name + Files.RIGIDBODY.getSuffix()));
	}
	public HardBody(MechModel mech, String name, String fileName) {
		super(name, true, false, false, true, 1.0, new RigidTransform3d());
		addToModel(mech, fileName);
	}

	public HardBody(MechModel mech, String name, String fileName, double scale, RigidTransform3d worldTransform) {
		super(name, true, false, false, true, 1.0, worldTransform);
		addToModel(mech, fileName);
	}

	public HardBody(MechModel mech, VocalTractType vtModel, Names name) {
		this(mech, vtModel, name.getName());	       
	}
	
	
	public HardBody(MechModel mech, VocalTractType vtModel, Names name, boolean dynamicFlag) {
		this(mech, vtModel, name.getName(), dynamicFlag);	       
	}

	public HardBody(MechModel mech, VocalTractType vtModel, Names name, boolean dynamicFlag, boolean visibleFlag) {
		this(mech, vtModel, name.getName(), dynamicFlag, visibleFlag);	       
	}

	public HardBody(MechModel mech, VocalTractType vtModel, Names.All name, boolean dynamicFlag, boolean visibleFlag) {
		this(mech, vtModel, name.getName(), dynamicFlag, visibleFlag);
	}

	public HardBody(String name, RigidBody rigidBody) {
		super(name, true, false, true, true, 1.0, new RigidTransform3d());
		body = rigidBody;
	}
	
	public HardBody(MechModel mech, String name, RigidBody rigidBody) {
		super(name, true, false, true, true, 1.0, new RigidTransform3d());
		body = rigidBody;
		mech.addRigidBody(rigidBody);
	}
	
	public static enum HardBodyConstraint {
	       NONE,
	       FULL_PLANAR_JOINT_X;
	}
	
	
	@Override
	public void printCrashReport() {}
	@Override
	public void printStatistics() {}
	@Override
	public Point assemblePoint(MechModel mech, String name, Point3d point) {
		FrameMarker fm = new FrameMarker(name);
		fm.setPosition(point);
		mech.addFrameMarker(fm, body, point);
		return fm;
	}
	@Override
	public void setDynamic(boolean dynamicFlag) {
	       if (isUsed()) {
	              this.dynamicFlag = dynamicFlag;
	              body.setDynamic(dynamicFlag);
	       }
	}
	@Override
	public void setVisible(boolean visibleFlag) {
		if (isUsed()) {
			RenderProps.setVisible(body, visibleFlag);
		}
	}
	@Override
	public PolygonalMesh getMesh() {
		if (isUsed()) {
			return body.getMesh();
		}
		return null;
	}
	@Override
	public void transformGeometry(AffineTransform3dBase transform) { body.transformGeometry(transform); }
	@Override
	public void setName(String name) {
		this.name = name;
		body.setName(name);
	}
	@Override
	public void setLeftRightNames() {
		Vector3d centroid = new Vector3d();
		body.getMesh().computeCentroid(centroid);
		if (centroid.x > 0.0 && body.getName().contains("left")) {
			body.setName(body.getName().replace("left", "right"));
			name = body.getName();
		}
		else if (centroid.x < 0.0 && body.getName().contains("right")) {
			body.setName(body.getName().replace("right", "left"));
			name = body.getName();
		}
	}
	@Override
	public void setDensity(double density) {
		if (isUsed()) {
			body.setDensity(density);
		}
	}
	@Override
	public Point3d getKeyPoint() {
		Point3d centroid = new Point3d();
		body.getMesh().computeCentroid(centroid);
		return centroid; 
	}

	public RigidBody getBody() { return body; }
	public static RigidBody getBody(ArrayList<HardBody> hardBodies, String bodyName) {
		RigidBody rigidBody = null;

		for (HardBody hb : hardBodies) {
			if (hb.name.equals(bodyName)) {
				rigidBody = hb.body;
			}
		}

		return rigidBody;
	}
	public void addToModel(MechModel mech, String fileName) {
		if (usageFlag) {
			body = new RigidBody(getName());
			double scale = getScale();
			PolygonalMesh mesh = null;

			try {
				//OBJ file
				mesh = new PolygonalMesh(new File(fileName));
				mesh.scale(scale);

				//Set general body properties
				body.setDynamic(isDynamic());
				body.setMesh(mesh, fileName);
				body.transformGeometry(getWorldTransform());

				RenderProps.setVisible(body, isVisible());
				if (mesh.getTextureCoords() == null || 
				    mesh.getTextureCoords().size() == 0) {
					RenderProps.setColorMapEnabled(body, false);
				}

				body.setDensity(1900);
				body.setFrameDamping(0.1);
				body.setRotaryDamping(0.00001);

				mech.addRigidBody(body);
			}
			catch (IOException e) {
				System.err.println("Could not load rigid body '" + getName() + "' because of failure to locate the file " + fileName);
				usageFlag = false;
			}
		}
	}
	
	public ArrayList<RigidBody> getCollisionBoxes() { return collisionBoxes; } 

	/** Loads a collision box for this hard body, assuming a file named "[hard_body_name]_collisionBox.obj" exists in the model directory. **/
	public RigidBody createCollisionBox(MechModel mech, VocalTractType vtModel) {
		RigidBody collisionBox = new RigidBody(name + " collision box");
		collisionBoxes.add(collisionBox);
		if (!dynamicFlag) {
			try {
				PolygonalMesh mesh = null;

				//OBJ file
				String fileName = vtModel.getFullFileName(name + "_collisionBox.obj");
				mesh = new PolygonalMesh(new File(fileName));

				//Set general body properties
				collisionBox.setDynamic(false);
				collisionBox.setMesh(mesh, fileName);

				RenderProps.setVisible(collisionBox, false);
				mech.addRigidBody(collisionBox);
			}
			catch (IOException e) {
				e.printStackTrace();
			}
		}
		else {
			System.err.println("Use of collision box for a dynamic model component ('" + name + "') is not supported.");
		}
		return body;		
	}

	public void swapMesh(String fileName) {
		try {
			PolygonalMesh mesh = new PolygonalMesh(new File(fileName));
			body.setMesh(mesh, fileName);
		} catch (IOException e) {
			System.err.println("Error swapping mesh: could not read file " + fileName);
		}
	}

       @Override
       public ModelComponent getModelComponent () {
              return body;
       }

}

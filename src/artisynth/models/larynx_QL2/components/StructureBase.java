package artisynth.models.larynx_QL2.components;

import java.awt.Color;
import java.util.ArrayList;

import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.PlanarConnector;
import artisynth.core.mechmodels.RevoluteJoint;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.util.ArtisynthPath;
/**
 *  <p>
 *   @author Scott Moisik (Scott.Moisik@ntu.edu.sg), 2017, Nanyang Technological University, Singapore. 
 *   <p>
 *   If you use this model in your research, please cite: 
 *   Moisik, S. R., & Gick, B. (2017). The quantal larynx: the stable regions of laryngeal biomechanics and implications for speech production. Journal of Speech Language and Hearing Research, 60(3), 540-560. https://doi.org/10.1044/2016_JSLHR-S-16-0019

 *  **/
public abstract class StructureBase {
       //Global model properties
	protected static String dataDirectory = ArtisynthPath.getSrcRelativePath(StructureBase.class, "data/");
	
	protected static double mm = 1.0;
	protected static double mm_to_m = 1000.0;
	protected static double genericDamping = 0.25;
	protected static double genericStiffness = 5e1;

	protected static double muscleForceScaling = 10.0;
	protected static double muscleTendoRatio = 0.1;
	protected static double musclePassive = 0.1;
	protected static double muscleDamping = 1e-3;
	
	protected static Point3d right(Point3d point) {
		return new Point3d(-point.x, point.y, point.z);
	}

	protected static Point3d[] right(Point3d[] points) {
		Point3d[] rightPoints = new Point3d[points.length];
		int count = 0;
		for (Point3d point : points) {
			rightPoints[count++] = right(point);
		}
		return rightPoints;
	}

	protected static Point3d offsetX(Point3d point, double offset) {
		return new Point3d(point.x + offset, point.y, point.z);
	}

	protected static Point3d offsetY(Point3d point, double offset) {
		return new Point3d(point.x, point.y + offset, point.z);
	}

	protected static Point3d offsetZ(Point3d point, double offset) {
		return new Point3d(point.x, point.y, point.z + offset);
	}


      public static RigidTransform3d badinRotationTransform() {
             RigidTransform3d worldTransform = new RigidTransform3d();
             RigidTransform3d yRotationCorrection = new RigidTransform3d(new Vector3d(0.0, 0.0, 0.0), new AxisAngle(0.0, 1.0, 0.0, Math.PI/2));
             RigidTransform3d xRotationCorrection = new RigidTransform3d(new Vector3d(0.0, 0.0, 0.0), new AxisAngle(1.0, 0.0, 0.0, -Math.PI/2));

             worldTransform.mul(yRotationCorrection);
             worldTransform.mul(xRotationCorrection);

             return worldTransform;             
      }


       public static RigidTransform3d badinFaceTransform() {
              RigidTransform3d worldTransform = new RigidTransform3d();
              RigidTransform3d translationCorrection = new RigidTransform3d(0.0, 94.4, 56.4);
              RigidTransform3d yRotationCorrection = new RigidTransform3d(new Vector3d(0.0, 0.0, 0.0), new AxisAngle(0.0, 1.0, 0.0, Math.PI/2));
              RigidTransform3d xRotationCorrection = new RigidTransform3d(new Vector3d(0.0, 0.0, 0.0), new AxisAngle(1.0, 0.0, 0.0, -Math.PI/2));

              worldTransform.mul(translationCorrection);
              worldTransform.mul(yRotationCorrection);
              worldTransform.mul(xRotationCorrection);

              return worldTransform;             
       }

       public static RigidTransform3d badinTongueTransform() {
              RigidTransform3d worldTransform = new RigidTransform3d();
              RigidTransform3d translationCorrection = new RigidTransform3d(0.0, 55.7, 33.0);
              RigidTransform3d yRotationCorrection = new RigidTransform3d(new Vector3d(0.0, 0.0, 0.0), new AxisAngle(0.0, 1.0, 0.0, Math.PI/2));
              RigidTransform3d xRotationCorrection = new RigidTransform3d(new Vector3d(0.0, 0.0, 0.0), new AxisAngle(1.0, 0.0, 0.0, -Math.PI/2));

              worldTransform.mul(translationCorrection);
              worldTransform.mul(yRotationCorrection);
              worldTransform.mul(xRotationCorrection);

              return worldTransform;      

       }

       public static RigidTransform3d identityTransform() {
              return new RigidTransform3d();
       }

       public static RigidTransform3d mirrorTransform() {
              return new RigidTransform3d(0.0, 0.0, 0.0);
       }

       public static RigidTransform3d visibleHumanTransform() {
              return new RigidTransform3d(0.0, -70.0, -33.3);
       }

	
	protected static void planarTembromandibularJoint(MechModel mech, RigidBody mandible) {
		ArrayList<PlanarConnector> planes = new ArrayList<PlanarConnector>();

		Vector3d left = new Vector3d(-51.0, 120.0, -8.0);
		Vector3d right = new Vector3d(51.0, 120.0, -8.0);

		planes.add(new PlanarConnector(mandible, left, new RigidTransform3d(left, new AxisAngle(1.0, 0.0, 0.0, Math.toRadians(120)))));
		planes.add(new PlanarConnector(mandible, left, new RigidTransform3d(left, new AxisAngle(1.0, 0.0, 0.0, Math.toRadians(230)))));
		planes.add(new PlanarConnector(mandible, left, new RigidTransform3d(left, new AxisAngle(1.0, 0.0, 0.0, Math.toRadians(90)))));

		planes.add(new PlanarConnector(mandible, right, new RigidTransform3d(right, new AxisAngle(1.0, 0.0, 0.0, Math.toRadians(120)))));
		planes.add(new PlanarConnector(mandible, right, new RigidTransform3d(right, new AxisAngle(1.0, 0.0, 0.0, Math.toRadians(230)))));
		planes.add(new PlanarConnector(mandible, right, new RigidTransform3d(right, new AxisAngle(1.0, 0.0, 0.0, Math.toRadians(90)))));

		for (PlanarConnector plane : planes) {
			plane.setName("tmj");
			plane.setPlaneSize(20.0);
			mech.addBodyConnector(plane);
			RenderProps.setFaceStyle(plane, Renderer.FaceStyle.FRONT_AND_BACK);
			RenderProps.setAlpha(plane, 0.8);
		}
	}

	protected static void revoluteTembromandibularJoint(MechModel model, RigidBody mandible, RigidBody maxilla) {
		RigidTransform3d jointTransform = new RigidTransform3d(new Vector3d(0.0, 44.0, -41.0), new AxisAngle(0.0, 1.0, 0.0, -Math.PI/2));
		RigidTransform3d jawJointTransform = new RigidTransform3d();
		RigidTransform3d maxillaJointTransform = new RigidTransform3d();

		Point3d pmin = new Point3d();
		Point3d pmax = new Point3d();
		mandible.updateBounds(pmin, pmax);

		jawJointTransform.mulInverseLeft(mandible.getPose(), jointTransform);
		maxillaJointTransform.mulInverseLeft(maxilla.getPose(), jointTransform);
		RevoluteJoint tmjJoint = new RevoluteJoint(mandible, jawJointTransform, maxilla, maxillaJointTransform);
		tmjJoint.setMinTheta(-5);
		tmjJoint.setMaxTheta(30);
		tmjJoint.setName("tmj");
		tmjJoint.setAxisLength(1.0 * (pmax.x - pmin.x));

		//RenderProps.setLineSlices(tmjJoint, 12);
		RenderProps.setLineColor(tmjJoint, Color.ORANGE);
		RenderProps.setLineRadius(tmjJoint, 0.6);

		model.addBodyConnector(tmjJoint);                         
	}

	protected static void planarCricoarytenoidJoint(MechModel mech, HardBody cricoid, HardBody arytenoidLeft, HardBody arytenoidRight) {
		ArrayList<PlanarConnector> planes = new ArrayList<PlanarConnector>();

		Vector3d leftJoint = new Vector3d(-9.8, -7.9, -5.0);
		Vector3d rightJoint = new Vector3d(9.8, -7.9, -5.0);

		RigidTransform3d leftAnteriorTransform = new RigidTransform3d(leftJoint, new AxisAngle(1.0, 0.0, 0.0, Math.toRadians(130)));
		RigidTransform3d rightAnteriorTransform = new RigidTransform3d(rightJoint, new AxisAngle(1.0, 0.0, 0.0, Math.toRadians(130)));

		RigidTransform3d leftLateralTransform = new RigidTransform3d(leftJoint, new AxisAngle(0.0, 1.0, 1.0, Math.toRadians(-90)));
		RigidTransform3d rightLateralTransform = new RigidTransform3d(rightJoint, new AxisAngle(0.0, 1.0, 1.0, Math.toRadians(90)));

		planes.add(new PlanarConnector(arytenoidLeft.getBody(), leftJoint, cricoid.getBody(), leftAnteriorTransform));
		planes.add(new PlanarConnector(arytenoidRight.getBody(), rightJoint, cricoid.getBody(), rightAnteriorTransform));

		planes.add(new PlanarConnector(arytenoidLeft.getBody(), leftJoint, cricoid.getBody(), leftLateralTransform));
		planes.add(new PlanarConnector(arytenoidRight.getBody(), rightJoint, cricoid.getBody(), rightLateralTransform));

		for (PlanarConnector plane : planes) {
			plane.setName("cricoarytenoid joint ");
			plane.setPlaneSize(20.0);
			mech.addBodyConnector(plane);
			RenderProps.setFaceStyle(plane, Renderer.FaceStyle.FRONT_AND_BACK);
			RenderProps.setAlpha(plane, 0.8);
		}
	}

	/*
       public static void createPlanarBiteConstraint(MechModel mech, RigidBody mandible) {
              Plane leftBitePlane = new Plane(new Vector3d(-20.4, 77.8, 64.0), 0.0);
              Plane rightBitePlane = new Plane(new Vector3d(20.4, 77.8, 64.0), 0.0);

              FrameMarker leftMandibularOcclusionPoint = new FrameMarker("left mandibular occlusion point");
              FrameMarker rightMandibularOcclusionPoint = new FrameMarker("right mandibular occlusion point");

              mech.addFrameMarker(leftMandibularOcclusionPoint, mandible, new Point3d(-16.2, 70.9, 62.3));
              mech.addFrameMarker(rightMandibularOcclusionPoint, mandible, new Point3d(16.2, 70.9, 62.3));

              ParticlePlaneConstraint ppc = new ParticlePlaneConstraint(leftMandibularOcclusionPoint, leftBitePlane);
              PlanarConnector leftBitePlane = new PlanarConnector(mandible, left, new RigidTransform3d(, new AxisAngle(1.0, 0.0, 0.0, Math.toRadians(90))));
              PlanarConnector rightBitePlane = new PlanarConnector(mandible, right, new RigidTransform3d(new Vector3d(20.4, 77.8, 64.0), new AxisAngle(1.0, 0.0, 0.0, Math.toRadians(90))));
       }
	 */
}

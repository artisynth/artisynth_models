package artisynth.models.registration;

import artisynth.core.femmodels.FemNode;
import artisynth.core.femmodels.PointFem3dAttachment;
import artisynth.core.femmodels.PointSkinAttachment;
import artisynth.core.mechmodels.DynamicAttachment;
import artisynth.core.mechmodels.DynamicComponent;
import artisynth.core.mechmodels.Frame;
import artisynth.core.mechmodels.FrameAttachment;
import artisynth.core.mechmodels.Particle;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.PointAttachment;
import artisynth.core.mechmodels.PointFrameAttachment;
import artisynth.core.mechmodels.PointParticleAttachment;
import maspack.matrix.DenseMatrix;
import maspack.matrix.Matrix6dBlock;
import maspack.matrix.MatrixBlock;
import maspack.matrix.MatrixBlockBase;
import maspack.matrix.Point3d;
import maspack.matrix.SparseNumberedBlockMatrix;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorBase;
import maspack.matrix.VectorNd;
import maspack.spatialmotion.Wrench;

/**
 * Shared utilities for modifying force vectors and jacobians involving point attachments
 *
 */
public class AttachmentForceUtilities {

   /**
    * Add solve blocks associated with the pair of point attachments
    * @param M matrix to add blocks to
    * @param pa attachment a
    * @param pb attachment b
    */
   public static void addSolveBlocks(SparseNumberedBlockMatrix M, DynamicAttachment pa, DynamicAttachment pb) {

      DynamicComponent[] mastersa = pa.getMasters ();
      DynamicComponent[] mastersb = pb.getMasters ();
      
      for (int i=0; i<mastersa.length; ++i) {
         int bi = mastersa[i].getSolveIndex ();

         if (bi >= 0) {
            for (int j=0; j<mastersb.length; ++j) {
               
               int bj = mastersb[j].getSolveIndex ();
               if (bj >= 0) {
                  MatrixBlock blk = M.getBlock (bi, bj);
                  if (blk == null) {
                     blk =  MatrixBlockBase.alloc (mastersa[i].getVelStateSize (), mastersb[j].getVelStateSize ());
                     M.addBlock (bi, bj, blk);
                  }
               } // valid block indices
            } // j
         } // valid block index 
      } // masters 
   }
   
   /**
    * Multiplies block by vector
    * @param vr result, can be same as v
    * @param blk block to multiply vector by
    * @param v input vector
    */
   protected static void mulTranspose(VectorBase vr, MatrixBlock blk, Vector3d v) {
      double x = v.x;
      double y = v.y;
      double z = v.z;

      for (int i=0; i<blk.rowSize (); ++i) {
         double s = blk.get (i, 0)*x + blk.get (i, 1)*y + blk.get (i, 2)*z;
         vr.set (i, s);
      }
   }
   
   /**
    * Multiplies block by vector, NOT safe to have vr == v
    * @param vr result, can be same as v
    * @param blk block to multiply vector by
    * @param v input vector
    */
   protected static void mulTranspose(VectorBase vr, MatrixBlock blk, VectorBase v) {
      for (int i=0; i<blk.rowSize (); ++i) {
         double s = 0;
         for (int j = 0; j < blk.colSize (); ++j) {
            s += blk.get (i, j)*v.get (j);   
         }
         vr.set (i, s);
      }
   }
   
   /**
    * Adds a force applied at the attachment
    * @param p attachment point at which to apply force
    * @param s scale
    * @param f force vector
    */
   public static void addPointForce(PointAttachment p, Vector3d f) {

      // seems to be much faster by specifying
      if (p instanceof PointParticleAttachment) {
         // single node point attachment, add registration force
         PointParticleAttachment ppa = (PointParticleAttachment)p;
         Particle node = ppa.getParticle ();
         node.addForce (f);
      } else if (p instanceof PointFem3dAttachment) {
         // multi-node point attachment
         PointFem3dAttachment pfa = (PointFem3dAttachment)p;
         FemNode[] nodes = pfa.getNodes ();
         VectorNd coords = pfa.getCoordinates ();

         // distribute force across nodes
         for (int i=0; i<nodes.length; ++i) {
            nodes[i].addScaledForce (coords.get (i), f);
         }
      } else if (p instanceof PointFrameAttachment){
         // single frame attachment
         PointFrameAttachment pfa = (PointFrameAttachment)p;
         Frame frame = pfa.getFrame ();
         Point3d loc = pfa.getLocation ();
         frame.addPointForce (loc, f);

      } else if (p instanceof PointSkinAttachment) {
         ((PointSkinAttachment)p).addForce (f);
      } else {
         
         // generic case
         DynamicComponent[] dca = p.getMasters ();

         for (int a=0; a<dca.length; ++a) {
            MatrixBlock GTa = p.getGT(a);
            if (dca[a] instanceof Point) {
               Vector3d pf = new Vector3d();  // local force
               mulTranspose (pf, GTa, f);
               pf.negate();
               Point point = (Point) dca[a];
               point.addForce (pf);
            } else if (dca[a] instanceof Frame) {
               Wrench w = new Wrench ();
               mulTranspose (w, GTa, f);
               w.negate();
               Frame frame = (Frame) dca[a];
               frame.addForce (w);
            }
         }
      }
   }
   
   /**
    * Distributes a wrench to underlying components
    * @param fa attachment
    * @param f wrench
    */
   public static void addWrench(FrameAttachment fa, Wrench f) {
      
      // generic case
      DynamicComponent[] dca = fa.getMasters ();

      for (int a=0; a<dca.length; ++a) {
         MatrixBlock GTa = fa.getGT(a);
         if (dca[a] instanceof Point) {
            Vector3d pf = new Vector3d();  // local force
            mulTranspose (pf, GTa, f);
            pf.negate();
            Point point = (Point) dca[a];
            point.addForce (pf);
         } else if (dca[a] instanceof Frame) {
            Wrench w = new Wrench ();
            mulTranspose (w, GTa, f);
            w.negate();
            Frame frame = (Frame) dca[a];
            frame.addForce (w);
         }
      }
      
   }
   

   /**
    * Adds a block multiplication, br += s*bi*bj^T
    * @param br block to add to
    * @param s scale
    * @param bi first block
    * @param bj second block
    */
   protected static void addScaledMulTransposeRight(MatrixBlock br, double s, MatrixBlock bi, MatrixBlock bj) {
      for (int i=0; i<bi.rowSize (); ++i) {
         for (int j = 0; j<bj.rowSize (); ++j) {
            double sij = br.get (i, j);
            for (int k = 0; k < bi.colSize (); ++k) {
               sij += s*bi.get (i, k)*bj.get (j, k);
            }
            br.set(i, j, sij);
         }
      }
   }

   /**
    * Given two attachments with velocity interpolation matrices 
    * v_a = Phi_a v and v_b = Phi_b v, adds the product Phi_a^T Phi_b
    * to the matrix M
    * @param M target matrix
    * @param pa first attachment
    * @param pb second attachment
    */
   public static void addScaledMulTranspose(SparseNumberedBlockMatrix M, double s, DynamicAttachment pa, DynamicAttachment pb) {

      if (pa instanceof PointAttachment && pb instanceof PointAttachment) {
         Point pointa = ((PointAttachment)pa).getPoint ();
         Point pointb = ((PointAttachment)pb).getPoint ();
   
         // special cases, faster multiplication
         if ( pointa == null && pointb == null) {
            // working with a simpler vertex attachment
            if (pa instanceof PointFrameAttachment && pb instanceof PointFrameAttachment) {
               PointFrameAttachment pfa = (PointFrameAttachment)pa;
               PointFrameAttachment pfb = (PointFrameAttachment)pb;
   
               if (pfa == pfb) {
                  // single frame attachment
                  Frame frame = pfa.getFrame ();
                  int bi = frame.getSolveIndex ();
   
                  Matrix6dBlock blk = (Matrix6dBlock)M.getBlock (bi, bi);
                  Point3d loc = pfa.getLocation ();
   
                  // J =    s   [  I  -[Rl] ]
                  //            [ [Rl]  0   ]
   
                  blk.m00 += s;
                  blk.m11 += s;
                  blk.m22 += s;
   
                  Vector3d li = new Vector3d();
                  frame.getPose ().R.mul (li, loc);
                  li.scale (s);
   
                  blk.m04 += li.z;
                  blk.m05 -= li.y;
                  blk.m13 -= li.z;
                  blk.m15 += li.x;
                  blk.m23 += li.y;
                  blk.m24 -= li.x;
   
                  blk.m40 += li.z;
                  blk.m50 -= li.y;
                  blk.m31 -= li.z;
                  blk.m51 += li.x;
                  blk.m32 += li.y;
                  blk.m42 -= li.x;
   
                  return;
               }
            }
   
         }
      }
   
      // generic case
      DynamicComponent[] dca = pa.getMasters ();
      DynamicComponent[] dcb = pb.getMasters ();
      for (int a=0; a<dca.length; ++a) {
         int blocka = dca[a].getSolveIndex ();
         if (blocka >= 0) {
            MatrixBlock GTa = pa.getGT (a);
            for (int b=0; b<dcb.length; ++b) {
               int blockb = dcb[b].getSolveIndex ();
               if (blockb >= 0) {
                  MatrixBlock GTb = pb.getGT (b);
                  MatrixBlock bij = M.getBlock (blocka, blockb);
                  addScaledMulTransposeRight (bij, s, GTa, GTb);
               }
            }
         }
      }      
     
   }
   
  /**
    * Multiplies and adds M += A W B^T
    * @param M output matrix, must be different from A, B, and W
    * @param A matrix to multiply by on left
    * @param B matrix to multiply by on right
    * @param W middle matrix
    */
   public static void addMulLeftAndTransposeRight(DenseMatrix M, DenseMatrix A, DenseMatrix W, DenseMatrix B) {
      int m = A.rowSize ();
      int n = W.rowSize ();
      
      VectorNd v = new VectorNd(n);
      
      for (int r = 0; r < m; ++r) {
         // A W
         for (int i = 0; i < n; ++i) {
            double s = 0;
            for (int j = 0; j < n; ++j) {
               s += A.get (r, j) * W.get (j, i);
            }
            v.set (i, s);
         }
         
         // A W B^T
         for (int c = 0; c < m; ++c) {
            double s = 0;
            for (int j = 0; j < n; ++j) {
               s += v.get (j) * B.get (c, j);
            }
            
            // add to element
            M.set (r, c, s + M.get (r, c));
         }
      }
      
   }
   
   /**
    * Given two attachments with velocity interpolation matrices 
    * v_a = Phi_a v and v_b = Phi_b v, adds the product Phi_a^T W Phi_b
    * to the matrix M
    * @param M target matrix
    * @param pa first attachment
    * @param pb second attachment
    */
   public static void addMulTransposeLeftAndRight(SparseNumberedBlockMatrix M, DynamicAttachment pa, DenseMatrix W, DynamicAttachment pb) {
   
      // generic case
      DynamicComponent[] dca = pa.getMasters ();
      DynamicComponent[] dcb = pb.getMasters ();
      for (int a=0; a<dca.length; ++a) {
         int blocka = dca[a].getSolveIndex ();
         if (blocka >= 0) {
            MatrixBlock GTa = pa.getGT (a);
            for (int b=0; b<dcb.length; ++b) {
               int blockb = dcb[b].getSolveIndex ();
               if (blockb >= 0) {
                  MatrixBlock GTb = pb.getGT (b);
                  MatrixBlock bij = M.getBlock (blocka, blockb);
                  addMulLeftAndTransposeRight (bij, GTa, W, GTb);
               }
            }
         }
      }
   
   }
  
   
}

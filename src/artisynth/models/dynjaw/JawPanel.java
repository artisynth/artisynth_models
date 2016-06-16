package artisynth.models.dynjaw;

import java.awt.Color;

import javax.swing.JSeparator;

import maspack.properties.Property;
import maspack.render.RenderProps;
import maspack.render.Renderer.LineStyle;
import maspack.widgets.BooleanSelector;
import maspack.widgets.DoubleField;
import artisynth.core.driver.Main;
import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.modelbase.ModelComponent;

public class JawPanel
{

   public static void addBodyDampingControls(JawModel jaw,
      String bodyName, ControlPanel panel, double[] rotLimits,
      double[] transLimits)
   {
      if (panel == null
         || jaw.findComponent("rigidBodies/" + bodyName) == null)
      {
         return;
      }
      panel.addWidget(new JSeparator());

      panel.addWidget(bodyName.toUpperCase() + " Rot Damping", jaw,
         "rigidBodies/" + bodyName + ":rotaryDamping", rotLimits[0],
         rotLimits[1]);
      panel.addWidget(bodyName.toUpperCase() + " Trans Damping", jaw,
         "rigidBodies/" + bodyName + ":frameDamping", transLimits[0],
         transLimits[1]);

      // panel.addWidget (bodyName.toUpperCase() + " Rot Damping",
      // jaw, "dampers/"+bodyName+"/rotationalDamping",
      // rotLimits[0], rotLimits[1]);
      // panel.addWidget (bodyName.toUpperCase() + " Trans Damping",
      // jaw, "dampers/"+bodyName+"/translationalDamping",
      // transLimits[0], transLimits[1]);
   }

   public static void addHyoidDepressorControls(JawModel jaw, ControlPanel panel)
   {
      if (panel == null)
         return;
      panel.addWidget(new JSeparator());

      panel.addWidget(jaw.getMuscleName("bi_hd"), jaw,
         "exciters/bi_infrahyoid:excitation", 0.0, 1.0);

   }

   public static void addPharynxStiffnessControl(JawModel jaw,
      ControlPanel panel)
   {
      if (panel == null || jaw.axialSprings().get("lphc") == null
         || jaw.axialSprings().get("rphc") == null)
         return;
      panel.addWidget(new JSeparator());

      panel.addWidget("LeftPharynxConstrict", jaw,
         "axialSprings/lphc:stiffness", 0.0, 1000.0);
      panel.addWidget("RightPharynxConstrict", jaw,
         "axialSprings/rphc:stiffness", 0.0, 1000.0);

   }
   
   public static void addGroupedMuscleControls(JawModel jaw, ControlPanel panel)
   {
      if (panel == null)
         return;
      panel.addWidget(new JSeparator());

      String[] muscleNames = new String[]{
         "bi_open",
         "bi_close",
      };
      
      for (int i = 0; i < muscleNames.length; i++)
         {
            panel.addWidget(jaw.getMuscleName(muscleNames[i]), jaw, 
               "exciters/"+muscleNames[i]+":excitation",
               0.0, 1.0);
         }
   }
   public static void addMuscleControls(JawModel jaw, ControlPanel panel)
   {
      if (panel == null)
         return;
      panel.addWidget(new JSeparator());

      String[] muscleNames = new String[]{
         "bi_ad",
         "bi_ip",
         "bi_mp",
         "bi_at",
         "bi_pt"
      };
      
      for (int i = 0; i < muscleNames.length; i++)
         {
            panel.addWidget(jaw.getMuscleName(muscleNames[i]), jaw, 
               "exciters/"+muscleNames[i]+":excitation",
               0.0, 1.0);
         }
   }

//   public static void setRightSideCoactivation(JawModelX jaw, String name)
//   {
//      JawMuscle masterMuscle, pairedMuscle;
//      masterMuscle = (JawMuscle) jaw.axialSprings().get("l" + name);
//      pairedMuscle = (JawMuscle) jaw.axialSprings().get("r" + name);
//      if (masterMuscle != null && pairedMuscle != null)
//         masterMuscle.addCoactivator(pairedMuscle);
//   }

   public static void addEnableMusclesControl(JawModel jaw,
      ControlPanel panel)
   {
      if (panel == null)
         return;
      panel.addWidget(new JSeparator());

      panel.addWidget("Enable Muscles", jaw, "enableMuscles");
   }

   public static void addFoodBolusControls(JawModel jaw, String bolusName,
      ControlPanel panel)
   {
      if (panel == null)
         return;
      panel.addWidget(new JSeparator());
      
//      BooleanSelector checkBox;
//      DoubleField doubleBox1, doubleBox2;

      
      ModelComponent bolus = jaw.findComponent("forceEffectors/" + bolusName);

      if (bolus != null)
      {
         panel.addWidget(bolusName.toUpperCase() + ": Diameter (mm)", bolus,
            "diameter");
         panel.addWidget(bolusName.toUpperCase() + ": Resistance (N)", bolus,
            "resistance");
         panel.addWidget("isActive", bolus, "active");
      }

      // if (bolus != null)
      // {
      // doubleBox1 = (DoubleField)PropertyWidget.create (bolus, "diameter");
      // doubleBox1.setLabelText (bolusName.toUpperCase()+": Diameter (mm)");
      // doubleBox1.setFormat ("%4.1f");
      // doubleBox1.addValueChangeListener(new ValueChangeListener(){
      // public void valueChange(ValueChangeEvent e)
      // { rerender();
      // }
      // });
      //
      // doubleBox2 = (DoubleField)PropertyWidget.create (bolus, "resistance");
      // doubleBox2.setLabelText ("Max Resistance (N)");
      // doubleBox2.setFormat ("%4.1f");
      //
      // checkBox = (BooleanSelector)PropertyWidget.create (bolus, "active");
      // checkBox.setLabelText ("isActive");
      // checkBox.addValueChangeListener(new ValueChangeListener(){
      // public void valueChange(ValueChangeEvent e)
      // { rerender();
      // }
      // });
      //
      // doubleBox1.add(doubleBox2);
      // doubleBox1.add(checkBox);
      // panel.addWidget (doubleBox1);
      // }
   }

   public static void addMemStiffnessControls(JawModel jaw,
      ControlPanel panel)
   {
      if (panel == null)
         return;
      panel.addWidget(new JSeparator());

      panel.addWidget("TH Mem Stiffness", jaw, "thMemStiffness", 0.0,
         1000.0);
      panel.addWidget("CTr Mem Stiffness", jaw, "ctrMemStiffness", 0.0,
         1000.0);

      panel.addWidget(new JSeparator());
   }

   public static void addSimulationControls(JawModel jaw,
      ControlPanel panel)
   {
      if (panel == null)
         return;
      panel.addWidget(new JSeparator());
      panel.addWidget("Integration Method", jaw, "integrator");
      panel.addWidget("Maximum Step Size", jaw, "maxStepSize");
      // panel.addWidget ("Matrix Solver", jaw, "matrixSolver");
   }

   public static void addMuscleTypeControls(JawModel jaw,
      ControlPanel panel)
   {
      if (panel == null)
         return;
      panel.addWidget(new JSeparator());
      panel.addWidget("Muscle Type", jaw, "jawMusclesType");
   }

   public static void addRenderControls(JawModel jaw, ControlPanel panel)
   {

      panel.addWidget("Skeleton  vis", jaw, "renderFaces");
      // panel.addWidget("Edges: Visible", jaw, "renderEdges");
      // panel.addWidget("Transparency On: ", jaw, "transparency");
      panel.addWidget("Tissue  vis", jaw, "memShow");
      panel.addWidget("Muscles  vis", jaw, "musShow");
      panel.addWidget("Constraint vis", jaw, "planesVisible");
      if (jaw.findComponent("rigidBodies/pharynx") != null)
      {
         panel.addWidget("Pharynx  vis", jaw,
            "rigidBodies/pharynx:renderProps.visible");
      }
   }

   public static void addBodyDynamicControls(JawModel jaw,
      String bodyName, ControlPanel panel)
   {
      if (panel == null)
         return;
      panel.addWidget(new JSeparator());

      if (jaw.findComponent("rigidBodies/" + bodyName) != null)
         panel.addWidget(bodyName.toUpperCase() + " Dynamic", jaw,
            "rigidBodies/" + bodyName + ":dynamic");
   }

   public static void addUnilateralConstraintBox(JawModel jaw,
      String conName, ControlPanel panel)
   {
      if (panel == null)
         return;

      if (jaw.findComponent("bodyConnectors/" + conName) != null)
         panel.addWidget(conName.toUpperCase() + " Disarticulated", jaw,
            "bodyConnectors/" + conName + ":unilateral");
   }

   public static void addHeadRotationControls(JawModel jaw,
      ControlPanel panel)
   {
      if (panel == null)
         return;
      panel.addWidget(new JSeparator());
      panel.addWidget("Head Angle", jaw, "headRotation", -30.0, 30.0);
   }

   public static void addIncisorTrace(JawModel jaw, ControlPanel panel)
   {
      if (panel == null)
         return;
      FrameMarker inc = jaw.frameMarkers().get("lowerincisor");
      if (inc == null)
         return;
      panel.addWidget(new JSeparator());
      panel.addWidget(jaw, "frameMarkers/lowerincisor:tracing");
   }

   public static void createJawTonguePanel(JawModel jaw, ControlPanel panel)
   {
      if (panel == null)
         return;
      addSimulationControls(jaw, panel);
      addRenderControls(jaw, panel);
      addBodyDynamicControls(jaw, "jaw", panel);
      addBodyDynamicControls(jaw, "hyoid", panel);
      // addJawTongueDampingControls(jaw, panel);
      addBodyDampingControls(jaw, "jaw", panel, new double[]
      { 0.0, 20000.0 }, new double[]
      { 0.0, 40.0 });
      addBodyDampingControls(jaw, "hyoid", panel, new double[]
      { 0.0, 20000.0 }, new double[]
      { 0.0, 40.0 });
      addHeadRotationControls(jaw, panel);
      addMuscleTypeControls(jaw, panel);
//      addMuscleControls(jaw, panel);
      addGroupedMuscleControls(jaw, panel);
      addPharynxStiffnessControl(jaw, panel);
      addMemStiffnessControls(jaw, panel);
      addIncisorTrace(jaw, panel);
      panel.setScrollable (true);
   }

   public static void createJawPanel(JawModel jaw, ControlPanel panel)
   {
      if (panel == null)
         return;
      addSimulationControls(jaw, panel);
      addMuscleTypeControls(jaw, panel);
      addEnableMusclesControl(jaw, panel);
      addRenderControls(jaw, panel);
      addBodyDynamicControls(jaw, "jaw", panel);
      // addFoodBolusControls(jaw, "rightbolus", panel);
      // addFoodBolusControls(jaw, "leftbolus", panel);
      addBodyDampingControls(jaw, "jaw", panel, new double[]
      { 0.0, 1000.0 }, new double[]
      { 0.0, 20.0 });
      addHeadRotationControls(jaw, panel);
      addMuscleControls(jaw, panel);
      addGroupedMuscleControls(jaw, panel);
      addIncisorTrace(jaw, panel);
      panel.addWidget(new JSeparator());
      addUnilateralConstraintBox(jaw, "LTMJ", panel);
      addUnilateralConstraintBox(jaw, "RTMJ", panel);
      panel.setScrollable (true);
   }

   public static void createJawLarynxPanel(JawModel jaw, ControlPanel panel)
   {
      if (panel == null)
         return;

      addSimulationControls(jaw, panel);
      addMuscleTypeControls(jaw, panel);
      addEnableMusclesControl(jaw, panel);

      addRenderControls(jaw, panel);

      addBodyDynamicControls(jaw, "jaw", panel);
      addBodyDynamicControls(jaw, "hyoid", panel);
      addBodyDynamicControls(jaw, "thyroid", panel);
      addBodyDynamicControls(jaw, "cricoid", panel);

      addFoodBolusControls(jaw, "rightbolus", panel);
      addFoodBolusControls(jaw, "leftbolus", panel);

      addBodyDampingControls(jaw, "jaw", panel, new double[]
      { 0.0, 20000.0 }, new double[]
      { 0.0, 40.0 });
      addBodyDampingControls(jaw, "hyoid", panel, new double[]
      { 0.0, 20000.0 }, new double[]
      { 0.0, 40.0 });
      addHeadRotationControls(jaw, panel);
//      addHyoidDepressorControls(jaw, panel);
      addMemStiffnessControls(jaw, panel);

      addUnilateralConstraintBox(jaw, "LTMJ", panel);
      addUnilateralConstraintBox(jaw, "RTMJ", panel);
      addIncisorTrace(jaw, panel);
      panel.addWidget(new JSeparator());
      panel.setScrollable (true);
   }

}

package artisynth.models.tubesounds;

import java.awt.Dimension;
import java.awt.LayoutManager;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JSeparator;
import javax.swing.JTextArea;
import javax.swing.SwingConstants;

import maspack.widgets.StringField;
import artisynth.core.util.ArtisynthPath;

public class TextSpeechDlg extends JFrame implements ActionListener
{
   private JPanel mainPane;
   private StringField wordBox;
   private StringField loadFileNameBox;
   private StringField saveFileNameBox;
   //private JButton okBtn;
   private JButton loadBtn;
   private JButton saveBtn;
   //private JButton cancelBtn;
   private VTNTDemo myParent;
   
   private VoiceProbeMaker probeMaker;
   
   public TextSpeechDlg(VTNTDemo parent)
   {
      myParent = parent;
      this.setTitle("Word Synthesizer");
      mainPane = new JPanel();
      mainPane.setLayout(new BoxLayout(mainPane, BoxLayout.Y_AXIS));
      wordBox = new StringField("Word to Synthesize", 12);
      wordBox.setAlignmentX(CENTER_ALIGNMENT);
      
      loadFileNameBox = new StringField("Input file base name", 12);
      loadFileNameBox.setValue("HI");
      loadFileNameBox.setAlignmentX(CENTER_ALIGNMENT);
      
      saveFileNameBox = new StringField("Output file base name", 12);
      saveFileNameBox.setAlignmentX(CENTER_ALIGNMENT);
      
      //okBtn = new JButton("Synthesize word");
      //okBtn.setAlignmentX(CENTER_ALIGNMENT);
      
      loadBtn = new JButton("Load Input File");
      loadBtn.setAlignmentX(CENTER_ALIGNMENT);
      
      saveBtn = new JButton("Save Output File");
      saveBtn.setAlignmentX(CENTER_ALIGNMENT);
      
      //JLabel jlabel = new JLabel("Usage: enter base name of probe files you want to load, then hit \"Load Input File\"");
      //jlabel.setHorizontalAlignment(SwingConstants.LEFT);
      //mainPane.add(jlabel);
      JTextArea loadtext = new JTextArea();
      //loadtext.setSize(30, 15);
      loadtext.append("Usage: enter base name of probe files you want to load, then hit \"Load Input File\"." +
          "It expects *_glot.txt, *_lung.txt, *_pitch.txt, and *_tube.txt files.");
      loadtext.setEditable(false);
      loadtext.setLineWrap(true);
      loadtext.setWrapStyleWord(true);
      mainPane.add(loadtext);
      //jlabel.setBounds(0, y, width, height)
      //jlabel.setAlignmentX(LEFT_ALIGNMENT);
      //jlabel.setHorizontalTextPosition(SwingConstants.LEFT);
      
      mainPane.add(Box.createRigidArea(new Dimension(0, 10)));
      
      mainPane.add(loadFileNameBox);
      mainPane.add(Box.createRigidArea(new Dimension(0, 10)));
      mainPane.add(loadBtn);
      mainPane.add(Box.createRigidArea(new Dimension(0, 10)));
      
      
      mainPane.add(new JSeparator(SwingConstants.HORIZONTAL));
      mainPane.add(Box.createRigidArea(new Dimension(0, 10)));
      
      JTextArea synthtext = new JTextArea(
          "Usage: enter word you would like to synthesize in provided box," + 
          "and base name of probe files you want to create, then hit \"Save Output File\"." +
          "Probes are created in file, but not automatically loaded.");
      synthtext.setEditable(false);
      synthtext.setLineWrap(true);
      synthtext.setWrapStyleWord(true);
      mainPane.add(synthtext);
      mainPane.add(Box.createRigidArea(new Dimension(0, 10)));
      
      mainPane.add(wordBox);
      mainPane.add(Box.createRigidArea(new Dimension(0, 10)));
      mainPane.add(saveFileNameBox);
      mainPane.add(Box.createRigidArea(new Dimension(0, 10)));
      mainPane.add(saveBtn);
      mainPane.add(Box.createRigidArea(new Dimension(0, 10)));
      
      
      //mainPane.add(okBtn);
      //okBtn.addActionListener(this);
      
      loadBtn.addActionListener(this);
      saveBtn.addActionListener(this);
      mainPane.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
      this.add(mainPane);
      this.pack();
      //mainPane.setSize(70, 150);
   }
   
   public void actionPerformed(ActionEvent e)
   {
      if (e.getActionCommand() == "Save Output File")
      {
	 String baseName= ArtisynthPath.getSrcRelativePath(
                  this, saveFileNameBox.getStringValue());
	 VoiceProbeMaker pm = new VoiceProbeMaker(wordBox.getStringValue(), baseName);
	 if (pm.makeProbe())
	 {
	    System.out.println("success! probes created for word \""+wordBox.getStringValue()+"\"");
	    try
	    {
	       myParent.loadInputProbeFiles(baseName);
	    }
	    catch (Exception ex)
	    {
	       JOptionPane.showMessageDialog(this, ex.getMessage(), "ERROR!", JOptionPane.ERROR_MESSAGE);
	    }
	    
	 }
	 else
	 {
	    System.out.println("error! probes not created");
	 }
      } 
      else if (e.getActionCommand() == "Load Input File")
      {
	 String baseName= ArtisynthPath.getSrcRelativePath(
                  this, loadFileNameBox.getStringValue());
	 try
	 {
	    myParent.loadInputProbeFiles(baseName);
	 }
	 catch (Exception ex)
	 {
	    JOptionPane.showMessageDialog(this, ex.getMessage(), "ERROR!", JOptionPane.ERROR_MESSAGE);
	 }
      }
   }
   
}
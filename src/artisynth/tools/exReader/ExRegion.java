package artisynth.tools.exReader;

import java.util.ArrayList;
import java.util.Collections;

public class ExRegion {
   String name;
   private ArrayList<ExNode> myNodes;
   private ArrayList<ExElement> myElements;
   private ArrayList<ExFace> myFaces;
   private ArrayList<ExLine> myLines;

   public ExRegion(String path) {
      name = path;
      myNodes = new ArrayList<ExNode>();
      myLines = new ArrayList<ExLine>();
      myElements = new ArrayList<ExElement>();
      myFaces = new ArrayList<ExFace>();
   }

   public ExRegion(ArrayList<ExElement> elements) {
      this("");
      setFromElements(elements);
   }

   public void setFromElements(ArrayList<ExElement> elements) {
      myElements.clear();
      for (ExElement elem : elements) {
	 if (!myElements.contains(elem)) {
	    myElements.add(elem);
	 }
      }
      updateLists();
   }

   /**
    * Sorts nodes, elements and lines
    * Nodes by  number of derivatives, number of versions
    * Elements by #components, interpolation type, shape, number scale factors, versions per node
    * Faces/Lines by reconstructing from element order
    */
   public void sort() {
      sortElements();
      setFromElements(myElements);  // regenerate nodes/surfaces
      sortNodes();
   }
   
   public void renumber() {
      
      for (int i=0; i<myNodes.size(); i++) {
         myNodes.get(i).setIdx(i+1);
      }
      for (int i=0; i<myElements.size(); i++) {
         myElements.get(i).setIdx(i+1);
      }
      for (int i=0; i<myFaces.size(); i++) {
         myFaces.get(i).setIdx(i+1);
      }
      for (int i=0; i<myLines.size(); i++) {
         myLines.get(i).setIdx(i+1);
      }
      
   }
   
   public void sortNodes() {
      Collections.sort(myNodes, new ExNodeComparator());
   }
   
   public void sortElements() {
      Collections.sort(myElements, new ExElementComparator());
   }
   
   public void updateDependencies() {
      
      // face->element dependencies
      for (ExFace face : myFaces) {
	 ArrayList<ExElement> oldDeps = new ArrayList<ExElement>();
	 oldDeps.addAll(face.getDependentElements());
	 
	 for (ExElement elem : oldDeps) {
	    if (!myElements.contains(elem)) {
	       face.removeDependentElement(elem);
	    }
	 }
      }
      
      // line->face dependencies
      for (ExLine line : myLines) {
	 ArrayList<ExFace> oldDeps = new ArrayList<ExFace>();
	 oldDeps.addAll(line.getDependenFaces());
	 
	 for (ExFace face : oldDeps) {
	    if (!myFaces.contains(face)) {
	       line.removeDependentFace(face);
	    }
	 }
      }
      
      // node->line dependencies
      for (ExNode node : myNodes) {
	 ArrayList<ExLine> oldDeps = new ArrayList<ExLine>();
	 oldDeps.addAll(node.getDependentLines());
	 
	 for (ExLine line : oldDeps) {
	    if (!myLines.contains(line)) {
	       node.removeDependentLine(line);
	    }
	 }
      }
      
   }
   
   public void updateLists() {
      myNodes.clear();
      myFaces.clear();
      myLines.clear();
      
      // go through all elements
      for (ExElement elem : myElements) {
	 
	 // add all faces from those elements
	 for (ExFace face : elem.getFaces()) {
	    if (face != null) {
	       if (!myFaces.contains(face)) {
		  myFaces.add(face);
		  
		  // add all lines from those faces
		  for (ExLine line : face.getLines()) {
		     if (line != null) {
			if (!myLines.contains(line)) {
			   myLines.add(line);
			   
			   // add all nodes from those lines
			   for (ExNode node : line.getNodes()) {
			      if (node != null) {
				 if (!myNodes.contains(node)) {
				    myNodes.add(node);
				 }
			      } // end null node
			   } // end looping through nodes
			} // end adding line
		     } // end null line
		  } // end looping through lines
	       }
	    } // end add face
	 } // end looping through faces
      } // end looping through elements

   }	

   public ArrayList<ExElement> getElements() {
      return myElements;
   }

   public ArrayList<ExFace> getFaces() {
      return myFaces;
   }

   public ArrayList<ExLine> getLines() {
      return myLines;
   }

   public ArrayList<ExNode> getNodes() {
      return myNodes;
   }

   public String getName() {
      return name;
   }

   public boolean addNode(ExNode node) {
      if (!myNodes.contains(node)) {
	 myNodes.add(node);
	 return true;
      }
      return false;
   }

   public boolean addElement(ExElement elem) {
      if (!myElements.contains(elem)) {
	 myElements.add(elem);
	 return true;
      }
      return false;
   }

   public boolean addFace(ExFace face) {
      if (!myFaces.contains(face)) {
	 myFaces.add(face);
	 return true;
      }
      return false;
   }

   public boolean addLine(ExLine line) {
      if (!myLines.contains(line)) {
	 myLines.add(line);
	 return true;
      }
      return false;
   }

}


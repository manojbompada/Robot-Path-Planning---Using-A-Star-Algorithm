
import java.awt.Color;
import java.awt.FontMetrics;
import javax.swing.JPanel;
import java.awt.Graphics;
import java.awt.geom.Line2D;
import java.util.ArrayList;


public class GUILayoutClass extends JPanel {

    private static final long serialVersionUID = 1L;
    public static final double scale = 65; 
    public final static double checkpoint_bias = 1000000.0;
    private final int scale_X = 400;
    private final int scale_Y = 400;
    private final int vertex_width = 5; // node width
    private final int vertex_height = 5; // node height
    public final static double robot = 0.2; // robot size
    public final static double limit = 0.01;
    public final static double bias = 10000000000.0;
	

    private ArrayList<Vertex> vertices; 
    private ArrayList<Edge> edgeList; 

   
    public GUILayoutClass() {

        vertices = new ArrayList<Vertex>();
        edgeList = new ArrayList<Edge>();

    }

    /**
     * This method creates start point of robot
     */
    public void startpoint(double x, double y) {
        vertices.add(new Vertex("start_location", (int) (x * scale) + scale_X,
                (int) (y * scale) + scale_Y, Color.LIGHT_GRAY));
        this.repaint();
    }

    /**
     * This method creates desired goal point
     *
     */
    public void goalpoint(double x, double y) {
        vertices.add(new Vertex("goal_location", (int) (x * scale) + scale_X,
                (int) (y * scale) + scale_Y, Color.MAGENTA));
        this.repaint();
    }

    /**
     * This method will create obstacle map 
     */
    public void createPolyObs(ArrayList<Line2D> poly_Edges) {
        for (Line2D edge : poly_Edges) {
            edgeList.add(new Edge(edge, Color.BLACK));
        }
        this.repaint();
    }

    /**
     *This method will create expanded polygon obstacle map
     */
    public void createAltPolyObs(ArrayList<Line2D> alt_obsEdges) {
        for (Line2D edge : alt_obsEdges) {
            edgeList.add(new Edge(edge, Color.GRAY));
        }
        this.repaint();
    }

    /**
     * This method will add the lines corresponding to visibility edges
     * 
     */
    public void visibilitygraph(ArrayList<Line2D> visEdges) {
        for (Line2D edge : visEdges) {
            edgeList.add(new Edge(edge, Color.ORANGE));
        }
        this.repaint();
    }

    /**
     * This method adds the path of robot from start to goals node
   
     */
    public void robot_path(ArrayList<Line2D> robo_path) {
        for (Line2D edge : robo_path) {
            edgeList.add(new Edge(edge, Color.BLUE));
        }
        this.repaint();
    }


    public void paintComponent(Graphics gr) {

        FontMetrics fm = gr.getFontMetrics();

        gr.clearRect(gr.getClipBounds().x, gr.getClipBounds().y,
                gr.getClipBounds().width, gr.getClipBounds().height);
        
        // This will form all the vertices
        for (int i = 0; i < vertices.size(); i++) {

            Vertex vert = vertices.get(i);

            gr.setColor(vert.vertex_color);
            gr.fillOval(vert.x_pos - vertex_width / 2, vert.y_pos - vertex_height / 2,
                    vertex_width, vertex_height);
            gr.setColor(Color.BLACK);
            gr.drawOval(vert.x_pos - vertex_width / 2, vert.y_pos - vertex_height / 2,
                    vertex_width, vertex_height);

            gr.drawString(vert.vertex_name, vert.x_pos - fm.stringWidth(vert.vertex_name) / 2,
                    vert.y_pos + fm.getHeight());
        }

        // This will form all the edges in the edge list
        for (int i = 0; i < edgeList.size(); i++) {

            Edge edge = edgeList.get(i);
            gr.setColor(edge.edge_color);
            gr.drawLine((int) (edge.edge.getX1() * scale) + scale_X,
                    (int) (edge.edge.getY1() * scale) + scale_Y,
                    (int) (edge.edge.getX2() * scale) + scale_X,
                    (int) (edge.edge.getY2() * scale) + scale_Y);
        }

      
    }

    private class Vertex {

        private int x_pos, y_pos;
        private String vertex_name;
        private Color vertex_color;

        public Vertex(String vname, int x_loc, int y_loc, Color vert_color) {

            vertex_color = vert_color;
            vertex_name = vname;
            x_pos = x_loc;
            y_pos = y_loc;

        }
    }

    private class Edge {

        private Line2D edge;
        private Color edge_color;

        public Edge(Line2D edg, Color e_color) {
            this.edge = edg;
            this.edge_color = e_color;
        }
    }

}
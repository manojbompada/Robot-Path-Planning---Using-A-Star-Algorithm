import java.io.*;
import java.util.*;
import java.awt.*;
import java.awt.geom.*;
import javax.swing.*;
// This class maintains the polygonal obstacles
public class PolygonalObstacle
{
	
	public int vertex_count = 0;
	private final double dia_robot = GUILayoutClass.robot; 
	private final double cut_off = GUILayoutClass.limit;
	public boolean obs_map_outline = false;
	private final double setbias = GUILayoutClass.bias;
	public static final double check_bias = GUILayoutClass.checkpoint_bias;
	public ArrayList<Line2D> polygon_edges = new ArrayList<Line2D>();
	public ArrayList<Line2D> resized_hullEdges = new ArrayList<Line2D>();
	public ArrayList<Point2D > alteredvertices = new ArrayList< Point2D>();
	public ArrayList<Point2D> hullvertexpoints = new ArrayList<Point2D>();
	
	
	public ArrayList<Point2D> poly_vertices = new ArrayList<Point2D>();

	/**
	 * constructor for a polygonal obststacles which has edges, vertices and corners count
	 */
	public PolygonalObstacle(ArrayList<Line2D> edges, ArrayList<Point2D> vertices, int vertex_cnt)
	{
		polygon_edges = edges;
		poly_vertices = vertices;
		vertex_count = vertex_cnt;
	}

	public boolean isOutline() {
		return obs_map_outline;
	}

	public void setOutline(boolean isOutline) {
		this.obs_map_outline = isOutline;
	}
	
	public Polygon alteredPolygon = new Polygon();
	public ArrayList<Point2D> alteredhullnodes = new ArrayList<Point2D>();
	
	/**
	 * This method will increase the size of original polygonal obstacles to a size which is
	 * compatible with robot size
	 */
	public void alterObstacles()
	{
		for( int cnt=0; cnt<polygon_edges.size(); cnt++ )
		{
			Line2D edge1 = polygon_edges.get(cnt);
			Line2D edge2 = polygon_edges.get( ( cnt + 1 ) % polygon_edges.size() );
			//	edges_cornerX, edges_cornerY is the meeting point of two edges edge1 and edge2
			double edges_cornerX = edge1.getX2();
			double edges_cornerY = edge1.getY2();

			double slope1 = getSlope( edge1 );
			double slope2 = getSlope( edge2 );

			double alpha = findCornerAngle(slope1, slope2);

			double rad = dia_robot / Math.cos( ( Math.PI - Math.abs(alpha) ) / 2.0);
			double beta = ( 2.0 * Math.PI - Math.abs(alpha) ) / 2.0;
			double slope = ( slope2 - Math.tan( beta ) ) / ( 1.0 + slope2 * Math.tan( beta ) );

			ArrayList<Point2D.Double> alteredPts = findResizedVertices(edges_cornerX, edges_cornerY, rad, slope);
			Point2D pt = alteredPts.get(0);
			//	Here the dist represents the relative distance to altered vertex from both the edges  
			double dist = edge1.ptSegDist(pt) - edge2.ptSegDist(pt);
			//			If the relative dist is greater than the robot radius then should find the new altered vertices of an edge
			if( Math.abs(dist) > cut_off )
			{
				//System.out.println("first attempt wrong");
				slope = -1.0 / slope;
				beta = Math.atan( ( slope2 - slope ) / ( 1 + slope*slope2 ) );
				alpha = -2.0 * beta + 2.0 * Math.PI;
				rad = dia_robot / Math.cos( ( Math.PI - Math.abs(alpha) ) / 2.0);
				alteredPts = findResizedVertices(edges_cornerX, edges_cornerY, rad, slope);

				pt = alteredPts.get(0);
				dist = edge1.ptSegDist(pt) - edge2.ptSegDist(pt);
			}


			alteredvertices.addAll( alteredPts );
		}

		findHull();
		setPolygon();
	}
	
	//		This method will create polygon objects
	public void setPolygon()
	{
		ArrayList<Point2D> corners = hullvertexpoints;
		if(obs_map_outline)
			corners = poly_vertices;

		int[] xcorners = new int[corners.size()];
		int[] yCorners = new int[corners.size()];

		for(int i=0; i<corners.size(); i++)
		{
			int xCorner = (int)(corners.get(i).getX()*check_bias);
			int yCorner = (int)(corners.get(i).getY()*check_bias);
			xcorners[i] = xCorner;
			yCorners[i] = yCorner;
			alteredhullnodes.add(new Point2D.Double(xCorner, yCorner));
		}

		alteredPolygon = new Polygon(xcorners, yCorners, corners.size());
	}

	// This method will find the hull of the altered vertices 

	private void findHull() {

		Point2D rightMostRef = new Point2D.Double(-1000, -1000);
		for( Point2D vertex : alteredvertices )
		{
			if( vertex.getY() > rightMostRef.getY() )
				rightMostRef = vertex;
			else if( vertex.getY() == rightMostRef.getY() && vertex.getX() > rightMostRef.getX() )
				rightMostRef = vertex;
		}

		// This ptList will be the collection of all the altered vertices and their corresponding reference point and its distance
		//		from ref point of an obstacle 
		LinkedList<RefPoint> ptList = new LinkedList<RefPoint>();
		for( Point2D vertex : alteredvertices )
		{
			if( vertex != rightMostRef )
			{
				double dist = rightMostRef.distance(vertex);
				ptList.add(new RefPoint(vertex, rightMostRef, dist));
			}
		}
		Collections.sort(ptList);

		Stack<Point2D> alt_vertNodes = new Stack<Point2D>();
		alt_vertNodes.push(ptList.getLast().vertex);
		alt_vertNodes.push(rightMostRef);

		Point2D extreme_pt;
		Point2D nxt_extpt;
		ListIterator<RefPoint> lstItr = ptList.listIterator();
		while( lstItr.hasNext() )
		{
			Point2D temp = lstItr.next().vertex;
			extreme_pt = alt_vertNodes.pop();
			nxt_extpt = alt_vertNodes.peek();
			double a1 = extreme_pt.getX() - nxt_extpt.getX();
			double b1 = extreme_pt.getY() - nxt_extpt.getY();

			double a2 = temp.getX() - extreme_pt.getX();
			double b2 = temp.getY() - extreme_pt.getY();

			double crossPdct = a1 * b2 - b1 * a2;
			if( crossPdct > 0 )
			{
				alt_vertNodes.push(extreme_pt);
				alt_vertNodes.push(temp);
			}
			else
				lstItr.previous();
		}

		while( !alt_vertNodes.empty() )
		{
			Point2D temp = alt_vertNodes.pop();
			if( !alt_vertNodes.empty() )
			{
				hullvertexpoints.add(temp);
				//System.out.println("hull point: "+current.getX()+", "+current.getY());
				resized_hullEdges.add(new Line2D.Double(temp, alt_vertNodes.peek()));
			}
		}
	}

	// This method will check if two polygonal obstacles are equal or not
	public boolean equals(PolygonalObstacle plygn)
	{
		//checks the edge count    
		if(this.polygon_edges.size() != plygn.polygon_edges.size())
			return false;
		// checks corresponding edges
		for(int x=0; x<this.polygon_edges.size(); x++)
		{
			if(!this.polygon_edges.get(x).equals(plygn.polygon_edges.get(x)))
				return false;
		}

		return true;
	}

	
	
	// This method will alter the vertices of the original obstacles according to the robot size 
	private ArrayList<Point2D.Double> findResizedVertices(double edges_cornerX, double edges_cornerY, double rad, double slope)
	{       
		double intercept = edges_cornerY - slope * edges_cornerX;
		return findalteredVertices( edges_cornerX, edges_cornerY, rad, slope, intercept );
	}
	
	// This method finds the new resized vertex points of an edge relative to the robot size from the calculated slope, intercept
		private ArrayList<Point2D.Double> findalteredVertices( double edges_cornerX , double edges_cornerY, double rad, double slope, double intcpt )
		{
			//		The altptList will have newly formed vertices of an edge
			ArrayList<Point2D.Double> altptList = new ArrayList<Point2D.Double>(2);

			double v1x = getv1x( edges_cornerX, edges_cornerY, rad, slope, intcpt );
			double v1y = slope * v1x + intcpt;
			altptList.add( new Point2D.Double( v1x, v1y ) );

			double v2x = getv2x( edges_cornerX, edges_cornerY, rad, slope, intcpt );
			double v2y = slope * v2x + intcpt;
			altptList.add( new Point2D.Double( v2x, v2y ) );

			return altptList;
		}

	// This method will set the final slope depending on the calculated slope of an edge
	private double extremeSet(double slop) {
		if( slop == Double.NEGATIVE_INFINITY )
			return -1.0 * setbias;
		if( slop == Double.POSITIVE_INFINITY )
			return setbias;
		return slop;
	}

	//	This method will find the angle between two edges of an obstacle
	private double findCornerAngle(double slope1, double slope2)
	{
		if( slope2 < slope1 )
		{
			double m = slope1;
			slope1 = slope2;
			slope2 = m;
		}
		double value = Math.atan( ( slope2 - slope1 ) / ( 1 + slope1 * slope2 ) );
		if( value < 0 )
			value += Math.PI;
		return value;
	}

	//	This method finds the slope of a line
	private double getSlope( Line2D edge )
	{
		double m = ( edge.getY2() - edge.getY1() ) / ( edge.getX2() - edge.getX1() );
		return extremeSet( m );
	}

	
	// This methods will return the altered x coordinates of both the ends of an edge 

	private double getv1x( double ecX, double ecY, double rad, double slp, double intcpt )
	{
		return ( ( ecX - slp*intcpt + slp*ecY ) + Math.sqrt(Math.pow(-1*ecX + slp*intcpt - slp*ecY, 2) - (1 + slp*slp) * (ecX*ecX + intcpt*intcpt - 2*intcpt*ecY + ecY*ecY - rad*rad)) ) / (1 + slp*slp);
	}

	private double getv2x(  double ecX, double ecY, double rad, double slp, double intcpt  )
	{
		return ( ( ecX - slp*intcpt + slp*ecY ) - Math.sqrt(Math.pow(-1*ecX + slp*intcpt - slp*ecY, 2) - (1 + slp*slp) * (ecX*ecX + intcpt*intcpt - 2*intcpt*ecY + ecY*ecY - rad*rad)) ) / (1 + slp*slp);
	}
	// This method will perform cross product of vectors formed from the new altered edges of hull 
	
	private int crossprod(Line2D edg1, Line2D edg2)
	{
		double e1_X2x1 = edg1.getX1() - edg1.getX2();
		double e1_Y2y1 = edg1.getY1() - edg1.getY2();
		double e2_X2x1 = edg2.getX1() - edg2.getX2();
		double e2_Y2y1 = edg2.getY1() - edg2.getY2();
		return ( (e1_X2x1 * e2_Y2y1 - e1_Y2y1 * e2_X2x1) > 0 ) ? 1 : -1;
	}
 
//	This RefPoint class is used to maintain the altered vertices of hull 
	private class RefPoint implements Comparable<RefPoint>
	{
		public Point2D vertex;
		public Point2D ref_vertex;
		public double dist;


		public RefPoint(Point2D vertex, Point2D ref_vertex, double dist )
		{
			this.vertex = vertex;
			this.ref_vertex = ref_vertex;
			this.dist = dist;
		}

		public int compareTo(RefPoint other) 
		{
			Line2D edge2 = new Line2D.Double(this.vertex, this.ref_vertex);
			Line2D edge1 = new Line2D.Double(other.vertex, this.ref_vertex);
			int prod = crossprod(edge1, edge2);
			if( prod != 0 )
				return prod;

			if( other.dist > this.dist )
				return -1;
			else if( other.dist < this.dist )
				return 1;

			return 0;
		}
	}
}
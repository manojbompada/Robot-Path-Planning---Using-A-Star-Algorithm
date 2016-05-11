import java.io.*;
import java.util.*;
import java.awt.*;
import java.awt.geom.*;

import javax.swing.*;

public class MainClass {

	//	public static double robotSize = 165.0;
	public static Point2D starting_position = new Point2D.Double();
	public static Point2D goal_position = new Point2D.Double();
	public static int obstacleCount = 0;
	public static ArrayList<PolygonalObstacle> polygons = new ArrayList<PolygonalObstacle>();
	private static GUILayoutClass graphlayer;

	@SuppressWarnings("resource")
	public static void main(String[] args) throws FileNotFoundException 
	{

		File polygonal_map = new File("C:/Users/Manoj/Desktop/ROBO/obstacleMapB.txt");


		Scanner obstaclemap = new Scanner(polygonal_map);


		obstacleCount = Integer.parseInt(obstaclemap.nextLine());

		//Iterates through input obstacle file and gets the number of obstacles and their corresponding edges and vertices
		for (int obscount = 0; obscount < obstacleCount; obscount++) 
		{

			int polygon_vertexcount = Integer.parseInt(obstaclemap.nextLine());
			ArrayList<Line2D> polygon_edges = new ArrayList<Line2D>();
			ArrayList<Point2D> polygon_vertices = new ArrayList<Point2D>();

			for (int vCount = 0; vCount < polygon_vertexcount; vCount++) 
			{
				String polygon_position = obstaclemap.nextLine();
				String[] positionXY = polygon_position.split(",");
				double positionX = Double.parseDouble(positionXY[0]);
				double positiony = Double.parseDouble(positionXY[1]);

				Point2D polygon_vertex = new Point2D.Double(positionX, positiony);
				polygon_vertices.add(polygon_vertex);
			}

			for (int edgecount = 0; edgecount < polygon_vertexcount - 1; edgecount++)
			{
				polygon_edges.add(new Line2D.Double(polygon_vertices.get(edgecount), polygon_vertices
						.get(edgecount + 1)));

			}
			polygon_edges.add(new Line2D.Double(polygon_vertices.get(polygon_vertexcount - 1), polygon_vertices
					.get(0)));

			PolygonalObstacle obsObject = new PolygonalObstacle(polygon_edges, polygon_vertices, polygon_vertexcount);
			if (polygons.size() == 0)
				obsObject.setOutline(true);
			polygons.add(obsObject);
		}

		//		File start_goalFile = new File("C:/Users/Manoj/Desktop/ROBO/start_goal.txt");
		String start, goal;
		Scanner start_goalScanner = new Scanner(System.in);

		// get the starting point
		System.out.println("Enter start location (x,y) : ");
		start = start_goalScanner.nextLine();
		String[] startCoord = start.split(",");
		starting_position = new Point2D.Double(Double.parseDouble(startCoord[0]),Double.parseDouble(startCoord[1]));
		System.out.println(starting_position);

		// get the goal point
		System.out.println("Enter goal location (x,y) : ");
		goal = start_goalScanner.next();
		String[] goalCoord = goal.split(",");
		goal_position = new Point2D.Double(Double.parseDouble(goalCoord[0]), Double.parseDouble(goalCoord[1]));
		System.out.println(goal_position);
	
		
		
		//creating rejected list and checks if start and goal position are valid or not
		ArrayList<Point2D> checkpoints = PathPlanning.checkPoints(polygons,starting_position, goal_position);
		//Generating the obstacles elements (edges and vertices) relative to robot

		ArrayList<Line2D> poly_edges = new ArrayList<Line2D>();
		ArrayList<Line2D> resized_polyedges = new ArrayList<Line2D>();


		/*
		 * Add elements that are related to the room boundaries.
		 */
		//		polygons.get(0).Resize();
		//		resized_edges.addAll(polygons.get(0).resized_lines);
		//		poly_edges.addAll(polygons.get(0).polygon_lines);

		for (int i = 0; i < polygons.size(); i++)
		{
			polygons.get(i).alterObstacles();
			resized_polyedges.addAll(polygons.get(i).resized_hullEdges);
			poly_edges.addAll(polygons.get(i).polygon_edges);

		}
		
		
//		for(PolygonalObstacle polygn : polygons){
//			if(!polygn.obs_map_outline){
//			if(polygn.alteredPolygon.contains(starting_position)){
//				
//				System.out.println("Enter proper start position");
//			    System.exit(0);
//			}
//			else if(polygn.alteredPolygon.contains(goal_position)){
//					
//					System.out.println("Enter proper goal position");
//					System.exit(0);
//				}
//			}
//		}	
		


		/*
		 ***Calling various methods for creating visibility graph, performing Astar search algorithm
		 */



		ArrayList<Point2D> visible_nodes = PathPlanning.findVisibleNodes(starting_position,goal_position,resized_polyedges, checkpoints);

		ArrayList<Line2D> visibility_edges = PathPlanning.visibilityGraph(visible_nodes,resized_polyedges, poly_edges, starting_position, goal_position, checkpoints,polygons);

		//*********************************************************************************************************
		ArrayList<GraphnodeWrapper> graphnodes_list = new ArrayList<GraphnodeWrapper>();

		for(Line2D vis_edge1 : visibility_edges)
		{
			Point2D point1 = vis_edge1.getP1();
			Point2D point2 = vis_edge1.getP2();

			//setGraph will create graph nodes and corresponding reachable nodes for point1 and point2
			PathPlanning.setGraph(visibility_edges, checkpoints, graphnodes_list, point1, starting_position, goal_position);
			PathPlanning.setGraph(visibility_edges, checkpoints, graphnodes_list, point2, starting_position, goal_position);
		}
		//***********************************************************************************************************

		ArrayList<Line2D> goal_path = PathPlanning.aStarSearch(graphnodes_list,visibility_edges,checkpoints, starting_position, goal_position);


		/**
		 * Creating frame layout
		 */


		graphlayer = new GUILayoutClass();
		JFrame window = new JFrame("Map GUI");
		JScrollPane scrollbar = new JScrollPane(graphlayer);

		window.setSize(1500, 900);
		window.add(scrollbar, BorderLayout.CENTER);
		window.setVisible(true);
		window.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

		graphlayer.setPreferredSize(new Dimension(1500, 900));

		/*
		 * constructing map for obstacles, visibility graph and goal path
		 */

		graphlayer.visibilitygraph(visibility_edges);
		graphlayer.startpoint(starting_position.getX(), starting_position.getY());
		graphlayer.goalpoint(goal_position.getX(), goal_position.getY());
		//		graphlayer.polygonalEdges(polygons.get(0).polygon_lines);
		for (int i = 0; i < polygons.size(); i++) {
			graphlayer.createPolyObs(polygons.get(i).polygon_edges);
			graphlayer.createAltPolyObs(polygons.get(i).resized_hullEdges);
		}

		graphlayer.robot_path(goal_path);


	}
}
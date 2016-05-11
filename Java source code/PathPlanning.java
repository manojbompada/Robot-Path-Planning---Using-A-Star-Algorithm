import java.awt.Point;
import java.awt.Polygon;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedList;
import java.util.Queue;
import java.util.Collections;


public class PathPlanning
{       

	/**
	 * This method creates visibility graph. It returns all the visibile edges
	 */
	public static ArrayList<Line2D> visibilityGraph(ArrayList<Point2D> visible_nodes,ArrayList<Line2D> resized_polyedges, ArrayList<Line2D> poly_edges, 
			Point2D start_location, Point2D goal_location, ArrayList<Point2D> rejected_list, ArrayList<PolygonalObstacle> polygons)
			{

		ArrayList<Line2D> visibility_edges = findVisPoly_resizedEdges(resized_polyedges);

		ArrayList<Line2D> allpoly_edges = new ArrayList<Line2D>();
		allpoly_edges.addAll(resized_polyedges);
		allpoly_edges.addAll(poly_edges);

		/*
		 * This will make all the possible visible edges from the available visible_nodes list and existing polygon edges.
		 * This is done by checking if the new edge created will intersect the other existing edges or not.
		 * Also validates if the intersection point is a node of intersection of two edges and ignores if it satisfies
		 */
		for(Point2D visAnode : visible_nodes)
		{
			for(Point2D visBnode : visible_nodes)
			{

				if(!visAnode.equals(visBnode))
				{
					Line2D.Double newVisibleedge = new Line2D.Double(visAnode, visBnode);

					String edgeintersect = "no";
					for(Line2D polyEdge : allpoly_edges)
					{
						if(newVisibleedge.intersectsLine(polyEdge))
						{
							if(!(newVisibleedge.getP1().equals(polyEdge.getP1()) ||
									newVisibleedge.getP1().equals(polyEdge.getP2()) ||
									newVisibleedge.getP2().equals(polyEdge.getP1()) ||
									newVisibleedge.getP2().equals(polyEdge.getP2())))
							{
								edgeintersect = "yes";
								break;
							}
						}
					}

					/*
					 * This will check if the new visible edge intersects any altered polygons by passing
					 * between two diagonal corners of polygon and is added to thevisibility_edges list.
					 * The intersection with altered polygons is checked by testing whether the mid point of new edge will
					 * lie inside the polygon or not. 
					 */
					double centerX = (newVisibleedge.x1 + newVisibleedge.x2)/2.0*1000000.0;
					double centerY = (newVisibleedge.y1 + newVisibleedge.y2)/2.0*1000000.0;

					Point2D edgeCenter = new Point2D.Double(centerX, centerY);

					String center_inside = "no";

					for(PolygonalObstacle poly : polygons)
					{
						if(!poly.obs_map_outline && poly.alteredPolygon.contains(edgeCenter)){
							center_inside = "yes";
						}

					}

					if((edgeintersect.equals("no"))){
						if(center_inside.equals("no") && polygons.get(0).alteredPolygon.contains(edgeCenter))
						{
							visibility_edges.add(newVisibleedge);

						}
					}

				}
			}
		}

		return visibility_edges;
			}

	public static ArrayList<Point2D> checkPoints(ArrayList<PolygonalObstacle> polygons, Point2D start, Point2D goal)
	{
		ArrayList<Point2D> rejectedlist = new ArrayList<Point2D>();

		for(PolygonalObstacle polygon1 : polygons)
		{
			for(PolygonalObstacle polygon2 : polygons)
			{
				if(!polygon1.equals(polygon2) && !polygon2.obs_map_outline)
				{

					for(int i=0; i<polygon1.alteredhullnodes.size(); i++)
					{

						Point2D alteredpoint = polygon1.alteredhullnodes.get(i);
						if(polygon2.alteredPolygon.contains(alteredpoint) && !rejectedlist.contains(polygon1.hullvertexpoints.get(i)))
						{
							rejectedlist.add(polygon1.hullvertexpoints.get(i));
						}
					}
				}
				else if(!polygon1.obs_map_outline && polygon2.obs_map_outline)
				{
					for(int pt=0; pt<polygon1.alteredhullnodes.size(); pt++)
					{
						Point2D scaledPt = polygon1.alteredhullnodes.get(pt);
						if(!polygon2.alteredPolygon.contains(scaledPt) && !rejectedlist.contains(polygon1.hullvertexpoints.get(pt)))
						{
							rejectedlist.add(polygon1.hullvertexpoints.get(pt));
						}
					}
				}
			}
		}

		return rejectedlist;
	}


	public static ArrayList<Point2D> findVisibleNodes(Point2D start_location, Point2D goal_location,
			ArrayList<Line2D> resized_polyedges, ArrayList<Point2D> rejected_list)
			{
		ArrayList<Point2D> vis_nodes = new ArrayList<Point2D>();

		// initialize the visibility_nodes array list with start and goal locations
		vis_nodes.add(start_location);
		vis_nodes.add(goal_location);

		/*
		 * Iterates through each polygon edges and add the list of nodes of edges to 
		 * visibility_nodes array list without redundancy		 */


		for(Line2D edge : resized_polyedges)
		{

			/*
			 * gets the both end points of an edge of an obstacle and adds to the list
			 */
			if(!vis_nodes.contains(edge.getP1())){
				if(!rejected_list.contains(edge.getP1()))
				{
					vis_nodes.add(edge.getP1());

				}

			}


			if(!vis_nodes.contains(edge.getP2())){
				if(!rejected_list.contains(edge.getP2()))
				{
					vis_nodes.add(edge.getP2());

				}

			}
		}
		return vis_nodes;
			}


	/*
	 * This method will check all the resized polygon edges and add an edge to the visibility_edges list
	 * if it doesn't intersect with other edges
	 */

	public static ArrayList<Line2D> findVisPoly_resizedEdges(ArrayList<Line2D> resized_polyedges)
	{
		ArrayList<Line2D> vis_edges = new ArrayList<Line2D>();


		for(Line2D polyAedge : resized_polyedges)
		{

			String edgeAintersect = "no";
			for(Line2D polyBedge : resized_polyedges)
			{
				if(polyAedge.intersectsLine(polyBedge))
				{
					if(!(polyAedge.getP1().equals(polyBedge.getP1()) ||
							polyAedge.getP1().equals(polyBedge.getP2()) ||
							polyAedge.getP2().equals(polyBedge.getP1()) ||
							polyAedge.getP2().equals(polyBedge.getP2())))
					{
						edgeAintersect = "yes";
						break;
					}
				}
			}

			if(edgeAintersect.equals("no"))	
				vis_edges.add(polyAedge);
		}
		return vis_edges;
	}

	/**
	 * This method performs the ASTAR search on the graph containing list of visible edges in order to find the shortest
	 * path from start point to goal point
	 */
	public static ArrayList<Line2D> aStarSearch(ArrayList<GraphnodeWrapper> graphnodes_list, ArrayList<Line2D> visible_edges, 
			ArrayList<Point2D> checkpoints, Point2D start_pos, Point2D goal_pos)
			{       

		//		System.out.println("ASTAR");

		int startIndex = searchIndex(graphnodes_list, start_pos);
		GraphnodeWrapper curNode = graphnodes_list.get(startIndex);

		//		System.out.println(curNode.getPoint());
		Point2D goalpt = goal_pos;
		//		System.out.println(goalpt);

		GraphnodeWrapper init =  new GraphnodeWrapper(curNode.getPoint(),curNode.getReachables(),goalpt);
		//		System.out.println("INIT NODE:");
		//		System.out.println("init point: " +init.getPresentnode());
		//		System.out.println("init reachabess: "+curNode.getReachables());

		ArrayList<Line2D> shortestPath = new ArrayList<Line2D>();

		LinkedList<GraphnodeWrapper> openList = new LinkedList<GraphnodeWrapper>();
		// Initiate the open list with start node        
		openList.add(init);

		ArrayList<GraphnodeWrapper> closedList = new ArrayList<GraphnodeWrapper>();


		while (!openList.isEmpty())
		{
			GraphnodeWrapper qnode = openList.poll();
			//			System.out.println(qnode.getPresentnode());
			if(qnode.isGoal(goalpt))
			{
				shortestPath = goalpath(qnode,start_pos);
				return shortestPath;
			}	
			closedList.add(qnode);

			ArrayList<Point2D> checksuccessor = new ArrayList<Point2D>();
			for(GraphnodeWrapper graphnode : graphnodes_list)
			{
				if(graphnode.point.equals(qnode.getPresentnode()))
					checksuccessor = graphnode.getReachables();

			}

			ArrayList<GraphnodeWrapper> Successor = new ArrayList<GraphnodeWrapper>();

			for (int i = 0; i < checksuccessor.size(); i++)
			{ 

				//				System.out.println("Successor" + i +" "+checksuccessor.get(i));
				GraphnodeWrapper generatednode;

				// Here the heuristic cost is implemented as the distance between the node and the goal
				double levelcost = qnode.getLevelcost() + checksuccessor.get(i).distance(qnode.getPresentnode());
				double heuristiccost = checksuccessor.get(i).distance(goalpt);

				int pointIndex = searchIndex(graphnodes_list, checksuccessor.get(i));

				GraphnodeWrapper sucNode = graphnodes_list.get(pointIndex);					

				ArrayList<Point2D> neighbours = sucNode.getReachables();

				/*
				 * generates new Graph node by using the constructor 
				 * 
				 */
				generatednode = new GraphnodeWrapper(qnode,checksuccessor.get(i), levelcost,heuristiccost,neighbours);
				//					System.out.println("generat Success   " +generatednode.getPresentnode());
				//					System.out.println(generatednode.getReachables());

				/*
				 * checks whether the generated node is already in one of the parents node
				 */
				if (!nodeTest(generatednode))
				{
					//Adds the successor to the array list if the node is not repeated


					Successor.add(generatednode);
					//						System.out.println("success add  ");
					//					}

					//						System.out.println("end of sucesor add-------------------------------  ");


				}

				//					System.out.println(" end of success for loop");

				if (Successor.size() == 0){
					//						System.out.println("Suceesor size = 0");
					continue;
				}
				//				System.out.println("minpriori  ");
				genMinpriorityQ(Successor,openList,goalpt);

			}
		}

		return null;

			}

	//This method creates the goal path from start point to goal point
	public static ArrayList<Line2D> goalpath(GraphnodeWrapper qnode, Point2D start)
	{
		ArrayList<Line2D> shortestPath = new ArrayList<Line2D>();
		GraphnodeWrapper current = qnode;
		//			System.out.println("goal                "+goalpt);
		//			System.out.println((current.getParentnode().getPresentnode()));
		//			System.out.println(current.getParentnode());
		boolean pathFound = false;
		while((!pathFound) && current.getParentnode() != null)
		{
			shortestPath.add(new Line2D.Double(current.getPresentnode(), current.getParentnode().getPresentnode()));
			current = current.getParentnode();
			if(current.getPresentnode().equals(start))
				pathFound = true;
		}
		Collections.reverse(shortestPath);

		//		System.out.println("path");
		//		for (int i = 0; i < shortestPath.size(); i++){

		//			System.out.println(shortestPath.get(i));

		//		}


		return shortestPath;

	}


	/**
	 * Returns the index of the starting or ending location.
	 */
	private static int searchIndex(ArrayList<GraphnodeWrapper> nodes, Point2D findPoint)
	{
		int index = 0;
		for(GraphnodeWrapper gn : nodes)
		{
			if(gn.point.equals(findPoint))
				return index;

			index++;
		}

		return -1;
	}

	/*
	 * This method will refactor the open linked list to imitate like a priority queue
	 * based on the evaluation cost of the node and add the successors to the list which are 
	 * of minimum evaluation cost
	 */
	private static void genMinpriorityQ(ArrayList<GraphnodeWrapper> minQ,LinkedList<GraphnodeWrapper> q, Point2D goal)
	{
		//		GraphNode minnode = minQ.get(0);

		//		System.out.println("min node before "+ minnode.getPresentnode()+" dist-- "+ minnode.getPresentnode().distance(goal));

		Collections.sort(minQ);
		//		for (int i = 0; i < minQ.size(); i++)
		//		{
		//			if (minnode.getEvalfcost() > minQ.get(i).getEvalfcost())
		//			{
		//				minnode = minQ.get(i);
		//			}
		//		}

		//		double minval =  minnode.getEvalfcost();
		double minval =  minQ.get(0).getEvalfcost();


		// Adds any nodes that have that same lowest value.
		for (int i = 0; i < minQ.size(); i++)
		{
			if (minQ.get(i).getEvalfcost() <= minval)
			{
				q.add(minQ.get(i));
				//				System.out.println("min node after " +minQ.get(i).getPresentnode()+" dist-- "+ minQ.get(i).getPresentnode().distance(goal));

			}
		}

		Collections.sort(q);
	}


	/*
	 * This method checks whether a node is already evaluated or not, that is, whether
	 * the node is repeated or not	
	 */
	private static boolean nodeTest(GraphnodeWrapper newnode)
	{
		boolean isRepeat = false;
		GraphnodeWrapper tempnode = newnode;

		while (newnode.getParentnode() != null && !isRepeat)
		{
			//			System.out.println("nodetest " +newnode.getParentnode().getPresentnode()+ " " +tempnode.getPresentnode());
			if (newnode.getParentnode().getPresentnode().equals(tempnode.getPresentnode()))
			{
				isRepeat = true;
			}
			newnode = newnode.getParentnode();
		}
		//		System.out.println("repeat  " +isRepeat);



		return isRepeat;
	}


	/*
	 * This method will create graph nodes which has list of all the reachable points from a given point
	 * The Astar algorithm uses this method to find the list of reachable nodes from each node. 
	 * 
	 */
	public static void setGraph(ArrayList<Line2D> visibility_edges, ArrayList<Point2D> checkpoints,
			ArrayList<GraphnodeWrapper> graphnodes_list, Point2D point1, Point2D starting_position, Point2D goal_position) {

		//This will check the duplicates
		if(!listcontains(graphnodes_list, point1))
		{

			ArrayList<Point2D> reachables_points = new ArrayList<Point2D>();

			//This will find all the reachable points from given point 'point1'
			for(Line2D vis_edge : visibility_edges)
			{
				// A point1 which is on an edge must be reachable to the other end of the edge
				if(point1.equals(vis_edge.getP1()) && !reachables_points.contains(vis_edge.getP2())){
					if(!checkpoints.contains(vis_edge.getP2())){
						reachables_points.add(vis_edge.getP2());
					}
				}


				if(point1.equals(vis_edge.getP2()) && !reachables_points.contains(vis_edge.getP1())){
					if(!checkpoints.contains(vis_edge.getP1())){
						reachables_points.add(vis_edge.getP1());
					}
				}

				//				System.out.println("Reachables");
				//				for(int i=0;i<reachables_points.size();i++){
				//
				//					//					System.out.println(p1+"  "+reachables.get(i));
				//
				//				}

			}

			GraphnodeWrapper graphnode = new GraphnodeWrapper(point1, reachables_points, graphnodes_list.size());
			if(graphnode.point.equals(starting_position))
				graphnode.isStart = true;

			else if(graphnode.point.equals(goal_position))
				graphnode.isGoal = true;

			graphnodes_list.add(graphnode);
		}
	}

	/**
	 * This method will check for repeated nodes
	 */
	private static boolean listcontains(ArrayList<GraphnodeWrapper> nodes, Point2D p)
	{
		for(GraphnodeWrapper n : nodes)
		{
			if(n.equals(p))
				return true;
		}

		return false;
	}

}
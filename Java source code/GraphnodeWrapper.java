import java.awt.geom.Point2D;
import java.util.ArrayList;

// This class is used to maintain all the nodes of with their corresponding attribute measures.
public class GraphnodeWrapper implements Comparable<GraphnodeWrapper>
	{
		public Point2D point;
		public ArrayList<Point2D> reachables;
		public double distance;
		public int index = 0;
		public boolean isStart;
		public boolean isGoal;

		private Point2D presentnode; // current node
		private GraphnodeWrapper parentnode; // parent current node
		private double levelcost; // cost to get to this state
		private double heuristiccost; // heuristic cost
		private double Evalfcost; // Evaluation cost f(n) 


		public boolean equals(GraphnodeWrapper gn)
		{
			return this.point.equals(gn.point);
		}

		//constructor
		public GraphnodeWrapper(Point2D p, ArrayList<Point2D> r, int in)
		{
			point = p;
			isStart = false;
			isGoal = false;
			reachables = r;
		}

		public GraphnodeWrapper(Point2D present, ArrayList<Point2D> r, Point2D goal)
		{
			presentnode = present;
			parentnode = null;
			levelcost = 0;
			heuristiccost = presentnode.distance(goal);
			Evalfcost = levelcost+heuristiccost;
			reachables = r;
		}

		public GraphnodeWrapper(GraphnodeWrapper parent, Point2D present, double gcost, double hcost,ArrayList<Point2D> r )
		{
			parentnode = parent;
			presentnode = present;
			levelcost = gcost;
			heuristiccost = hcost;
			Evalfcost = levelcost + heuristiccost;
			reachables = r;
		}

		public int compareTo(GraphnodeWrapper otherGraphNode) {
			return Double.compare(this.Evalfcost,otherGraphNode.Evalfcost);
		}

		public Point2D getPoint() {
			return point;
		}

		public void setPoint(Point2D point) {
			this.point = point;
		}

		public ArrayList<Point2D> getReachables() {
			return reachables;
		}

		public void setReachables(ArrayList<Point2D> reachables) {
			this.reachables = reachables;
		}

		

		public double getDistance() {
			return distance;
		}

		public void setDistance(double distance) {
			this.distance = distance;
		}

		public int getIndex() {
			return index;
		}

		public void setIndex(int index) {
			this.index = index;
		}

		public boolean isStart() {
			return isStart;
		}

		public void setStart(boolean isStart) {
			this.isStart = isStart;
		}

		//			public boolean isGoal() {
		//				return isGoal;
		//			}

		public void setGoal(boolean isGoal) {
			this.isGoal = isGoal;
		}

		public Point2D getPresentnode() {
			return presentnode;
		}

		public void setPresentnode(Point2D presentnode) {
			this.presentnode = presentnode;
		}

		public GraphnodeWrapper getParentnode() {
			return parentnode;
		}

		public void setParentnode(GraphnodeWrapper parentnode) {
			this.parentnode = parentnode;
		}

		public double getLevelcost() {
			return levelcost;
		}

		public void setLevelcost(int levelcost) {
			this.levelcost = levelcost;
		}

		public double getHeuristiccost() {
			return heuristiccost;
		}

		public void setHeuristiccost(int heuristiccost) {
			this.heuristiccost = heuristiccost;
		}

		public double getEvalfcost() {
			return Evalfcost;
		}

		public void setEvalfcost(int evalfcost) {
			Evalfcost = evalfcost;
		}

		public boolean isGoal(Point2D x)
		{


			if (getPresentnode().equals(x))
			{
				return true;
			}
			return false;

		}


	}
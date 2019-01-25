
import java.util.*;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class Driver {
    // Set Initial default goal state
    int[] goal_state = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0};
    // Set Initial default goal state
    int [] initial_state = {1 ,0 ,3 ,7 ,5 ,2 ,6 ,4 ,9 ,10, 11, 8};
    // Some global variable
    State goal_node ;
    String FILEPATH = "Results/";
    int max_frontier_size = 0;
    int nodes_expanded = 0;
    int	max_search_depth = 0;
    int	search_depth = 0;
    int depth = 0;
    ArrayList<String> moves = new ArrayList<String>()  ;   
    char firstNode = '0';
    int totalCost= 0;
    Set<String> costs = new HashSet<String>(); 
    boolean goalFound = false;
    int depthLimit = 30;
    
//Implementation of DFS search algorithm with iterative approach
void dfs(int[] start_state)
   {
	// Initializing local variables
	State goal_node = new State();
    goal_node = null;
    Set<String> explored = new HashSet<String>();
	State start = new State(start_state , null ,0  , 0, 0 ,0 );
    boolean solutionFound = false;
    // Iterating the DFS algorithm for given depth limit. 
	for (int maxDepth = 1; maxDepth < depthLimit; maxDepth++) {
		explored.clear();
	    Stack<State> stack = new Stack<State>();
	    stack.add( new State(start_state , null ,0  , 0, 0 ,0 )) ;
	    goal_node = null;
		System.out.println("Depth of Tree: "+ maxDepth);
		// check till open list is empty
	    while(stack.size() != 0)
	    {
	    	State node = stack.pop();
	    	// add the a node property to check for explored node
	    	explored.add(node.map);
	    	// check if the node state is same to goal state, if solution is found set a boolean flag and break from while loop
	    	if(   Arrays.equals(node.state,goal_state))
	    	{
	    		goal_node = node;
	    		search_depth = goal_node.depth;
	            solutionFound = true;
	    		break;
	    	}
	    	// else check for the depth if it has not exceeded the max depth for the iteration
	    	if(node.depth < maxDepth )
	    		{
		    		// expand the possible states from the current state
	    			List<State> neighbors = expand(node);
	    			// for each neighbor , check if it has not been explored , if not , add to open list
		    		for(State neighbor : neighbors)
		    		{
	    			if(!(explored.contains(neighbor.map)))
	    				{	
	    					stack.add(neighbor);
	    					// explored.add(neighbor.map);
	    					
	    				}
	    			// to count the maximum depth the algorithm went to search for the goal state
	    			if(neighbor.depth > max_search_depth)
	                    max_search_depth += 1;
		    		}
		    		// to hold the max size of open list states
		    		 if(stack.size()> max_frontier_size)  
			             max_frontier_size = stack.size();
	    		}
	    	}
	    if (solutionFound)
            break;
	 }
	if (!solutionFound)
        System.out.println("No solution Found!");
    else 
    {
    	System.out.println("\n Goal State Found!");
    	// backtrack search path algorithm
    	backtrackPath(goal_node, start, FILEPATH);
    }

 }


// Implementation of A star algorithm
void astar(int[] start_state , String h)
{
	  // Initializing local variables
	  State goal_node = new State();
      goal_node = null;
      // The set of nodes already evaluated
	  Set<String> closedSet = new HashSet<String>();
	  // Comparator object reference created
	  Comparator<State> comparator = new NodeCompare();
	  // The set of currently discovered nodes that are not evaluated yet.
	   // Initially, only the start node is known.
	  PriorityQueue<State> openSet = new PriorityQueue<State>(100,comparator);
	  State start = new State(start_state , null ,0  , 0, 0 ,0 );
	  // added first state to the open set list
	  openSet.add(start);
	  // The cost of going from start to start is zero.
	  start.gScore =0;
	 // For the first node, that value is completely heuristic.
	  start.fScore =costFunction(start,goal_state , h );
	  // execute the entire block till the open list is empty
	  while(!(openSet.isEmpty())) 
	  {
		  	// take the first state from the open list set
		  	State current = openSet.remove();
		  	// check if the the current state is equal to goal state
	    	if(Arrays.equals(current.state,goal_state))
	    	{
	    		goal_node = current;
	            search_depth = goal_node.depth;
	    		break;
	    		
	    	}
	    	// add the current state to closed set
	    	closedSet.add(start.map);
	    	// expand the current state to find its neighbor
	       	List<State> current_neighbors = expand(current);
	    	for(State neighbor : current_neighbors)
	    	{
	    	
	    		if(closedSet.contains(neighbor.map))
	    			continue; // Ignore the neighbor which is already evaluated.
	    		// The distance from start to a neighbor
	    		int tentative_gScore = current.gScore + costFunction(current,neighbor.state , h );
	    		neighbor.gScore = tentative_gScore;
	    		neighbor.fScore= neighbor.gScore + costFunction(neighbor,goal_state , h );
	    		// set the final score to priority of the node based on which the priority queue reorders the element
	    		neighbor.priority = neighbor.fScore;
	    		if(neighbor.depth > max_search_depth)
	                    max_search_depth += 1;
	    		if(!(openSet.contains(neighbor.map))) // Discover a new node
	    		{
	    			openSet.add(neighbor);
	    		}
	    		else if(tentative_gScore>= neighbor.gScore)
	    			continue;	    	// This is not a better path.	
	    	}
	    	// to count the max size of open list 
	    	 if(openSet.size() > max_frontier_size)  
	             max_frontier_size = openSet.size();
	  }
	  // to back track the solution path
	  backtrackPath(goal_node, start, FILEPATH);
}

//Implementation of best first search algorithm
void bfs(int[] start_state , String h)
{
	// initialize the intial variables
	 State goal_node = new State();
     goal_node = null;
	 Set<String> closedSet = new HashSet<String>();
	 // comparator object reference created
	 Comparator<State> comparator = new NodeCompare();
	 PriorityQueue<State> openSet = new PriorityQueue<State>(100,comparator);
	 State start = new State(start_state , null ,0  , 0, 0 ,0 );
	 // added initial state to open list
	 openSet.add(start);
	 // process the block till the priority queue is not empty
	 while(!(openSet.isEmpty())) 
	  {
		 	// take the element form the queue
		    State current = openSet.remove();
		    // if the required state is found , break
	    	if(Arrays.equals(current.state,goal_state))
	    	{
	    		goal_node = current;
	            search_depth = goal_node.depth;
	    		break;
	    		
	    	}
	    	// else add the state to the closed set
	    	closedSet.add(start.map);
	    	// find the next possible states that is neighboring states from the current state
	       	List<State> current_neighbors = expand(current);
	    	for(State neighbor : current_neighbors)
	    	{
	    		// Ignore the neighbor which is already evaluated.
	    		if(closedSet.contains(neighbor.map))
	    			continue;
	    		// calculate the cost of the function based on the heuristic 
	    		neighbor.priority = costFunction(neighbor,goal_state , h );
	    		// to set the max depth the algorithm looked to find the goal state
	    		if(neighbor.depth > max_search_depth)
                    max_search_depth += 1;
	    		// if the item is not in open state add it to the open set
	    		if(!(openSet.contains(neighbor.map)))
	    		{
	    			openSet.add(neighbor);
	    		}
	    	
	    	}
	    	// hold the max size of the open list
	    	 if(openSet.size() > max_frontier_size)  
	             max_frontier_size = openSet.size();

	  }
	 	// back track the path for solution
	  	  backtrackPath(goal_node, start ,FILEPATH );
}

// Function to backtrack the solution and write it to the file
void backtrackPath(State node, State start, String FILENAME )
{
    Stack<String> backtrack = new Stack<>();
	 while (node.parent != null)
		{
			if(node.move != 0)
			{
				backtrack.push(node.position+ " " + Arrays.toString(node.state));
				moves.add(movement(node.move) );
			}
			node = node.parent;
	        	
	        }
	        backtrack.push("0 "+ Arrays.toString(start.state));

		while(!backtrack.isEmpty())
		{
	                String content = (String) backtrack.pop();
	                writeTofile(FILENAME , content);
		}
}

// Function to get the movement in word from it enumerated code
String movement(int move) {

	String movement = "" ;
        if(move == 1)
            movement = "Up";
       else if (move == 2)
            movement = "Down";
       else if (move == 3)
            movement = "Left";
       else if (move == 4)
            movement = "Right";
       else if (move == 5)
            movement = "Up–Right";
       else if (move == 6)
            movement = "Down–Right";
       else if (move == 7)
            movement = "Down–Left";
        else
            movement = "Up–Left";

        return movement;
  
}
// for debugging purpose
void print(State current)
{
	
	  System.out.println(current.move+ " - " + Arrays.toString(current.state) + " - " + current.gScore+ " - " +current.fScore+ " -"   +current.priority);

}

// wrapper function to invoke heuristic functions
private static int costFunction(State node, int[] goal, String h) {
    int priority = 0 ;
    if(h.equals("h1")) 
        priority = heuristic1( node,goal) ;
    else 
         priority = heuristic2(node.state) ;
    return priority;
	}
//heuristic functions to calculate hamming distance
private static int heuristic1(State node, int[] goal) {
		int priority;
		int count = 0;		
		for(int i=0; i<1; i++){
			if(node.state[i] != goal[i]){
				count++;
			}
		}
		priority = node.cost + count; 
		return priority;
	}
// heuristic functions to calculate inversion number
private static int heuristic2(int [] intial_state)
{

	int total_inversion = 0;
    for(int i = 0; i < intial_state.length ; i++)
	{
            for(int j =i ; j < intial_state.length ; j++)
		{
                    if( (intial_state[i] > intial_state[j]) && (intial_state[i] != 0) && (intial_state[j] != 0))
				total_inversion += 1;
		}

	}

	return total_inversion;

}

// function to find list of possible states based on the possible set of actions
List<State> expand(State node)
   {
	nodes_expanded += 1;
    List<State> neighbors =  new ArrayList<State>();
    State up = new State(move(node.state, 1), node, 1, node.depth + 1, node.cost + 1, 0);
    State  Down = new State(move(node.state, 2), node, 2, node.depth + 1, node.cost + 1, 0); 
    State   Left = new State(move(node.state, 3), node, 3, node.depth + 1, node.cost + 1, 0);
    State   Right = new State(move(node.state, 4), node, 4, node.depth + 1, node.cost + 1, 0);
    State   UpRight = new State(move(node.state, 5), node, 5, node.depth + 1, node.cost + 1, 0) ;
    State  DownRight = new State(move(node.state, 6), node, 6, node.depth + 1, node.cost + 1, 0) ;
    State  DownLeft = new State(move(node.state, 7), node, 7, node.depth + 1, node.cost + 1, 0) ;
    State  UpLeft = new State(move(node.state, 8), node, 8, node.depth + 1, node.cost + 1, 0) ;
    neighbors.add(UpLeft);
    neighbors.add(Left);
    neighbors.add(DownLeft);
    neighbors.add(Down);
    neighbors.add(DownRight);
    neighbors.add(Right);
    neighbors.add(UpRight); 
    neighbors.add(up);

    List<State> neighbors1 =  new ArrayList<State>();
    for(State x: neighbors)
    {
        if(x.state != null)
        {
            neighbors1.add(x);
        }  
    }
    
    return neighbors1 ;
    }

  // function that find the index of "0" from its current state for give movement.
int[] move(int []state, int position)
{
    int[] new_state = new int[12] ;  
    System.arraycopy( state, 0, new_state, 0, new_state.length );
    int index = 0;
    for(int i = 0 ; i < new_state.length ; i++ )
    {
        if(new_state[i] == 0)
        {
            index = i;
        }
    }
    if(position == 1)// Up
    {

    	// Index where the movement is not allowed for empty tiles from existing state by moving in upward direction
        if(!(index == 0 || index ==  1 || index == 2|| index ==  3))
        {   
            int temp =  new_state[index]; 
            new_state[index]= new_state[index - 4];
            new_state[index - 4]= temp;
            return new_state;
          }
        else
            return null;
      
    }
    if(position == 2)  // Down
    {
    	// Index where the movement is not allowed for empty tiles from existing state by moving in downward direction
        if(!(index == 8 || index ==  9 || index == 10|| index ==  11))
        {
            int temp =     new_state[index] ;
            new_state[index] = new_state[index + 4];
            new_state[index + 4] = temp;
            return new_state;
        }
        else
            return null;
    }
    if(position == 3)// Left
    {
    	// Index where the movement is not allowed for empty tiles from existing state by moving in left direction
        if(!(index == 0 || index ==  4 || index == 8))
        {
            int temp = new_state[index]; 
            new_state[index] =  new_state[index -1];
            new_state[index - 1] = temp;
            return new_state;
        }
        else
            return null;
     }
    if(position == 4)  //  Right
    {
    	// Index where the movement is not allowed for empty tiles from existing state by moving in right direction
        if(!(index == 3|| index ==  7 || index == 11))
        {
            int temp =  new_state[index];
            new_state[index]= new_state[index + 1];
            new_state[index + 1]  = temp;
            return new_state;
        }
        else
            return null;
    }
    if(position == 5)// UP & RIGHT
    {
    	// Index where the movement is not allowed for empty tiles from existing state by moving in up & right direction

        if(!(index == 0|| index ==  1 || index == 2|| index == 3|| index == 7|| index == 11 ))
        {
            int temp = new_state[index] ;
            new_state[index]= new_state[index - 3];
            new_state[index - 3]=temp; 
            return new_state; 
        }
        else
            return null;
    }
    if(position == 6)// DOWN &“RIGHT
    {
    	// Index where the movement is not allowed for empty tiles from existing state by moving in DOWN &“RIGHT direction

        if(!(index == 8|| index ==  9 || index == 10|| index == 11|| index == 3|| index == 7 ))
        {
            int temp = new_state[index] ; 
            new_state[index] = new_state[index + 5] ; 
            new_state[index + 5]= temp; 
            return new_state;
        }
        else
            return null;
                    }
     if(position == 7)// DOWNâ€“LEFT
    {
     	// Index where the movement is not allowed for empty tiles from existing state by moving in DOWN &“left direction

        if(!(index == 8|| index ==  9 || index == 10|| index == 11|| index == 0|| index == 4 ))
        {
            int temp = new_state[index] ; 
            new_state[index] = new_state[index + 3] ; 
            new_state[index + 3]= temp; 
            return new_state;
        }
        else
            return null;
                    }
 
       if(position == 8) // UPâ€“LEFT
    {
        // Index where the movement is not allowed for empty tiles from existing state by moving in UP &“left direction
        if(!(index == 0|| index ==  1 || index == 2|| index == 3|| index == 4|| index == 8 ))
        {
            int temp = new_state[index] ; 
            new_state[index] = new_state[index -5] ; 
            new_state[index -5]= temp; 
            return new_state;
        }
        else
            return null;
                    }
        return null;
}

// Code to write output result to the text file
void writeTofile(String filePath, String data)
{
   BufferedWriter bw = null;
		FileWriter fw = null;
		try 
		{
			File file = new File(filePath);
			
            if (!file.exists()) {
                file.createNewFile();

            }

            fw = new FileWriter(file.getAbsoluteFile(), true);
			bw = new BufferedWriter(fw);
                     
			bw.write(data);
            bw.newLine();		
		} catch (IOException e) {
			e.printStackTrace();
		} finally {

			try {

				if (bw != null)
					bw.close();

				if (fw != null)
					fw.close();

			} catch (IOException ex) {

				ex.printStackTrace();

			}
		}

}
// main method
  public static void main(String[] args)
  {
      Driver d= new Driver() ;

      System.out.println("Welcome to 12 Puzzle Problem");
      Scanner sc = new Scanner(System.in);
       // taking users input
      System.out.println("\nTo change default Inital State enter values space seperated");
      String iState = sc.nextLine();
      String[] statearr ;
      if(!(iState.isEmpty())) {
    	  statearr= iState.split(" ");
    	  if(statearr.length==12)
    	  {
    		  for(int i = 0 ; i < statearr.length ; i++)
    		  {
    			  d.initial_state[i] =Integer.parseInt(statearr[i]);
    			
    		  }	  
    	  }
      
      }
      else System.out.println("default Inital state selected : " +  Arrays.toString(d.initial_state));
      
      
      System.out.println("Goal State is : " +  Arrays.toString(d.goal_state ));

      System.out.println("\nChoose below options for the mentioned algoritihm to execute");
      System.out.println("\nFor Best First Search  - > Enter \"1\" \nFor Depth First Search - > Enter \"2\" \nFor A Star             - > Enter \"3\" \n");
      String option = sc.nextLine();
      String heuristic = "";
      String h="0";
      String algorithm = ""; 
      String value= "";
      if(Integer.parseInt(option)== 1 || Integer.parseInt(option)== 3) {
          System.out.println("For Heuristic 1(Hamming Distance) - > Enter \"1\" \nFor Heuristic 2( Inversion Calcualtion) - > Enter \"2\" ");
          h = sc.nextLine();
      }
      
      if(Integer.parseInt(option)==1)
      {
    	  algorithm ="Best First Search"; 
    	  d.FILEPATH = d.FILEPATH+  "puzzleBFS";
      }
      else if(Integer.parseInt(option)==2)
      {
    	  algorithm ="Depth First Search";
    	  d.FILEPATH = d.FILEPATH+  "puzzleDFS.txt";

      }
      else
      {
    	  algorithm ="A Start Search";
    	  d.FILEPATH = d.FILEPATH+ "puzzleAs";
      }
      if(Integer.parseInt(h)== 1)
      {
    	  value ="hamming distance";
    	  heuristic= "h1";
    	  d.FILEPATH = d.FILEPATH+ "-h1"+".txt";

      }
      else if(Integer.parseInt(h)== 2){
    	  value ="Inversion Calculation"; 
    	  heuristic= "h2";

    	  d.FILEPATH = d.FILEPATH+ "-h2"+".txt";

      }
          
      
      System.out.println("\n<-- Search Starts-- > \n");
      
      System.out.println("Solving \"12 Puzzle Problem\" using " + "\""+algorithm+ "\"" + " with "+ "\""+value+ "\"" + " heuristic \n" );

      long startTime = System.nanoTime();
      long beforeUsedMem=Runtime.getRuntime().totalMemory()-Runtime.getRuntime().freeMemory();


      if(Integer.parseInt(option)==1)
      {
    	  d.bfs(d.initial_state, heuristic);
      }
      else if(Integer.parseInt(option) == 2) {
    	  d.dfs(d.initial_state);

      }
      else
      {
    	  d.astar(d.initial_state, heuristic);
      }
      long endTime   = System.nanoTime();
      long totalTime = endTime - startTime;
      long afterUsedMem=Runtime.getRuntime().totalMemory()-Runtime.getRuntime().freeMemory();
      long actualMemUsed=afterUsedMem-beforeUsedMem;
   
      // displaying results
      Collections.reverse(d.moves);
      System.out.println("\n<-- Search Ends-- > \n");
      System.out.println("Moves : " + d.moves.toString());
	  System.out.println("max Open List size : " + d.max_frontier_size);
	  System.out.println("search_depth:  : " + d.search_depth );
	  System.out.println("max_search_depth: : " + d.max_search_depth);
      System.out.println("running_time: " +  totalTime );
      System.out.println("Memory used: " +  actualMemUsed );
     
 }
}



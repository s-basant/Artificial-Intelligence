
import java.util.Arrays;

/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/**
 *
 * @author s_basant
 */
public class State {
    
    int[] state;
    State parent;
    int move;
    int depth;
    int cost;
    int key;
    int gScore;
    int fScore;

    int  priority;
    char position;
    String map;
    State(int[] state,State parent,int move,int depth, int cost, int key)
    {
    	
        this.state = state;
        this.parent = parent;
        this.move =move;
        this.depth =depth;
        this.cost =cost;
        this.key =key;
        this.map = Arrays.toString(state);
        gScore= Integer.MAX_VALUE;
        fScore= Integer.MAX_VALUE;
        if(state != null){
        for(int i= 0 ; i < state.length; i++)
        {
            if(state[i]==0)
            {
            this.position =  ((char) (97+  i));
                 
            }
        
        }
        }
        else 
            this.position =  '0';
        
    }
    
    State(){}
    
}

import java.util.Comparator;
public class NodeCompare implements Comparator<State> {

	@Override
	public int compare(State state1, State state2) {
		// TODO Auto-generated method stub
		if (state1.priority > state2.priority){
			return 1;
		}
            if (state1.priority < state2.priority){
			return -1;
		}
		return 0;
	}

	

}

package ch.heig.gre.groupD;

import ch.heig.gre.maze.MazeBuilder;
import ch.heig.gre.maze.MazeGenerator;
import ch.heig.gre.maze.Progression;

import java.util.*;


public final class DfsGenerator implements MazeGenerator {
  @Override
  public void generate(MazeBuilder builder, int from) {

    // crashes if the first vertex doesn't exist
    if (!builder.topology().vertexExists(from)) {
      throw new UnsupportedOperationException("Vertex does not exist: " + from);
    }

    Stack<Integer> stack = new Stack<>();

    //push the first vertex
    stack.push(from);
    builder.progressions().setLabel(from, Progression.PROCESSED);
    Random rand = new Random();

    while (!stack.isEmpty()) {


      int v = stack.pop();
      List<Integer> neighbours = builder.topology().neighbors(v);

      Collections.shuffle(neighbours, rand); // randomly chose the next vertex

      // remove all neighbours that have already been visited
      neighbours.removeIf(neighbour -> !builder.progressions().getLabel(neighbour).equals(Progression.PENDING));

      // chose a random neighbour from the unvisited ones
      if (!neighbours.isEmpty()) {
        int neighbour = neighbours.get(rand.nextInt(neighbours.size()));
        stack.push(v);
        builder.removeWall(neighbour, v);
        builder.progressions().setLabel(neighbour, Progression.PROCESSED);
        stack.push(neighbour);
      }
    }
  }

  @Override
  public boolean requireWalls() {
    return true;
  }
}

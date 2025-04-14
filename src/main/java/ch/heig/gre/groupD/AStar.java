package ch.heig.gre.groupD;

import ch.heig.gre.graph.GridGraph2D;
import ch.heig.gre.graph.PositiveWeightFunction;
import ch.heig.gre.graph.VertexLabelling;
import ch.heig.gre.maze.GridMazeSolver;

import java.util.Arrays;
import java.util.PriorityQueue;

public final class AStar implements GridMazeSolver {
  public enum Heuristic {
    DIJKSTRA, INFINITY_NORM, EUCLIDEAN_NORM, MANHATTAN, K_MANHATTAN
  }

  /** Heuristique utilisée pour l'algorithme A*. */
  private final Heuristic heuristic;

  /** Facteur multiplicatif de la distance de Manhattan utilisé par l'heuristique K-Manhattan. */
  private final int kManhattan;

  public AStar(Heuristic heuristic) {
    this(heuristic, 1);
  }

  public AStar(Heuristic heuristic, int kManhattan) {
    this.heuristic = heuristic;
    this.kManhattan = kManhattan;
  }


  @Override
  public Result solve(GridGraph2D grid,
                      PositiveWeightFunction weights,
                      int source,
                      int destination,
                      VertexLabelling<Boolean> processed) {
    //throw new UnsupportedOperationException("Not implemented yet");

    //Pour chaque sommet i, posé dist = infini et p[j] = NULL
    int[] distance = new int[grid.nbVertices()];
    Arrays.fill(distance, 100000); // devrait etre inf
    Object[] pred = new Object[grid.nbVertices()];
    Arrays.fill(pred, null);

    //calcul de l'heuristique

    //Queue de priorité des sommets à traiter
    PriorityQueue<Integer> priorityQueue = new PriorityQueue<>();
    priorityQueue.add(source);
    distance[source] = 0;

    while (!priorityQueue.isEmpty()) {
      int vertex = priorityQueue.poll();

      //on a trouvé la destination
      if (vertex == destination) {
        break;
      }

      //Pour chaque successeur
      for (Integer i : grid.neighbors(vertex)) {
        if (pred[i] == null) {
          if (distance[i] > distance[vertex] /* + poids ij */) {
            if (distance[i] == 100000) { //INFINI
              //calculer heuristique de j

            }
            distance[i] = distance[vertex] /* +poids ij */;
            pred[i] = vertex;
            if (!priorityQueue.contains(i)) {
              priorityQueue.add(i);
            }
          }
        }
      }

    }

    return null;
  }
}
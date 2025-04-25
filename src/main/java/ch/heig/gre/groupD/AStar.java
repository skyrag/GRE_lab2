package ch.heig.gre.groupD;

import ch.heig.gre.graph.GridGraph2D;
import ch.heig.gre.graph.PositiveWeightFunction;
import ch.heig.gre.graph.VertexLabelling;
import ch.heig.gre.maze.GridMazeSolver;

import java.util.*;

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
    Arrays.fill(distance, Integer.MAX_VALUE); // devrait etre inf
    int[] pred = new int[grid.nbVertices()];
    Arrays.fill(pred, -1);


    //calcul de l'heuristique CAS DIJKSTRA
    int[] heuristicTab = new int[grid.nbVertices()];
    Arrays.fill(heuristicTab, 0); //devrait être null


    //Queue de priorité des sommets à traiter
    PriorityQueue<Integer> priorityQueue = new PriorityQueue<>(
            Comparator.comparingInt(v -> distance[v] + heuristicTab[v]));
    priorityQueue.add(source);
    distance[source] = 0;

    //nombre de sommet traité
    int nbProcessed = 0;

    while (!priorityQueue.isEmpty()) {
      int vertex = priorityQueue.poll();

      //le sommet a déjà été traité
      if (processed.getLabel(vertex) == true) {
        continue;
      }
      processed.setLabel(vertex, true);
      nbProcessed++;

      //on a trouvé la destination
      if (vertex == destination) {
        break;
      }

      //Pour chaque successeur
      for (Integer i : grid.neighbors(vertex)) {
        if (distance[i] > distance[vertex] + weights.get(vertex, i)) {

          distance[i] = distance[vertex] + weights.get(vertex, i);
          pred[i] = vertex;

          if (!processed.getLabel(i)) {
            priorityQueue.add(i);
          }
        }
      }
    }
    //creation de la list
    List<Integer> parcours = new ArrayList<>();
    if (distance[destination] != -1) { //si -1 -> pas de résultat
      int current_vertex = destination;
      while (current_vertex != -1) {
        parcours.add(current_vertex);
        current_vertex = pred[current_vertex];
      }
      Collections.reverse(parcours);
    }

    return new Result(parcours, distance[destination], nbProcessed);
  }
}
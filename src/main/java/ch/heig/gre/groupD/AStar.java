package ch.heig.gre.groupD;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;

import ch.heig.gre.graph.GridGraph2D;
import ch.heig.gre.graph.PositiveWeightFunction;
import ch.heig.gre.graph.VertexLabelling;
import ch.heig.gre.maze.GridMazeSolver;

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
    double[] heuristicTab = new double[grid.nbVertices()];
    Arrays.fill(heuristicTab, 0);
    //coordonnées destination
    int destX = destination % grid.width();
    int destY = destination / grid.height();
    int cMin = weights.minWeight();

    //Queue de priorité des sommets à traiter
    PriorityQueue<Integer> priorityQueue = new PriorityQueue<>(
            Comparator.comparingDouble(v -> (double) distance[v] + heuristicTab[v]));
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
      for (Integer successor : grid.neighbors(vertex)) {
        if (distance[successor] > distance[vertex] + weights.get(vertex, successor)) {
          
          //calculer heuristic si distance = infini
          if (distance[successor] == Integer.MAX_VALUE) {
            //coordonnées actuel 
              int vX = successor % grid.width();
              int vY = successor / grid.height();
              switch (heuristic) {
                case DIJKSTRA -> {
                  //valeur deja à 0
                  break;
                }
                case INFINITY_NORM -> {
                  heuristicTab[successor] = cMin * Math.max(Math.abs(destX - vX), Math.abs(destY - vY));
                  break;
                }
                case EUCLIDEAN_NORM -> {
                  heuristicTab[successor] = cMin * Math.floor(Math.hypot(destX - vX, destY - vY));
                  break;
                }
                case MANHATTAN -> {
                  heuristicTab[successor] = cMin * (Math.abs(destX - vX) + Math.abs(destY - vY));
                  break;
                }
                case K_MANHATTAN -> {
                  heuristicTab[successor] = kManhattan * cMin * (Math.abs(destX - vX) + Math.abs(destY - vY));
                  break;
                }
                default -> {break;}
              }
          }

          distance[successor] = distance[vertex] + weights.get(vertex, successor);
          pred[successor] = vertex;

          //Mise a jour ou ajout dans la priority queue
          priorityQueue.remove(successor); //remove seulement s'il est présent
          priorityQueue.add(successor);
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
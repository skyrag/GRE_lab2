package ch.heig.gre.groupD;

import ch.heig.gre.graph.GridGraph;
import ch.heig.gre.graph.GridGraph2D;
import ch.heig.gre.graph.PositiveWeightFunction;
import ch.heig.gre.maze.BoolVertexLabelling;
import ch.heig.gre.maze.MazeBuilder;
import ch.heig.gre.maze.MazeGenerator;
import ch.heig.gre.maze.impl.GridMazeBuilder;
import ch.heig.gre.maze.impl.MazeTuner;
import ch.heig.gre.maze.impl.ShenaniganWeightFunction;

import java.util.Date;
import java.util.random.RandomGenerator;

// TODO: renommer le package (Shift + F6) selon la lettre attribuée à votre groupe

public final class Experiment {
  /** Dimension de la grille (carrée) */
  private static final int SIDE = 1100;

  /** Sommets source et destination pour les expériences */
  private static final int SRC = 550500;
  private static final int DST = 660600;

  /** Nombre de grilles à générer pour chaque expérience */
  private static final int N = 100;

  /** Topologie de la grille */
  private static final GridGraph2D TOPOLOGY;

  static {
    var g = new GridGraph(SIDE);
    GridGraph.bindAll(g);
    TOPOLOGY = g;
  }

  /** Paramètres des expériences à réaliser */
  private static final Params[] PARAMS = {
      new Params(
          "Relief très peu dense, labyrinthe très ouvert",
          new double[]{0, 0.15, 20, 1, 20}),
      new Params(
          "Relief très peu dense, labyrinthe assez ouvert",
          new double[]{0, 0.1, 20, 1, 20}),
      new Params(
          "Relief très peu dense, labyrinthe peu ouvert",
          new double[]{0, 0.01, 20, 1, 20}),
      new Params(
          "Relief dense, labyrinthe moyennement ouvert",
          new double[]{0.25, 0.05, 25, 5, 20}),
      new Params(
          "Relief très dense, labyrinthe moyennement ouvert",
          new double[]{0.5, 0.05, 25, 5, 20}),
      new Params(
          "Relief très dense et fortement pondéré, labyrinthe moyennement ouvert",
          new double[]{0.5, 0.05, 25, 5, 100})
  };

  /**
   * <p>Paramètres d'une expérience, avec une description approximative de leurs effets sur la génération.</p>
   *
   * <p>À passer en paramètre de la méthode {@link #generateGrid} pour générer un labyrinthe.</p>
   *
   * @param description Description de l'expérience
   * @param parameters  Paramètres de l'expérience
   */
  record Params(String description, double[] parameters) {}

  public static void main(String[] args) {
    RandomGenerator rng = RandomGenerator.getDefault();
    Date start = new Date();

    for (Params params : PARAMS) {
      System.out.println("Expérience : " + params.description());

      // initialisation des variables utilisé pour relevé la performance de chaque algorithme
      long totalLengthD = 0, totalVerticesD = 0;
      long totalLengthH1 = 0, totalVerticesH1 = 0;
      long totalLengthH2 = 0, totalVerticesH2 = 0;
      long totalLengthH3 = 0, totalVerticesH3 = 0;
      long[] totalLengthH4 = new long[7], totalVerticesH4 = new long[7];

      int[] optimalSolutions = new int[7];
      double[] totalErrors = new double[7];

      double[] minError = new double[7];
      double[] maxError = new double[7];

      AStar DSolver = new AStar(AStar.Heuristic.DIJKSTRA);
      AStar H1Solver = new AStar((AStar.Heuristic.INFINITY_NORM));
      AStar H2Solver = new AStar((AStar.Heuristic.EUCLIDEAN_NORM));
      AStar H3Solver = new AStar((AStar.Heuristic.MANHATTAN));
      AStar[] H4Solver = new AStar[7];

      for (int i = 0; i < 7; i++) {
        minError[i] = Double.MAX_VALUE;
        maxError[i] = Double.MIN_VALUE;
        totalErrors[i] = 0;
        totalLengthH4[i] = 0;
        totalVerticesH4[i] = 0;
        H4Solver[i] = new AStar(AStar.Heuristic.K_MANHATTAN, i + 2);
      }


      for (int i = 0; i < N; i++) {
        GenerationResult result = generateGrid(new DfsGenerator(), params.parameters(), rng);
        GridGraph2D maze = result.maze();
        PositiveWeightFunction weights = result.weights();

        // Dijkstra
        var resultD = DSolver.solve(maze, weights, SRC, DST, new BoolVertexLabelling(maze.width() * maze.height()));
        totalLengthD += resultD.length();
        totalVerticesD += resultD.treatments();

        // A* avec heuristique H1 : INFINITY_NORM
        var resultH1 = H1Solver.solve(maze, weights, SRC, DST, new BoolVertexLabelling(maze.width() * maze.height()));
        totalLengthH1 += resultH1.length();
        totalVerticesH1 += resultH1.treatments();

        // A* avec heuristique H2 : EUCLIDIAN_NORM
        var resultH2 = H2Solver.solve(maze, weights, SRC, DST, new BoolVertexLabelling(maze.width() * maze.height()));
        totalLengthH2 += resultH2.length();
        totalVerticesH2 += resultH2.treatments();

        // A* avec heuristique H3 : MANHATTAN
        var resultH3 = H3Solver.solve(maze, weights, SRC, DST, new BoolVertexLabelling(maze.width() * maze.height()));
        totalLengthH3 += resultH3.length();
        totalVerticesH3 += resultH3.treatments();

        // A* avec heuristique H4 : K_MANHATTAN (où k varie de 2 à 8)
        for (int k = 0; k <= 6; k++){
          var resultH4 = H4Solver[k].solve(maze, weights, SRC, DST, new BoolVertexLabelling(maze.width() * maze.height()));
          totalLengthH4[k] += resultH4.length();
          totalVerticesH4[k] += resultH4.treatments();

          // check si le chemin est optimal
          if (Math.abs(resultH4.length() - resultD.length()) == 0) {
            optimalSolutions[k]++;
          }
          // Calcul erreur relative
          double error = ((double) (resultH4.length() - resultD.length())) / resultD.length();
          totalErrors[k] += error;
          minError[k] = Math.min(minError[k], error);
          maxError[k] = Math.max(maxError[k], error);

        }

        System.out.println("nombre de labyrinthe réalisé :" + (i + 1) + "/" + N);
      }

      // Moyennes
      double meanLengthDijkstra = (double) totalLengthD / N;
      double meanVerticesDijkstra = (double) totalVerticesD / N;
      double meanLengthAStar = (double) totalLengthH1 / N;
      double meanVerticesAStar = (double) totalVerticesH1 / N;
      double meanLengthH2 = (double) totalLengthH2 / N;
      double meanVerticesH2 = (double) totalVerticesH2 / N;
      double meanLengthH3 = (double) totalLengthH3 / N;
      double meanVerticesH3 = (double) totalVerticesH3 / N;

      double[] percentageOptimal = new double[7];
      double[] averageError = new double[7];
      double[] meanVerticesH4 = new double[7];
      double[] gainVertices = new double[7];
      for (int k = 0; k <= 6; k++){
        percentageOptimal[k] = 100.0 * optimalSolutions[k] / N;
        averageError[k] = totalErrors[k] / N;
        meanVerticesH4[k] = (double) totalVerticesH4[k] / N;
        gainVertices[k] = gain(meanVerticesDijkstra , meanVerticesH4[k]);
      }


      // Affichage
      System.out.printf("""
            Résultats pour : %s
              - Dijkstra : longueur moyenne = %.2f, moyenne de sommets traités = %.2f
              - A* : longueur moyenne = %.2f, moyenne de sommets traités = %.2f (gain : %.2f%%)
              - A* H2 : longueur moyenne = %.2f, moyenne de sommets traités = %.2f (gain : %.2f%%)
              - A* H3 : longueur moyenne = %.2f, moyenne de sommets traités = %.2f (gain : %.2f%%)
            """,
              params.description(),
              meanLengthDijkstra, meanVerticesDijkstra,
              meanLengthAStar, meanVerticesAStar, gain(meanVerticesDijkstra, meanVerticesAStar),
              meanLengthH2, meanVerticesH2, gain(meanVerticesDijkstra, meanVerticesH2),
              meanLengthH3, meanVerticesH3, gain(meanVerticesDijkstra, meanVerticesH3)
      );

      for (int k = 0; k <= 6; k++){
        System.out.printf("""
            Résultats pour H4 avec K = %d :
              - Pourcentage de solutions optimales : %.2f%%
              - Erreur relative minimale : %.4f
              - Erreur relative moyenne : %.4f
              - Erreur relative maximale : %.4f
              - Gain moyen sur sommets traités : %.2f%%
            """,
                k + 2,
                percentageOptimal[k],
                minError[k],
                averageError[k],
                maxError[k],
                gainVertices[k]
        );
      }
      System.out.println("\n--------------------------\n");
    }
    Date end = new Date();
    System.out.println("time took to realise all of it : " + ((end.getTime() - start.getTime())/1000.0) + " seconds");
  }

  // fonction qui calcule le gain par rapport a une valeur reférante
  private static double gain(double ref, double value) {
    return (ref - value) / ref * 100.0;
  }


  /**
   * Résultat de la méthode {@link #generateGrid}, fournit un labyrinthe et une fonction de pondération des arêtes
   * associée.
   *
   * @param maze    labyrinthe généré
   * @param weights Fonction de pondération associée
   */
  private record GenerationResult(GridGraph2D maze, PositiveWeightFunction weights) {}

  /**
   * Génère un labyrinthe en forme de grille avec un générateur donné et des réglages spécifiques pour le relief et
   * l'ouverture (i.e. densité de murs) du labyrinthe.
   *
   * @param generator      Générateur de labyrinthe.
   * @param tuneParameters Paramètres de réglage du relief et de l'ouverture du labyrinthe.
   * @param rng            Générateur de nombres aléatoires.
   * @return Un {@link GenerationResult} contenant le labyrinthe et la fonction de pondération associée.
   */
  private static GenerationResult generateGrid(MazeGenerator generator, double[] tuneParameters, RandomGenerator rng) {
    GridGraph maze = new GridGraph(SIDE);
    if (!generator.requireWalls())
      GridGraph.bindAll(maze);

    MazeBuilder builder = new GridMazeBuilder(TOPOLOGY, maze);
    generator.generate(builder, 0);

    MazeTuner tuner = new MazeTuner()
        .setRandomGenerator(rng)
        .setReliefDensityFactor(tuneParameters[0])
        .setWallRemovalProbability(tuneParameters[1])
        .setReliefRadiusRatio(tuneParameters[2])
        .setReliefSummitsPerRange((int) tuneParameters[3])
        .setReliefMaxSummitWeight((int) tuneParameters[4]);

    tuner.removeWalls(TOPOLOGY, maze);
    int[] weights = tuner.generateRelief(SIDE, SIDE);
    PositiveWeightFunction wf = new ShenaniganWeightFunction(weights, tuner.getReliefMinWeight());

    return new GenerationResult(maze, wf);
  }
}

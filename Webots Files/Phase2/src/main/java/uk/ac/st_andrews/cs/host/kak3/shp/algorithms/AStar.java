package uk.ac.st_andrews.cs.host.kak3.SHP.algorithms;

import uk.ac.st_andrews.cs.host.kak3.SHP.framework.ControlParameters;
import uk.ac.st_andrews.cs.host.kak3.SHP.framework.Framework;
import uk.ac.st_andrews.cs.host.kak3.SHP.algorithms.samplers.SampleTechnique;

import java.util.HashSet;
import java.util.PriorityQueue;

public class AStar extends TreeBasedAlgorithm {
    private final SampleTechnique sampler;
    private final double minCost;

    /**
     * A* search technique, each iteration builds up a sequence of moves, scoring them using the application's cost
     * function, and executes the first move along the sequence.
     * @param sampler the sample technique to use to determine the neighbours of a control space point.
     * @param minCost the cost that much be reached before a move sequence can be executed.
     */
    public AStar(SampleTechnique sampler, double minCost) {
        this.sampler = sampler;
        this.minCost = minCost;
    }

    @Override
    public void iteration(Framework framework) {
        PriorityQueue<AStarMoveSequence> frontier = new PriorityQueue<>();
        HashSet<ControlParameters> explored = new HashSet<>();

        // Initially add all neighbours of the current move to the frontier. One of these will be eventually executed,
        // if a path is found in time.
        for (ControlParameters neighbour : sampler.getSamplesFrom(framework.getCurrent())) {
            AStarMoveSequence sequence = new AStarMoveSequence(neighbour, 0);
            sequence.setScore(framework.getSequenceCost(sequence) + 1);
            frontier.add(sequence);
            explored.add(neighbour);
        }

        while (!frontier.isEmpty()) {
            AStarMoveSequence node = frontier.remove();

            // If we're close enough to the goal (cost low enough), execute the first move along the path created.
            if (framework.getSequenceCost(node) < minCost) {
                doMove(framework, node);
                return;
            }

            // Expand the best node in the frontier, and add all of its neighbours if they weren't already present.
            for (ControlParameters neighbour : sampler.getSamplesFrom(node.getLast())) {
                AStarMoveSequence sequence = new AStarMoveSequence(node.getParameters(), node.getScore());
                sequence.addMove(neighbour);

                ControlParameters combined = framework.combineMoves(sequence);

                if (!explored.contains(combined)) {
                    explored.add(combined);
                    double score = framework.getSequenceCost(sequence) + sequence.getParameters().size();
                    if (Double.isFinite(score)) {
                        sequence.setScore(score);
                        frontier.add(sequence);
                    }
                }
            }
        }

        throw new Error("Failed to find route.");
    }

    @Override
    public void terminate(Framework framework) {
        framework.finished();
    }
}

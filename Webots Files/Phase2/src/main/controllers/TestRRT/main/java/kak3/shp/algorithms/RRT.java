package main.java.kak3.shp.algorithms;



import main.java.kak3.shp.algorithms.samplers.SampleTechnique;
import main.java.kak3.shp.framework.ControlParameters;
import main.java.kak3.shp.framework.Framework;
import main.java.kak3.shp.framework.distance.Euclidean;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

import java.util.HashSet;
import java.util.Random;
import java.util.Set;

public class RRT extends TreeBasedAlgorithm {
    private final double minCost;
    private final double maxCost;

    private final SampleTechnique sampler;
    private final Random random = new Random();

    /**
     * RRTs implementation, planning a path in control space, and then executing the first move along that plan.
     * ineffective in practice, as discussed in report.
     * @param sampler the sampler instance to use. Its getSamplesFrom method is not actually used - it is simply used
     *                as a container for the minimum, maximum and step size parameters.
     * @param minCost the cost that must be reached for the algorithm to terminate.
     * @param maxCost the maximum allowed cost for a move. Moves with cost greater than this will not be considered.
     */
    public RRT(SampleTechnique sampler, double minCost, double maxCost) {
        this.sampler = sampler;
        this.minCost = minCost;
        this.maxCost = maxCost;
    }

    /**
     * Produces a random control parameters between those allowed by the minimum and maximum values.
     * @param dimensions the number of dimensions of control space.
     * @return a random control space point.
     */
    private ControlParameters genRandomSample(int dimensions) {
        double[] vals = new double[dimensions];
        double range = sampler.getMaxValue() - sampler.getMinValue();
        for (int i = 0; i < dimensions; i++) {
            vals[i] = (random.nextDouble() * range) + sampler.getMinValue();
        }

        return new ControlParameters(vals);
    }

    /**
     * Returns the nearest node out of nodes, that is closes to goal, using the framework for move combining.
     * @param goal the goal to attempt to get closest to.
     * @param nodes the available nodes to consider.
     * @param framework the framework instance.
     * @return the nearest node in nodes.
     */
    private MoveSequence getNearestNode(ControlParameters goal, Set<MoveSequence> nodes, Framework framework) {
        Double minDistance = null;
        MoveSequence nearest = null;

        for (MoveSequence node : nodes) {
            double dist = Euclidean.euclideanDistance(goal, framework.combineMoves(node));
            if ((nearest == null || dist < minDistance)) {
                minDistance = dist;
                nearest = node;
            }
        }

        return nearest;
    }

    /**
     * Expands a given node towards the goal, using the framework to combine moves.
     * @param goal the goal to get closer to.
     * @param node the node to expand.
     * @param framework the framework instance.
     * @return a new sequence of moves, as one of the existing sequences in nodes, with an extra move appended.
     */
    private MoveSequence expandTowards(ControlParameters goal, MoveSequence node, Framework framework) {
        RealVector goalVector = new ArrayRealVector(goal.getArray());
        RealVector nodeVector = new ArrayRealVector(framework.combineMoves(node).getArray());

        RealVector direction = goalVector.subtract(nodeVector).unitVector();

        RealVector move = direction.mapMultiply(sampler.getStepSize());
        MoveSequence newSequence = new MoveSequence(node);
        newSequence.addMove(new ControlParameters(move.toArray()));
        return newSequence;
    }

    @Override
    public void iteration(Framework framework) {
        Set<MoveSequence> nodes = new HashSet<>();
        double[] current = framework.getCurrent().getArray();
        ControlParameters zero = new ControlParameters(new double[current.length]);
        nodes.add(new MoveSequence(zero));

        while (true) {
            ControlParameters randomGoal = genRandomSample(framework.getCurrent().getArray().length);
            MoveSequence nearest = getNearestNode(randomGoal, nodes, framework);
            MoveSequence newNode = expandTowards(randomGoal, nearest, framework);

            double cost = framework.getSequenceCost(newNode.translate(framework.getCurrent()));

            // Skip moves to places with cost too high (e.g. into an obstacle).
            if (cost > maxCost) {
                continue;
            }

            if (cost < minCost) {
                doMove(framework, newNode, 1);
                return;
            }

            nodes.add(newNode);
        }
    }

    @Override
    public void terminate(Framework framework) {

    }

}

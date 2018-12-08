package main.java.kak3.shp.algorithms;


import main.java.kak3.shp.algorithms.samplers.SampleTechnique;
import main.java.kak3.shp.framework.ControlParameters;
import main.java.kak3.shp.framework.Framework;

public class FractionalProgress implements SearchAlgorithm {
    private final SampleTechnique sampler;
    private final double minCost;

    /**
     * Maximised fractional progress technique. Each iteration executes the move which maximises past / (past + future),
     * where past is the difference between current and sample point, and future is the cost at the same point.
     * @param sampler the sample technique to use to determine the neighbours of a control space point.
     * @param minCost the cost that must be reached to consider the algorithm finished.
     */
    public FractionalProgress(SampleTechnique sampler, double minCost) {
        this.sampler = sampler;
        this.minCost = minCost;
    }

    @Override
    public void iteration(Framework framework) {
        Double bestProgress = null;
        ControlParameters bestMove = null;

        double current = framework.getCurrentCost();
        if (current < minCost) {
            framework.finished();
            return;
        }

        for (ControlParameters sample : sampler.getSamplesFrom(framework.getCurrent())) {
            double past = current - framework.getCost(sample);
            double future = framework.getCost(sample);

            double progress = past / (past + future);
            if (bestProgress == null || progress > bestProgress) {
                bestProgress = progress;
                bestMove = sample;
            }
        }

        framework.move(bestMove);
    }

    @Override
    public void terminate(Framework framework) {
        framework.finished();
    }


}

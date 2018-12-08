package main.java.kak3.shp.algorithms;

import main.java.kak3.shp.algorithms.samplers.SampleTechnique;
import main.java.kak3.shp.framework.ControlParameters;
import main.java.kak3.shp.framework.Framework;


import java.util.LinkedList;
import java.util.List;
import java.util.Random;

public class SimulatedAnnealing implements SearchAlgorithm {
    private double[] bestVals;
    private Double bestCost;

    private final Random random = new Random();

    private final SampleTechnique sampler;
    private final double energyCoefficient;
    private double temperature;
    private final double coolingRate;

    /**
     * The simulated annealing search technique, starts "hot", where a worse move can be attempted, and cools down over
     * time, effectively becoming gradient descent.
     * @param sampler the sample technique to use to determine the neighbours of a control space point.
     */
    public SimulatedAnnealing(SampleTechnique sampler, double energyCoefficient, double temperature, double coolingRate) {
        this.sampler = sampler;
        this.energyCoefficient = energyCoefficient;
        this.temperature = temperature;
        this.coolingRate = coolingRate;
    }

    /**
     * Returns a boolean indicating whether a move should be accepted, given the current and neighbour costs.
     * @param currentCost the cost at the current point.
     * @param neighbourCost the cost of the neighbour evaluated. Indicates whether this should be moved to.
     * @return a boolean indicating whether this move should be executed.
     */
    private boolean accept(double currentCost, double neighbourCost) {
        // Always accept a better solution.
        if (neighbourCost < currentCost) {
            return true;
        }

        // Calculate a probability of accepting a worse solution.
        double probability = Math.exp(energyCoefficient * (currentCost - neighbourCost) / temperature);
        return probability > random.nextDouble();
    }

    @Override
    public void iteration(Framework framework) {
        double[] vals = framework.getCurrent().getArray();

        if (temperature < 1) {
            terminate(framework);
            return;
        }

        List<ControlParameters> samples = new LinkedList<>(sampler.getSamplesFrom(new ControlParameters(vals)));
        double neighbourVals[] = samples.get(random.nextInt(samples.size())).getArray();

        double currentCost = framework.getCost(new ControlParameters(vals));
        double neighbourCost = framework.getCost(new ControlParameters(neighbourVals));

        if (accept(currentCost, neighbourCost)) {
            framework.move(new ControlParameters(neighbourVals));

            if (bestCost == null || neighbourCost < bestCost) {
                bestVals = neighbourVals;
                bestCost = neighbourCost;
            }
        }

        temperature *= 1 - coolingRate;
    }

    @Override
    public void terminate(Framework framework) {
        framework.move(new ControlParameters(bestVals));
        framework.finished();
    }


}

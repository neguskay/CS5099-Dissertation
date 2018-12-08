package main.java.kak3.shp.application.neuralnet;

import main.java.kak3.shp.algorithms.MoveSequence;
import main.java.kak3.shp.algorithms.WindingUtil;
import main.java.kak3.shp.application.Application;
import main.java.kak3.shp.framework.ControlParameters;
import org.encog.ml.MLMethod;
import org.encog.ml.train.BasicTraining;
import org.encog.neural.networks.structure.NetworkCODEC;
import org.encog.neural.networks.training.propagation.TrainingContinuation;
import sun.reflect.generics.reflectiveObjects.NotImplementedException;


import java.util.List;

public class NeuralApplication extends BasicTraining implements Application {
    private final NeuralProblem problem;

    private ControlParameters previousMove;
    private double wind = 0;

    private int epoch = 0;

    /**
     * Wraps a neural problem with the appropriate interface for the search framework.
     * @param problem the problem instance to solve.
     */
    public NeuralApplication(NeuralProblem problem) {
        this.problem = problem;
    }

    @Override
    public void iteration() {

    }

    @Override
    public boolean canContinue() {
        return false;
    }

    @Override
    public TrainingContinuation pause() {
        final TrainingContinuation result = new TrainingContinuation();
        result.setTrainingType(this.getClass().getSimpleName());
        return result;
    }

    @Override
    public void resume(TrainingContinuation trainingContinuation) {

    }

    @Override
    public MLMethod getMethod() {
        return null;
    }

    private double move(ControlParameters parameters, boolean realMove) {
        if (realMove) {
            epoch++;
            System.out.println("Epoch #" + epoch + " Error:" + getError());

            if (previousMove != null) {
                wind += WindingUtil.calcWindingDelta(previousMove, parameters);
            }
            previousMove = parameters;
        }

        NetworkCODEC.arrayToNetwork(parameters.getArray(), problem.getNetwork());
        setError(problem.getNetwork().calculateError(problem.getTrainingSet()));
        return getError();
    }

    @Override
    public double getCurrentCost() {
        return getError();
    }

    @Override
    public void doMove(ControlParameters parameters) {

    }

    @Override
    public double simulateMove(ControlParameters parameters) {
        return 0;
    }


    @Override
    public double simulateMoveSequence(MoveSequence sequence) {
        return simulateMove(sequence.getLast());
    }

    @Override
    public void finished() {
        System.out.println("Neural Network Results:");
        System.out.println("Total error: " + getError());

        problem.finished();
    }

    @Override
    public ControlParameters initialise() {
        return problem.getInitialState();
    }

    @Override
    public int getParamCount() {
        return problem.getNetwork().encodedArrayLength();
    }

    @Override
    public ControlParameters combineMoves(List<ControlParameters> parameters) {
        return parameters.get(parameters.size() - 1);
    }

    @Override
    public boolean canMoveDirectly() {
        return true;
    }

    @Override
    public double getWind() {
        return wind;
    }
}

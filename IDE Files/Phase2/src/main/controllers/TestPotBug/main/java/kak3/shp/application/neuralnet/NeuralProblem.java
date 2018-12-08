package main.java.kak3.shp.application.neuralnet;

import main.java.kak3.shp.framework.ControlParameters;
import org.encog.Encog;
import org.encog.ml.data.MLDataSet;
import org.encog.neural.networks.BasicNetwork;


/**
 * Contains the encog neural network.
 */
public abstract class NeuralProblem {
    BasicNetwork network;
    MLDataSet trainingSet;

    BasicNetwork getNetwork() {
        return this.network;
    }

    MLDataSet getTrainingSet() {
        return this.trainingSet;
    }

    /**
     * Neural networks are allowed to provide an initial state, which may be overridden by algorithms depending on
     * settings. Returning null indicates that the initial state does not matter and should be left to the
     * framework/algorithm.
     */
    abstract ControlParameters getInitialState();

    void shutdown() {
        Encog.getInstance().shutdown();
    }

    public abstract void finished();
}

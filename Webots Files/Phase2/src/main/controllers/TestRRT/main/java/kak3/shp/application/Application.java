package main.java.kak3.shp.application;



import main.java.kak3.shp.algorithms.MoveSequence;
import main.java.kak3.shp.framework.ControlParameters;

import java.util.List;

public interface Application {
    /**
     * Should return the cost of the current state.
     * @return the cost.
     */
    double getCurrentCost();

    /**
     * Should execute the provided control parameters.
     * @param parameters the parameters.
     */
    void doMove(ControlParameters parameters);

    /**
     * Should *NOT* execute the provided paramters, but approximate the cost of the move if it were to be.
     * @param parameters the parameters.
     * @return the estimated cost.
     */
    double simulateMove(ControlParameters parameters);

    /**
     * Same as simulateMove, but for a sequence of moves to be executed in turn.
     * @param sequence the sequence.
     * @return the estimated cost.
     */
    double simulateMoveSequence(MoveSequence sequence);

    /**
     * Called to indicate that some other component now indicates that it has finished search. This could be for any
     * reason, such as reaching its goal, or timeout.
     */
    void finished();

    /**
     * Applications are allowed to provide an initial control space point to start with. Return null if this is not
     * desired.
     * @return the initial parameters, or null to use random ones.
     */
    ControlParameters initialise();

    /**
     * The dimensionality of the control space.
     * @return the number of dimensions.
     */
    int getParamCount();

    /**
     * "Combine" a sequence of moves into a single move. This is NOT required to accurately represent the combination,
     * such that the result of executing in turn is the same returned by this, but it should be some indication of
     * what it means to combine moves.
     */
    ControlParameters combineMoves(List<ControlParameters> parameters);

    /**
     * Indicates whether this application allows moving directly to any control space point, or if a sequence must be
     * executed to realise the goal.
     * @return a boolean indicating whether it is acceptable to move to the last move in a sequence rather than
     * executing in turn.
     */
    boolean canMoveDirectly();

    /**
     * Gets the current wind from the initial heading.
     * @return the wind.
     */
    double getWind();
}

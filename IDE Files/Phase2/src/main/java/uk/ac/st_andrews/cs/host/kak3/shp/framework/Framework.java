package uk.ac.st_andrews.cs.host.kak3.SHP.framework;

import uk.ac.st_andrews.cs.host.kak3.SHP.algorithms.MoveSequence;

public interface Framework extends Runnable {
    /**
     * Should execute a move in the application.
     * @param parameters the move to execute.
     */
    void move(ControlParameters parameters);

    /**
     * Should get the cost of a move from the application.
     * @param parameters the move to cost.
     * @return the cost.
     */
    double getCost(ControlParameters parameters);

    /**
     * Should get the cost of a move sequence from the application.
     * @param sequence the sequence.
     * @return the cost.
     */
    double getSequenceCost(MoveSequence sequence);

    /**
     * Should get the current cost from the application.
     * @return the cost.
     */
    double getCurrentCost();

    /**
     * Should perform an iteration of search.
     */
    void iteration();

    /**
     * Indicates to the various components that they are finished.
     */
    void finished();

    /**
     * Gets the current control space parameters used.
     * @return the parameters.
     */
    ControlParameters getCurrent();

    /**
     * Should request the application combines a sequence of moves.
     * @param parameters the sequence.
     * @return the combination.
     */
    ControlParameters combineMoves(MoveSequence parameters);

    /**
     * Should request the framework reveals whether it supports moving directly to any control parameter to realise the
     * goal.
     * @return the boolean indicating wheter this is allowed.
     */
    boolean canMoveDirectly();

    /**
     * Should get the current wind value from the application.
     * @return the wind value.
     */
    double getWind();
}

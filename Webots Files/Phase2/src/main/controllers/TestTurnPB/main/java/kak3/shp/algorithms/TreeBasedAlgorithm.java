package main.java.kak3.shp.algorithms;


import main.java.kak3.shp.framework.Framework;

/**
 * Convenience methods for tree based techniques, which will e.g. execute the first move along a sequence, or jump
 * directly to the last if this is allowed.
 */
abstract class TreeBasedAlgorithm implements SearchAlgorithm {

    /**
     * Make the first move in the provided sequence, or jump directly to the last if the application supports this.
     * @param framework the framework instance.
     * @param sequence the sequence of moves.
     */
    void doMove(Framework framework, MoveSequence sequence) {
        doMove(framework, sequence, 0);
    }

    /**
     * Make the *index*th move along the sequence, or jump directly to the last if the application supports this.
     * @param framework the framework instance.
     * @param sequence the sequence of moves.
     * @param index the index to execute, unless jumping to the last is supported.
     */
    void doMove(Framework framework, MoveSequence sequence, int index) {
        if (framework.canMoveDirectly()) {
            framework.move(sequence.getLast());
            framework.finished();
            return;
        }

        if (sequence.getParameters().size() >= 1) {
            framework.move(sequence.getParameters().get(index));
        }
    }
}

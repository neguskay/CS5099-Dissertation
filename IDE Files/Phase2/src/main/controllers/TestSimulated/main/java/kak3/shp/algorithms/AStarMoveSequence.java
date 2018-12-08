package main.java.kak3.shp.algorithms;



import main.java.kak3.shp.framework.ControlParameters;

import java.util.Collections;
import java.util.List;

class AStarMoveSequence extends MoveSequence implements Comparable<AStarMoveSequence> {
    private double score;

    /**
     * Represents a scored move sequence. Can be ordered by their score, with lower score being considered greater,
     * for the purpose of pulling from a priority queue.
     * @param firstParameters the first move in the sequence.
     * @param score the score of the sequence.
     */
    AStarMoveSequence(ControlParameters firstParameters, double score) {
        this(Collections.singletonList(firstParameters), score);
    }

    /**
     * Represents a scored move sequence. Can be ordered by their score, with lower score being considered greater,
     * for the purpose of pulling from a priority queue.
     * @param parameters the list of control parameters to put in this sequence.
     * @param score the score of the sequence.
     */
    AStarMoveSequence(List<ControlParameters> parameters, double score) {
        super(parameters);
        this.score = score;
    }

    double getScore() {
        return score;
    }

    void setScore(double score) {
        this.score = score;
    }

    @Override
    public int compareTo(AStarMoveSequence o) {
        return Double.compare(score, o.score);
    }

    @Override
    public String toString() {
        return parameters.toString() + " {" + score + "}";
    }
}

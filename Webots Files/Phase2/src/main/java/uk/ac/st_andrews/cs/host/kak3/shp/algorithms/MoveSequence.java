package uk.ac.st_andrews.cs.host.kak3.SHP.algorithms;

import uk.ac.st_andrews.cs.host.kak3.SHP.framework.ControlParameters;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

public class MoveSequence {
    final List<ControlParameters> parameters = new ArrayList<>();
    private ControlParameters last;

    /**
     * Represents a sequence of moves to be executed in turn.
     * @param firstParameters the first move in the sequence.
     */
    MoveSequence(ControlParameters firstParameters) {
        this(Collections.singletonList(firstParameters));
    }

    /**
     * Represents a sequence of moves to be executed in turn.
     * @param sequence a sequence to clone the members of.
     */
    MoveSequence(MoveSequence sequence) {
        this.last = sequence.getLast();
        this.parameters.addAll(sequence.getParameters());
    }

    /**
     * Represents a sequence of moves to be executed in turn.
     * @param parameters a list of moves to be executed in turn.
     */
    MoveSequence(List<ControlParameters> parameters) {
        this.parameters.addAll(parameters);
        last = parameters.get(parameters.size() - 1);
    }

    public List<ControlParameters> getParameters() {
        return new ArrayList<>(parameters);
    }

    /**
     * Gets the last move in the sequence.
     * @return the last move in the sequence.
     */
    public ControlParameters getLast() {
        return last;
    }

    /**
     * Add a move to the sequence.
     * @param parameters the move to add.
     */
    void addMove(ControlParameters parameters) {
        this.parameters.add(parameters);
        last = parameters;
    }

    @Override
    public int hashCode() {
        return parameters.hashCode();
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof MoveSequence && ((MoveSequence) other).parameters.equals(parameters);
    }

    @Override
    public String toString() {
        return parameters.toString();
    }

    /**
     * Translates this move sequence into a sequence of absolute co-ordinates, taking the passed control parameters
     * as where they are currently relative to.
     * @param by the control parameters point to translate all points in this sequence by.
     * @return a translated MoveSequence.
     */
    MoveSequence translate(ControlParameters by) {
        List<ControlParameters> translated = parameters.stream().map(parameter -> parameter.translate(by)).collect(Collectors.toList());

        return new MoveSequence(translated);
    }
}

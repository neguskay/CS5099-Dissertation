package uk.ac.st_andrews.cs.host.kak3.SHP.application.neuralnet;

import org.encog.ml.data.MLData;
import org.encog.ml.data.MLDataSet;
import org.encog.ml.data.basic.BasicMLData;
import org.encog.ml.data.basic.BasicMLDataSet;
import org.encog.neural.networks.BasicNetwork;
import org.encog.neural.pattern.ADALINEPattern;
import uk.ac.st_andrews.cs.host.kak3.SHP.framework.ControlParameters;

/**
 * ADALINE neural net digit recognition, modified from
 * https://github.com/encog/encog-java-examples/blob/master/src/main/java/org/encog/examples/neural/adaline/AdalineDigits.java
 */
public class DigitRecognition extends NeuralProblem {
    private final static int CHAR_WIDTH = 5;
    private final static int CHAR_HEIGHT = 7;

    private static final String[][] DIGITS = {
            { " OOO ",
                    "O   O",
                    "O   O",
                    "O   O",
                    "O   O",
                    "O   O",
                    " OOO "  },

            { "  O  ",
                    " OO  ",
                    "O O  ",
                    "  O  ",
                    "  O  ",
                    "  O  ",
                    "  O  "  },

            { " OOO ",
                    "O   O",
                    "    O",
                    "   O ",
                    "  O  ",
                    " O   ",
                    "OOOOO"  },

            { " OOO ",
                    "O   O",
                    "    O",
                    " OOO ",
                    "    O",
                    "O   O",
                    " OOO "  },

            { "   O ",
                    "  OO ",
                    " O O ",
                    "O  O ",
                    "OOOOO",
                    "   O ",
                    "   O "  },

            { "OOOOO",
                    "O    ",
                    "O    ",
                    "OOOO ",
                    "    O",
                    "O   O",
                    " OOO "  },

            { " OOO ",
                    "O   O",
                    "O    ",
                    "OOOO ",
                    "O   O",
                    "O   O",
                    " OOO "  },

            { "OOOOO",
                    "    O",
                    "    O",
                    "   O ",
                    "  O  ",
                    " O   ",
                    "O    "  },

            { " OOO ",
                    "O   O",
                    "O   O",
                    " OOO ",
                    "O   O",
                    "O   O",
                    " OOO "  },

            { " OOO ",
                    "O   O",
                    "O   O",
                    " OOOO",
                    "    O",
                    "O   O",
                    " OOO "  } };

    private static MLDataSet generateTraining()
    {
        MLDataSet result = new BasicMLDataSet();
        for(int i=0;i<DIGITS.length;i++)
        {
            BasicMLData ideal = new BasicMLData(DIGITS.length);

            // setup input
            MLData input = image2data(DIGITS[i]);

            // setup ideal
            for(int j=0;j<DIGITS.length;j++)
            {
                if( j==i )
                    ideal.setData(j,1);
                else
                    ideal.setData(j,-1);
            }

            // add training element
            result.add(input,ideal);
        }
        return result;
    }

    private static MLData image2data(String[] image)
    {
        MLData result = new BasicMLData(CHAR_WIDTH*CHAR_HEIGHT);

        for(int row = 0; row<CHAR_HEIGHT; row++)
        {
            for(int col = 0; col<CHAR_WIDTH; col++)
            {
                int index = (row*CHAR_WIDTH) + col;
                char ch = image[row].charAt(col);
                result.setData(index,ch=='O'?1:-1 );
            }
        }

        return result;
    }

    public DigitRecognition() {
        int inputNeurons = CHAR_WIDTH * CHAR_HEIGHT;
        int outputNeurons = DIGITS.length;

        ADALINEPattern pattern = new ADALINEPattern();
        pattern.setInputNeurons(inputNeurons);
        pattern.setOutputNeurons(outputNeurons);
        network = (BasicNetwork)pattern.generate();

        trainingSet = generateTraining();
    }

    @Override
    ControlParameters getInitialState() {
        return null;
    }

    public void finished()
    {
        // test it
        for (String[] DIGIT : DIGITS) {
            int output = network.winner(image2data(DIGIT));

            for (int j = 0; j < CHAR_HEIGHT; j++) {
                if (j == CHAR_HEIGHT - 1)
                    System.out.println(DIGIT[j] + " -> " + output);
                else
                    System.out.println(DIGIT[j]);

            }

            System.out.println();
        }
    }
}

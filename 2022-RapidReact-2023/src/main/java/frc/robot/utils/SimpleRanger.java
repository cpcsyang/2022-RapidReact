package frc.robot.utils;

import java.util.HashMap;

public class SimpleRanger implements Ranger {

    private final double DISTANCE_0 = 0;
    private final double DISTANCE_1 = 1.398;
    private final double DISTANCE_2 = 2.066; // no
    private final double DISTANCE_3 = 2.750;
    private final double DISTANCE_4 = 3.75;
    private final double DISTANCE_5 = 4.099; //
    private final double DISTANCE_6 = 4.741; //
    private final double DISTANCE_7 = 5;

    private final double ANGLE_0 = 85;
    private final double ANGLE_1 = 80;
    private final double ANGLE_2 = 75; //
    private final double ANGLE_3 = 71;
    private final double ANGLE_4 = 67; 
    private final double ANGLE_5 = 67; //
    private final double ANGLE_6 = 67; //
    private final double ANGLE_7 = 67;


    private final double SPEED_0 = 1200; 
    private final double SPEED_1 = 1300; 
    private final double SPEED_2 = 1350; //
    private final double SPEED_3 = 1700;
    private final double SPEED_4 = 1875;
    private final double SPEED_5 = 1950; //
    private final double SPEED_6 = 2200; //
    private final double SPEED_7 = 2400;

    private final int DISTANCE_COLUMN_INDEX = 0;
    private final int ANGLE_COLUMN_INDEX = 1;
    private final int SPEED_COLUMN_INDEX = 2;

    private double[][] KNOWN_DATA = new double[][] {
        {DISTANCE_0, ANGLE_0, SPEED_0},
        {DISTANCE_1, ANGLE_1, SPEED_1},
        {DISTANCE_2, ANGLE_2, SPEED_2},
        {DISTANCE_3, ANGLE_3, SPEED_3},
        {DISTANCE_4, ANGLE_4, SPEED_4},
        {DISTANCE_5, ANGLE_5, SPEED_5},
        {DISTANCE_6, ANGLE_6, SPEED_6},
        {DISTANCE_7, ANGLE_7, SPEED_7},
    };

    // We keep a list of a few fixed firing solutions that will work even if the limelight is not.
    private HashMap<String,FiringSolution> fixedSolutions = new HashMap<String, FiringSolution>();
    private FiringSolution hubShot = new FiringSolution((int) Math.round(SPEED_0), ANGLE_0);
    private FiringSolution hangarShot = new FiringSolution(0,0);  //TODO: Collect actual values

    private enum InterpolationMode {
        ANGLE,
        SPEED
    }

    public SimpleRanger() {
        fixedSolutions.put("hub", hubShot);
        fixedSolutions.put("hangar", hangarShot);
    }

    public FiringSolution getFiringSolution(double range) {

        // beyond endpoints
        if (range <= DISTANCE_0) {
            return new FiringSolution((int) Math.round(SPEED_0), ANGLE_0);
        }
        if (range >= DISTANCE_7) {
            return new FiringSolution((int) Math.round(SPEED_7), ANGLE_7); 
        }

        // find the two data points to be used
        int distanceHigherBoundIndex = 0;
        for (int i = 0; i < KNOWN_DATA.length; i ++) {
            double distanceReference = KNOWN_DATA[i][DISTANCE_COLUMN_INDEX];
            if (range < distanceReference) {
                distanceHigherBoundIndex = i;
                break;
            }
        }

        int speed = (int) Math.round(linearInterpolation(range, distanceHigherBoundIndex, InterpolationMode.SPEED));
        double angle = linearInterpolation(range, distanceHigherBoundIndex, InterpolationMode.ANGLE);

        return new FiringSolution(speed, angle);
    }

    public FiringSolution getFiringSolution(String name)
    {
        return fixedSolutions.get(name);
    }
    
    private double linearInterpolation(double range, int distanceHigherBoundIndex, InterpolationMode mode) {
        
        int modeIndex;
        if (mode.equals(InterpolationMode.ANGLE)) {
            modeIndex = ANGLE_COLUMN_INDEX;
        }
        else {
            modeIndex = SPEED_COLUMN_INDEX;
        }

        double x1 = KNOWN_DATA[distanceHigherBoundIndex - 1][DISTANCE_COLUMN_INDEX];
        double x2 = KNOWN_DATA[distanceHigherBoundIndex][DISTANCE_COLUMN_INDEX];
        double y1 = KNOWN_DATA[distanceHigherBoundIndex  - 1][modeIndex];
        double y2 = KNOWN_DATA[distanceHigherBoundIndex][modeIndex];

        double x = range;

        double m = (y2 - y1) / (x2 - x1);
        double y = m * x + y1 - m * x1;
        
        return y;
    }
}


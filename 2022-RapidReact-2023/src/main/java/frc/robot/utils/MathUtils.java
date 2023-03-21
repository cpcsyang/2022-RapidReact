package frc.robot.utils;

public class MathUtils {

    /**
     * Test if a value is with a specified delta of a target value.
     * 
     * @param value  - Value to be tested against a given range
     * @param target - Target value being tested against
     * @param delta  - Delta from target that value must be within
     * 
     * @return - True if value is within the range target +/- delta (inclusive)
     */
    public static boolean WithinDelta(double value, double target, double delta) {

        return (value >= target - delta) && (value <= target + delta);
    }

    
    /**
     * Test if a value is within a specific range.
     * 
     * @param value - Value to be tested against specified range
     * @param min - Lower boundry of test range.
     * @param max - Upper boundry of test range
     * 
     * @return - True if value is within the range specified by min and max (inclusive)
     */
    public static boolean WithinRange(double value, double min, double max) {

        return (value >= min) && (value <= max);
    }

    /**
     * Clip an input value so it it within a specified range
     * 
     * @param value - Value to be clipped to target range
     * @param min - Lower boundry of test range.
     * @param max - Upper boundry of test range
     * 
     * @return - If value between min and max (inclusive) returns value otherwise returns min or max.
     */
    public static double ClipToRange(double value, double min, double max) {

        if (value < min) {
            return min;
        } else if (value > max) {
            return max;
        } else {
            return value;
        }
    }
}

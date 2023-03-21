package frc.robot.utils;

import java.util.function.Function;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class XboxControllerHelper {

    private XboxController controller;
    private double deadband = 0.1;
    private Function<Double, Double> scaler = this::squared;
    private double rumbleIntensity = 1.0;

    public XboxControllerHelper(XboxController controller) {
        this.controller = controller;
    }

    /**
     * Set the function used to scale joystick values, defaults to squared.
     * 
     * @param scaler
     */
    public void setScaler(Function<Double, Double> scaler) {
        this.scaler = scaler;
    }

    /**
     * Set the width of the dead band applied to joystick axis values.
     * 
     * @param deadband
     */
    public void setDeadband(double deadband) {
        this.deadband = deadband;
    }

    /**
     * Set the default intensity of the controller rumble, defaults to 1.0 (full
     * intensity)
     * 
     * @param intensity
     */
    public void setRumbleIntensity(double intensity) {
        this.rumbleIntensity = intensity;
    }

    /**
     * Scale the value read from a joystick axis using the current scaler method and
     * apply the deadband.
     * 
     * @param value
     * @return
     */
    public double scaleAxis(double value) {
        value = deadband(value);
        value = Math.copySign(scaler.apply(value), value);

        return value;
    }

    /**
     * Turn the rumble feature of the controller on or off.
     * @param rumble - If true rumble is started, rumble is stopped if false
     */
    public void rumble(boolean rumble) {
        if (rumble) {
            controller.setRumble(RumbleType.kLeftRumble, rumbleIntensity);
            controller.setRumble(RumbleType.kRightRumble, rumbleIntensity);
        }
        else {
            controller.setRumble(RumbleType.kLeftRumble, 0);
            controller.setRumble(RumbleType.kRightRumble, 0);
        }
    }

    /**
     * Map values inside the deadband to zero and scale values outside of the deadband to distribute from 0.0 to 1.0.
     * @param value
     * @return
     */
    private double deadband(double value) {
        double result = 0.0;

        if (Math.abs(value) > deadband) {
            result = value > 0 ? value - deadband : value + deadband;
            result /= (1.0 - deadband);
        }

        return result;
    }

    /**
     * equal sensitivity across the entire stick range
     * 
     * @param value
     * @return
     */
    public double linear(double value) {
        return value;
    }

    /**
     * Square: less sensitive near the center
     * 
     * @param value
     * @return
     */
    public double squared(double value) {
        return value * value;
    }

    /**
     * Cubed: much more sensitive near the outer range of the stick
     * 
     * @param value
     * @return
     */
    public double cubed(double value) {
        return value * value * value;
    }

    /**
     * SquareRoot: more sensitive near the center of the stick
     * 
     * @param value
     * @return
     */
    public double squareRoot(double value) {
        return Math.sqrt(value);
    }

    /**
     * CubicRoot: much more sensitive near the outer range of the stick
     * 
     * @param value
     * @return
     */
    public double cubicRoot(double value) {
        return Math.cbrt(value);
    }
}

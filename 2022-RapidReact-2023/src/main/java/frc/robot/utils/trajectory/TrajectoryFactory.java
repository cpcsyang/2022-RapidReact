package frc.robot.utils.trajectory;

import org.frcteam2910.common.control.SplinePathBuilder;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.control.TrajectoryConstraint;

import java.util.HashMap;

import org.frcteam2910.common.control.CentripetalAccelerationConstraint;
import org.frcteam2910.common.control.FeedforwardConstraint;
import org.frcteam2910.common.control.MaxAccelerationConstraint;
import org.frcteam2910.common.control.MaxVelocityConstraint;
import org.frcteam2910.common.control.SimplePathBuilder;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import frc.robot.commands.FollowerCommand;

public class TrajectoryFactory {

    // smaller sample distance, more precision
    private static double sampleDistance = 0.00254;

    // constraints
    public static TrajectoryConstraint[] constraints = {
            new MaxAccelerationConstraint(1.5),
            new FeedforwardConstraint(10, //10
                    FollowerCommand.FEEDFORWARD_CONSTANTS.getVelocityConstant(),
                    FollowerCommand.FEEDFORWARD_CONSTANTS.getAccelerationConstant()),
            new CentripetalAccelerationConstraint(4) //4
    };

    // unused
    public static TrajectoryConstraint[] faster = {
        new MaxAccelerationConstraint(1.5),
        new FeedforwardConstraint(10, //10
                FollowerCommand.FEEDFORWARD_CONSTANTS.getVelocityConstant(),
                FollowerCommand.FEEDFORWARD_CONSTANTS.getAccelerationConstant()),
        new CentripetalAccelerationConstraint(4) //4
};

    public static TrajectoryConstraint[] testConstraints = {
        new MaxAccelerationConstraint(1.0),
        new MaxVelocityConstraint(1),
        new CentripetalAccelerationConstraint(3)
};
    // for tuning feedforward & pid & constraints
    // 1. tune kStatic : set a reasonable kV, kStatic = voltage at which the robot
    // starts up (Glass)
    // 2. tune kV : better kV = error * last kV + last kV
    // 3. tune kA : increase until robot twitches, then consider decrease kV or kA
    // 4. tune P: increase until consistantly hitting target within a reasonable
    // tolerance
    // 5. tune AConstraint : increase until robot overshoots, then decrease
    // 6. tune FFConstraint : adjust between 6 - 10
    // 7. tune AcConstraint : robot goes through the curve at the reasonable speed
    // while hitting target

    private HashMap<String, Trajectory> trajectories = new HashMap<String, Trajectory>();

    private Trajectory tuneLine = new Trajectory(
        new SimplePathBuilder(new Vector2(0, 0), Rotation2.ZERO)
            .lineTo(new Vector2(5, 0), Rotation2.fromDegrees(0))
            .build(),
        constraints,
        sampleDistance);

    private Trajectory tuneCurve = new Trajectory(
        new SplinePathBuilder(new Vector2(0, 0), Rotation2.ZERO, Rotation2.ZERO)
            .hermite(new Vector2(2, 2), Rotation2.fromDegrees(180), Rotation2.ZERO)
            .build(),
        constraints,
        sampleDistance);
    
    /*
    private Trajectory wrong_start_ball2 = new Trajectory(
        new SplinePathBuilder(new Vector2(-0.5, -2), new Rotation2(-.5, -2, true), Rotation2.fromDegrees(-90))
            .hermite(new Vector2(-.58, -2.85), new Rotation2(0, 1, true), Rotation2.fromDegrees(-90))
            // .hermite(new Vector2(-2, -2), new Rotation2(-3.5, -2.2, true),
            // Rotation2.fromDegrees(163.8720703125))
            // .hermite(new Vector2(-3.38, -1.56), new Rotation2(-3.20, -1.14, true),
            // Rotation2.fromDegrees(-136))
            .build(),
        constraints,
        sampleDistance);
    */

    private Trajectory back = new Trajectory(
        new SimplePathBuilder(new Vector2(-0.5, -4), Rotation2.fromDegrees(-90))
            .lineTo(new Vector2(-0.5, -2), Rotation2.fromDegrees(-90))
            .build(),
            constraints,
            sampleDistance);

            private Trajectory forward = new Trajectory(
                new SimplePathBuilder(new Vector2(-0.5, -2), Rotation2.fromDegrees(-90))
                    .lineTo(new Vector2(-.58, -4), Rotation2.fromDegrees(-90))
                    .build(),
                    constraints,
                    sampleDistance);
    
    private Trajectory start_ball2 = new Trajectory(
        new SimplePathBuilder(new Vector2(-0.5, -2), Rotation2.fromDegrees(-90))
            .lineTo(new Vector2(-.58, -2.95), Rotation2.fromDegrees(-90))
            .build(),
            constraints,
            sampleDistance);

    private Trajectory ball2_ball3 = new Trajectory(
        new SplinePathBuilder(new Vector2(-0.58, -2.95), new Rotation2(0, 1, true), Rotation2.fromDegrees(-90))
            .hermite(new Vector2(-3.38, -1.76), new Rotation2(-3.20, -1.14, true), Rotation2.fromDegrees(-154))
            .build(),
            constraints,
            sampleDistance);

    /*
     * public static Trajectory ball2_ball3 = new Trajectory(
     * new SplinePathBuilder(new Vector2(-.6, -3.5), new Rotation2(-2, -2, true),
     * Rotation2.fromDegrees(-90))
     * .hermite(new Vector2(-2, -2), new Rotation2(-3.5, -2.2, true),
     * Rotation2.fromDegrees(163.8720703125))
     * .hermite(new Vector2(-3.2, -2.2), new Rotation2(-2, -2, true),
     * Rotation2.fromDegrees(-114.96093749999997))
     * .build(),
     * slow,
     * sampleDistance);
     */

    private Trajectory ball3_station = new Trajectory(
        new SplinePathBuilder(new Vector2(-3.38, -1.76), new Rotation2(-7.2, -2.2, true), Rotation2.fromDegrees(-114.96093749999997))
            .hermite(new Vector2(-6.92, -2.31), new Rotation2(-7.2, -2.2, true), Rotation2.fromDegrees(-149)) // 7.2, 2.2
            // .hermite(new Vector2(-3.38, -1.56), new Rotation2(1, 1, true),
            // Rotation2.fromDegrees(-155))
            .build(),
        constraints,
        sampleDistance);
        
    private Trajectory station_shoot = new Trajectory(
        new SplinePathBuilder(new Vector2(-6.92, -2.31), Rotation2.ZERO, Rotation2.fromDegrees(-149))
            .hermite(new Vector2(-3.38, -1.76), Rotation2.ZERO, Rotation2.fromDegrees(-154))
            .build(),
        constraints,
        sampleDistance);

    private Trajectory ball3_station_line = new Trajectory(
        new SimplePathBuilder(new Vector2(-3.38, -1.76), Rotation2.fromDegrees(-114.96093749999997))
            .lineTo(new Vector2(-6.92, -2.31), Rotation2.fromDegrees(-149))
            .build(),
        constraints,
        sampleDistance);
            
    private Trajectory station_shoot_line = new Trajectory(
            new SimplePathBuilder(new Vector2(-6.92, -2.31), Rotation2.fromDegrees(-149))
                .lineTo(new Vector2(-4, -2.08), Rotation2.fromDegrees(-154))
                .build(),
            constraints,
            sampleDistance);
   
    public TrajectoryFactory() {
        trajectories.put("tune line", tuneLine);
        trajectories.put("tune curve", tuneCurve);
        trajectories.put("start to ball2", start_ball2);
        trajectories.put("ball2 to ball3", ball2_ball3);
        trajectories.put("ball3 to station", ball3_station);
        trajectories.put("station to shoot", station_shoot);
        trajectories.put("ball3 to station line", ball3_station_line);
        trajectories.put("station to shoot line", station_shoot_line);
        trajectories.put("back", back);
        trajectories.put("forward", forward);
    }

    public Trajectory get(String name) {
        return trajectories.get(name);
    }
}

package frc.robot.utils;

import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Conversions {
    public static RigidTransform2 poseToRigidTransform(Pose2d pose){
        return new RigidTransform2(new Vector2(pose.getX(), pose.getY()),
                Rotation2.fromDegrees(pose.getRotation().getDegrees()));
    }

    public static Vector2 translationToVector2(Translation2d translation){
        return new Vector2(translation.getX(), translation.getY());
    }

    public static Vector2 chassisSpeedsToVector(ChassisSpeeds speeds){
        return new Vector2(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }
    
}

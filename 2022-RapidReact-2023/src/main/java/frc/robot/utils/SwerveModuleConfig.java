package frc.robot.utils;

public class SwerveModuleConfig {
    public final int driveMotorCanId;
    public final int steerMotorCanId;
    public final int encoderCanId;
    public final double encoderOffset;

    public SwerveModuleConfig(int driveMotorCanId, int steerMotorCanId, int encoderCanId, double encoderOffset) {

        this.driveMotorCanId = driveMotorCanId;
        this.steerMotorCanId = steerMotorCanId;
        this.encoderCanId = encoderCanId;
        this.encoderOffset = encoderOffset;
    }
}

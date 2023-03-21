package frc.robot.utils;

public class SwerveDriveConfig {
    public final SwerveModuleConfig frontLeft;
    public final SwerveModuleConfig frontRight;
    public final SwerveModuleConfig backLeft;
    public final SwerveModuleConfig backRight;
    public final double trackWidth;
    public final double wheelbase;
    public final double wheelDiameter;

    public SwerveDriveConfig(SwerveModuleConfig fl, SwerveModuleConfig fr, SwerveModuleConfig bl, SwerveModuleConfig br, double trackWidth, double wheelbase, double wheelDiameter) {
        frontLeft = fl;
        frontRight = fr;
        backLeft = bl;
        backRight = br;
        this.trackWidth = trackWidth;
        this.wheelbase = wheelbase;
        this.wheelDiameter = wheelDiameter;
    }
}

package frc.robot.utils;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SwervePath {

    private List<Command> m_startingCommands;
    private List<Command> m_endingCommands;
    private List<Path> m_paths;

    private List<Command> m_runCommands;

    public DrivetrainSubsystem m_drivetrainSubsystem;

    double kpX = 0.00000000000001;  //.01 for both X and Y
    double kpY = 0.00000000000001;
    double kiX = 0.0;
    double kiY = 0.0;
    double kdX = 0.0;
    double kdY = 0.0;
    PIDController xController = new PIDController(kpX, kiX, kdX); 
    PIDController yController = new PIDController(kpY, kiY, kdY); 

    double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
    double kMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
    double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;

    TrapezoidProfile.Constraints thetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond,
        kMaxAngularAccelerationRadiansPerSecondSquared);

    double kpTheta = 0.0004;   // .04
    double kiTheta = 0.0;
    double kdTheta = 0.0;
    ProfiledPIDController thetaController = new ProfiledPIDController(kpTheta, kiTheta, kdTheta, thetaControllerConstraints); 

    public class Path{
        TrajectoryConfig config;
        List<Translation2d> m_points;

        public Path(List<Translation2d> Points, double Acceleration, double MaxSpeed){
            config = new TrajectoryConfig(MaxSpeed, Acceleration);
            m_points = Points;
        }

        public Trajectory getTrajectory(){
            return TrajectoryGenerator.generateTrajectory(new Pose2d(0,0, new Rotation2d(0)),m_points,new Pose2d(m_points.get(-1), new Rotation2d(0)), config);
        }

        public Command getPathCommand() {
            return new SwerveControllerCommand(
                getTrajectory(),
                m_drivetrainSubsystem::getPose,
                m_drivetrainSubsystem.getKinematics(),
                xController,
                yController,
                thetaController,
                m_drivetrainSubsystem::setModuleStates,
                m_drivetrainSubsystem);
        }
    }

    public SwervePath(List<Path> Paths){
        
    }


    Command[] commands;
    /**
     * 
     * @param Paths List of the paths that you want to make
     * @param StartCommands Theys commands will run at the *start* of the Path
     * @param EndCommands These Commands will run after the *end* of the Path
    */
    public SwervePath(DrivetrainSubsystem subsystem,List<Path> Paths, List<Command> StartCommands, List<Command> EndCommands){
        m_drivetrainSubsystem = subsystem;
        
        m_paths = Paths;
        m_startingCommands = StartCommands;
        m_endingCommands = EndCommands;
        
        Generate();
    }

    private void Generate(){
        
        

        for (Path path : m_paths) {
            
            //Adding Start Command
            for(Command command : m_startingCommands) {
                m_runCommands.add(command);
            }
            
            //Adding Path Command
            m_runCommands.add(path.getPathCommand());
            
            //Adding End Commands
            for (Command command : m_endingCommands) {
                m_runCommands.add(command);
            }
        }

        //Pushes all the commands into an Array
        commands = new Command[m_runCommands.size()];
        m_runCommands.toArray(commands);
    }


    
    /**
     * 
     * @return Returns all of the paths and starting and ending commands in a SequentialCommandGroup
     */
    public Command Run(){
        return new SequentialCommandGroup(commands);
    }
    
}

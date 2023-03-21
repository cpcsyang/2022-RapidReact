package frc.robot.utils.trajectory;

import java.util.ArrayList;
import java.util.List;

import org.frcteam2910.common.control.MaxAccelerationConstraint;
import org.frcteam2910.common.control.MaxVelocityConstraint;
import org.frcteam2910.common.control.Path;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.control.TrajectoryConstraint;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.FollowerCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoPath{
	

	private List<Command> m_parallelCommands = new ArrayList<Command>();
	private Path m_path;
	private TrajectoryConstraint[] constraints;

	private boolean m_isParallelCommand;


	public AutoPath(Path path, double Acceleration, double MaxSpeed){
		m_isParallelCommand = false;
		constraints = new TrajectoryConstraint[] {
			new MaxAccelerationConstraint(Acceleration),
			new MaxVelocityConstraint(MaxSpeed)
		};
	}
	
	public AutoPath (Path path, double Acceleration, double MaxSpeed, List<Command> ParallelCommands){
		m_isParallelCommand = true;
		m_parallelCommands = ParallelCommands;
		constraints = new TrajectoryConstraint[] {
			new MaxAccelerationConstraint(Acceleration),
			new MaxVelocityConstraint(MaxSpeed)
		};
	}

	private Trajectory getTrajectory(){
		return new Trajectory(m_path,constraints,Units.inchesToMeters(0.1));
	}

	public boolean isParallelCommand(){
		return m_isParallelCommand;
	}

	public Command getPathCommand(DrivetrainSubsystem m_drivetrainSubsystem) {
		return new FollowerCommand(m_drivetrainSubsystem,getTrajectory());
	}

	public Command getParllelPathCommand(DrivetrainSubsystem m_drivetrainSubsystem){
		m_parallelCommands.add(new FollowerCommand(m_drivetrainSubsystem, getTrajectory()));
		Command command_array[] = new Command[m_parallelCommands.size()];
		m_parallelCommands.toArray(command_array);
		return new ParallelCommandGroup(command_array);	
	}
}

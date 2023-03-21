package frc.robot.utils.trajectory;

import java.util.ArrayList;
import java.util.List;
import org.frcteam2910.common.control.Path;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;


public class AutoDrive {
	DrivetrainSubsystem m_subsystem;
	double m_acceleration = 1;
	double m_maxSpeed = 1;

	public List<AutoPath> paths = new ArrayList<AutoPath>();
	public SwervePath path;
	public List<Command> StartingCommands = new ArrayList<Command>();
	public List<Command> EndingCommands = new ArrayList<Command>();
	

	public void addPath(Path path){
		paths.add(new AutoPath(path, m_acceleration, m_maxSpeed));
	}

	public void addPath(Path Path,double Acceleration, double MaxSpeed,  List<Command> commands){
		paths.add(new AutoPath(Path, Acceleration, MaxSpeed, commands));
	}

	public AutoDrive(DrivetrainSubsystem subsystem, List<Command> StartingCommands, List<Command> EndingCommands) {
		m_subsystem = subsystem;
		this.StartingCommands = StartingCommands;
		this.EndingCommands = EndingCommands;

	}

	public SwervePath getPath(){
		path = new SwervePath(m_subsystem, paths, StartingCommands, EndingCommands);
		return path;
	}

	
}
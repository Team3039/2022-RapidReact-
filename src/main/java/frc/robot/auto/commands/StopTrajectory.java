package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class StopTrajectory extends CommandBase {

    private static final Drive s_Swerve = Drive.getInstance();

    @Override
    public void initialize() {
        System.out.println("Stop Trajectory Started");
        s_Swerve.setModuleStates(Constants.Swerve.ZERO_STATES);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Stop Trajectory Finished");
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
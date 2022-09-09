// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;

public class SpinShooterNoTrack extends CommandBase {

  double setpoint;
  /** Creates a new SpinShooterNoTrack. */
  public SpinShooterNoTrack(double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
   // addRequirements(RobotContainer.shooter);
    this.setpoint = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Shooter.setPointShooter = setpoint;
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.shooter.setState(ShooterState.SPIN_UP);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter.setState(ShooterState.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

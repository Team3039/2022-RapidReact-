// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Indexer.IndexerState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.Turret.TurretState;

public class SetSubsystemsClimbMode extends CommandBase {
  /** Creates a new SetSubsystemsClimbMode. */
  public SetSubsystemsClimbMode() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.intake.setState(IntakeState.CLIMBING);
    RobotContainer.indexer.setState(IndexerState.CLIMBING);
    RobotContainer.turret.setState(TurretState.CLIMBING);
    RobotContainer.shooter.setState(ShooterState.CLIMBING);
    // RobotContainer.climber.isClimbing = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intake.setState(IntakeState.IDLE);
    RobotContainer.indexer.setState(IndexerState.IDLE);
    RobotContainer.turret.setState(TurretState.DRIVE);
    RobotContainer.shooter.setState(ShooterState.IDLE);
    // RobotContainer.climber.isClimbing = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

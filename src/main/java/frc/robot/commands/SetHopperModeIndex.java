// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Hopper.HopperControlMode;

public class SetHopperModeIndex extends CommandBase {
  /** Creates a new SetIntakingMode. */
  public SetHopperModeIndex() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //RobotContainer.intake.actuateIntake(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.hopper.getWrongBallDetection()) {
      RobotContainer.hopper.setHopperControlMode(HopperControlMode.REJECTING);
      RobotContainer.intake.setIntakeSpeed(-0.3);
      }
    else {
      RobotContainer.hopper.setHopperControlMode(HopperControlMode.INDEXING);
      RobotContainer.intake.setIntakeSpeed(.3);
    }   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   // RobotContainer.intake.runIntake(0, false);
   RobotContainer.hopper.setHopperControlMode(HopperControlMode.IDLE);
   RobotContainer.intake.setIntakeSpeed(0);
   RobotContainer.hopper.runFeeder(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

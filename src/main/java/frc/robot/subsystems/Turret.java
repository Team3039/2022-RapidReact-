// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  public Turret() {

   public void setCamMode(boolean is) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
   }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

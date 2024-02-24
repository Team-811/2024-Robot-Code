// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.CANVenom;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Util.ISubsystem;

public class Climber extends SubsystemBase implements ISubsystem{

  CANVenom LeftHook = new CANVenom(OperatorConstants.leftHookID);
  CANVenom RightHook = new CANVenom(OperatorConstants.rightHookID);

  /** Creates a new Climber. */
  public Climber() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void updateSmartdashboard() {

  }
}

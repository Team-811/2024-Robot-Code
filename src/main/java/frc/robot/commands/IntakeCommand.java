// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {
  private boolean isTurningToShooter = false;
  private boolean isTurningToFloor = false; 
  private Intake myIntake;
  private BooleanSupplier shouldSpinWheels;
  private BooleanSupplier shouldTurnToShooter;
  private BooleanSupplier shouldTurnToFloor;
    private BooleanSupplier shouldSpinBackwards;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(Intake intake,BooleanSupplier spinWheels,BooleanSupplier turnToShooter,BooleanSupplier turnToFloor,BooleanSupplier spinBackwards) {
    myIntake = intake;
    shouldSpinWheels = spinWheels;
    shouldTurnToShooter=turnToShooter;
    shouldTurnToFloor=turnToFloor;
    shouldSpinBackwards = spinBackwards;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shouldSpinWheels.getAsBoolean()== true) {
      myIntake.spinIntake(-OperatorConstants.intakeWheelsSpeed);
    }
    else {
      if(shouldSpinBackwards.getAsBoolean()==true){
        myIntake.spinIntakeIgnoreLimit(OperatorConstants.intakeWheelsSpeed);
      }else{
        myIntake.stopIntakeWheels();
    }
    }
    if (isTurningToShooter == true) {
      isTurningToShooter =! myIntake.turnToPoint(OperatorConstants.intakeShooterSetPoint);
    }
    if (isTurningToFloor == true) {
      isTurningToFloor =! myIntake.turnToPoint(OperatorConstants.intakeFloorSetPoint);
    }
    if (isTurningToShooter == false && isTurningToFloor == false) {

      if (shouldTurnToShooter.getAsBoolean()== true) {
       isTurningToShooter = true; 
      }
      else if (shouldTurnToFloor.getAsBoolean()== true) {
        isTurningToFloor = true;
      }
    }
  } 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

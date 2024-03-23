// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Util.ISubsystem;

public class Shoooter extends SubsystemBase implements ISubsystem {

  private SparkPIDController leftPID;
  private SparkPIDController rightPID;

  // private RelativeEncoder leftEncoder; 
  // private RelativeEncoder rightEncoder; 


  public CANSparkMax leftWheels = new CANSparkMax(OperatorConstants.leftShooterWheelID,MotorType.kBrushless);
  public CANSparkMax rightWheels = new CANSparkMax(OperatorConstants.rightShooterWheelID,MotorType.kBrushless);



  /** Creates a new Shoooter. */
  public Shoooter() {
    rightWheels.setInverted(false);
    leftWheels.setInverted(true);
    // leftEncoder = leftWheels.getEncoder();
    // rightEncoder = rightWheels.getEncoder();
    leftPID = leftWheels.getPIDController();
    rightPID = rightWheels.getPIDController();
    rightWheels.setIdleMode(IdleMode.kBrake);
    leftWheels.setIdleMode(IdleMode.kBrake);
  }

 public void spinShooter(double sped) {
    if(sped == OperatorConstants.shootSpeed){
      leftWheels.set(1);
      rightWheels.set(1);
      return;
    }

    leftPID.setP(OperatorConstants.shooterkP);
    leftPID.setI(OperatorConstants.shooterkI);
    leftPID.setD(OperatorConstants.shooterkD);
    leftPID.setIZone(OperatorConstants.shooterkIz);
    leftPID.setFF(OperatorConstants.shooterkFF);
    rightPID.setP(OperatorConstants.shooterkP);
    rightPID.setI(OperatorConstants.shooterkI);
    rightPID.setD(OperatorConstants.shooterkD);
    rightPID.setIZone(OperatorConstants.shooterkIz);
    rightPID.setFF(OperatorConstants.shooterkFF);
    leftPID.setOutputRange(-1, 1);
    rightPID.setOutputRange(-1, 1);
    leftPID.setReference(sped, ControlType.kVelocity);
    rightPID.setReference(sped, ControlType.kVelocity);
    // System.out.println(leftEncoder.getVelocity());
    // System.out.println();
    // leftWheels.set(1);
    // rightWheels.set(1);
  }

  public void stopShooter(){
    leftWheels.set(0);
    rightWheels.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void updateSmartdashboard() {
    
  }
}

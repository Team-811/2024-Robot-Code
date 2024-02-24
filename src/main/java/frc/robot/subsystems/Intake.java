// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkAnalogSensor.Mode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Util.ISubsystem;

public class Intake extends SubsystemBase implements ISubsystem{
  DigitalInput noteLimitSwitch = new DigitalInput(OperatorConstants.noteLimitSwitchID);
  DigitalInput pivotLimitSwitch = new DigitalInput(OperatorConstants.pivotLimitSwitchID);

  private SparkPIDController wheelsPID;
  private SparkPIDController pivotPID;

  // private RelativeEncoder wheelsEncoder; 
  private RelativeEncoder pivotEncoder;
  

  // private CANcoder absoluteEncoder = new CANcoder(19);
  // private SparkAnalogSensor  absoluteEncoder;

  public CANSparkMax Wheels = new CANSparkMax(OperatorConstants.intakeMotorWheelsID,MotorType.kBrushless);
  public CANSparkMax pivotMotor = new CANSparkMax(OperatorConstants.pivotMotorID,MotorType.kBrushless);


  /** Creates a new Intake. */
  public Intake() {
    Wheels.setInverted(false);
    // wheelsEncoder = Wheels.getEncoder();
    wheelsPID = Wheels.getPIDController();
    Wheels.setIdleMode(IdleMode.kBrake);
    
    
    pivotMotor.setInverted(false);
    pivotPID = pivotMotor.getPIDController();
    pivotEncoder = pivotMotor.getEncoder();
    pivotMotor.setIdleMode(IdleMode.kBrake);
    // absoluteEncoder = pivotMotor.getAnalog(Mode.kAbsolute);
    pivotPID.setFeedbackDevice(pivotEncoder);
    syncEncoder();
    pivotMotor.setCANTimeout(0);
  }

  public boolean turnToPoint(double setPoint) {
     pivotPID.setP(OperatorConstants.intakePivotkP);
    pivotPID.setI(OperatorConstants.intakePivotkI);
    pivotPID.setD(OperatorConstants.intakePivotkD);
    pivotPID.setIZone(OperatorConstants.intakePivotkIz);
    pivotPID.setFF(OperatorConstants.intakePivotkFF);
    pivotPID.setOutputRange(-1, 1);
    pivotPID.setReference(setPoint, ControlType.kPosition);
    SmartDashboard.putNumber("Pivot Setpoint", setPoint);
    if (Math.abs(setPoint - pivotEncoder.getPosition())<1) 
      return true;
    // if(setPoint<pivotEncoder.getPosition())
    //   pivotMotor.set(-0.08);
    // else
    //   pivotMotor.set(0.08);
    return false;
  }

  public void stopPivot(){
    pivotMotor.set(0);
  }

 public void spinIntake(double sped) {
    wheelsPID.setP(OperatorConstants.intakeWheelskP);
    wheelsPID.setI(OperatorConstants.intakeWheelskI);
    wheelsPID.setD(OperatorConstants.intakeWheelskD);
    wheelsPID.setIZone(OperatorConstants.intakeWheelskIz);
    wheelsPID.setFF(OperatorConstants.intakeWheelskFF);
    wheelsPID.setOutputRange(-1, 1);
    if (noteLimitSwitch.get())
      wheelsPID.setReference(sped, ControlType.kVelocity);
      // Wheels.set(sped/5650);
    else {
      stopIntakeWheels();
    }

    // System.out.println();
    // leftWheels.set(1);
    // rightWheels.set(1);
  }
   public void spinIntakeIgnoreLimit(double sped) {
    wheelsPID.setP(OperatorConstants.intakeWheelskP);
    wheelsPID.setI(OperatorConstants.intakeWheelskI);
    wheelsPID.setD(OperatorConstants.intakeWheelskD);
    wheelsPID.setIZone(OperatorConstants.intakeWheelskIz);
    wheelsPID.setFF(OperatorConstants.intakeWheelskFF);
    wheelsPID.setOutputRange(-1, 1);
    wheelsPID.setReference(sped, ControlType.kVelocity);
  }
  public boolean calibrate () {
    // pivotMotor.set(-0.1);
    // if (pivotLimitSwitch.get()) {
    //   pivotMotor.set(0);
    //   pivotEncoder.setPosition(0);
    // }
    // return pivotLimitSwitch.get(); 
      return true;
  }
  public void stopIntakeWheels(){
    Wheels.set(0);
  }

  public void syncEncoder(){
    // pivotEncoder.setPosition(-absoluteEncoder.getPosition().getValueAsDouble()*75);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println(absoluteEncoder.getPosition().getValueAsDouble()*75);
    // pivotEncoder.setPosition(absoluteEncoder.getPosition().getValueAsDouble()*75);
  }

  @Override
  public void updateSmartdashboard() {
    // System.out.println(pivotEncoder.getPosition());
    SmartDashboard.putNumber("Pivot Relative", pivotEncoder.getPosition());
    SmartDashboard.putNumber("Something", pivotMotor.getAppliedOutput());
    // SmartDashboard.putNumber("Pivot Absolute", -absoluteEncoder.getPosition());
    // System.out.println(-absoluteEncoder.getPosition().getValueAsDouble()*75);
    // boolean pivotSynced = false;
    // if(Math.abs(pivotEncoder.getPosition()-(absoluteEncoder.getPosition().getValueAsDouble()*75))<0.1)
    //   pivotSynced = true;
    // SmartDashboard.putBoolean("Pivot Synced", pivotSynced);
  }
}
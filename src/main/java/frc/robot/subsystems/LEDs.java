// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.BlinkinLEDController;
import frc.robot.Util.ISubsystem;
import frc.robot.Util.BlinkinLEDController.BlinkinPattern;

public class LEDs extends SubsystemBase implements ISubsystem{

  private int indicate = 0;
  private BlinkinLEDController blinkin = BlinkinLEDController.getInstance();

  /** Creates a new LEDs. */
  public LEDs() {
    
  }

  public void amplify() {
    indicate = 1;

  }
  public void coopertition() {
    indicate = 2;
  }
  public void cycle() {
    indicate = 0;
  }


  @Override
  public void periodic() {
    if(indicate==1){
        blinkin.setPattern(BlinkinPattern.GREEN);
      return;
    }
    if(indicate==2){
        blinkin.setPattern(BlinkinPattern.HOT_PINK);
      return;
    }
    blinkin.setAllianceColorChase();
  }

  @Override
  public void updateSmartdashboard() {
    // TODO Auto-generated method stub
  }
}

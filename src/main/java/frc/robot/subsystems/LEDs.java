// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

public class LEDs extends SubsystemBase {

  private AddressableLED led = new AddressableLED(OperatorConstants.LEDPWMID);
  private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(OperatorConstants.numberofLEDs);
  private int count = 0;
  private int indicate = 0;
  /** Creates a new LEDs. */
  public LEDs() {
    
    led.setLength(m_ledBuffer.getLength());
    
    for(int i = 0; i < 30; i++){
           m_ledBuffer.setRGB(i, 0, 0, 255);
           led.setData(m_ledBuffer);
               led.start();
           System.out.println("test");
        }
  }

  public void amplify() {
    indicate = 1;
    for(int i = 0; i < 30; i++){
           m_ledBuffer.setRGB(i, 255, 0, 0);
           led.setData(m_ledBuffer);
               led.start();
           System.out.println("test");
        }
  }
  public void coopertition() {
    indicate = 2;
    for(int i = 0; i < 30; i++){
           m_ledBuffer.setRGB(i, 0, 255, 0);
           led.setData(m_ledBuffer);
               led.start();
           System.out.println("test");
        }
  }
  public void cycle() {
    indicate = 0;
  }


  @Override
  public void periodic() {
    if(indicate==1 || indicate == 2){
        
      return;
    }
    if(indicate==2){
      for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setLED(i, new Color("#DE8218"));
      }
      led.setData(m_ledBuffer);
      return;
    }
    count += 1;
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      String color;
      int colorID = i%5 + count%5;
      switch (colorID) {
        case 0:
          color = "#780C0C";
          break;
        case 1:
          color = "#8F800E";
          break;
        case 2:
          color = "#1E6115";
          break;
        case 3:
          color = "#8F800E";
          break;
        case 4:
          color = "#1E6115";
          break;
        default:
          color = "#000000";
          break;
      }
      m_ledBuffer.setLED(i, new Color(color));
      System.out.println(color);
    }
    if(count > 500){
      count = 0;
    }
    led.setData(m_ledBuffer);
    led.start();
  }
}

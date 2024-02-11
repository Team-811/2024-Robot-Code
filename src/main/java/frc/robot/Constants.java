// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {
    public static class OperatorConstants {
        public static final int intakeMotorWheelsID = 18;
        public static final int noteLimitSwitchID = 1;
        public static final int pivotLimitSwitchID = 2;
        public static final int pivotMotorID = 17;
        //                                                      V enter degrees right there
        public static final double intakeShooterSetPoint = 0;
        public static final double intakeFloorSetPoint = -(75* 40 )/360;
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final double intakeWheelskP = 0.00001;
        public static final double intakeWheelskI = 0.000001;
        public static final double intakeWheelskD = 0.001;
        public static final double intakeWheelskIz = 0;
        public static final double intakeWheelskFF = 0.00000481;
        public static final double intakePivotkP = 1;
        public static final double intakePivotkI = 0;
        public static final double intakePivotkD = 0;
        public static final double intakePivotkIz = 0;
        public static final double intakePivotkFF = 0;
        public static final double intakeWheelsSpeed = 1250;
        public static final double shooterkP = 0.00001;
        public static final double shooterkI = 0.000001;
        public static final double shooterkD = 0.001;
        public static final double shooterkIz = 0;
        public static final double shooterkFF = 0.00000481;
        public static final int leftShooterWheelID  = 3 ;
        public static final int rightShooterWheelID  = 7 ;
        //max speed 5675
        public static final double shootSpeed = 5675;
        public static final double ampSpeed = 350; 
      }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Utils.FieldCentricFacingAngle180;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.FieldCentricFacingAngle driveFacing = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(MaxSpeed * 0.2).withDriveRequestType(DriveRequestType.OpenLoopVoltage);
      private final FieldCentricFacingAngle180 driveFacing180 = new FieldCentricFacingAngle180()
      .withDeadband(MaxSpeed * 0.2).withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final PhoenixPIDController steerController = new PhoenixPIDController(10, 0, 0.05);
  private final PhoenixPIDController steerController180 = new PhoenixPIDController(0.3, 0.001, 0.01);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private double speedScale = 0.45;
  private double slowSpeed = 0.2;

  /* Path follower */
  // private Command runAuto = drivetrain.getAutoPath("3 Note Auto");
  private Command runAuto = drivetrain.getAutoPath("3 Note Auto");
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    driveFacing.HeadingController = steerController;
    driveFacing180.HeadingController = steerController180;


    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joyLeftY() * MaxSpeed * speedScale) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joyLeftX() * MaxSpeed * speedScale) // Drive left with negative X (left)
            .withRotationalRate(-joyRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));

    joystick.leftBumper().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(-joyLeftY() * MaxSpeed * slowSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joyLeftX() * MaxSpeed * slowSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joyRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));
    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));
    joystick.x().whileTrue(drivetrain.applyRequest(()-> driveFacing.withVelocityX(-joystick.getLeftY() * MaxSpeed * speedScale) // Drive forward with
                                                                                                             // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed * speedScale) // Drive left with negative X (left)
            .withTargetDirection(Rotation2d.fromDegrees(-90))
    ));
    joystick.y().whileTrue(drivetrain.applyRequest(()-> driveFacing.withVelocityX(-joystick.getLeftY() * MaxSpeed * speedScale) // Drive forward with
                                                                                                             // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed * speedScale) // Drive left with negative X (left)
            .withTargetDirection(Rotation2d.fromDegrees(0))
    ));

    // reset the field-centric heading on left bumper press
    joystick.leftTrigger().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    joystick.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    joystick.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
  }

  public RobotContainer() {     
    configureBindings();
    
  }

  public double joyLeftY(){
    if(joystick.getLeftY()< -0.1 || joystick.getLeftY() > 0.1)
      return joystick.getLeftY()*joystick.getLeftY();
    return 0;
  }

    public double joyLeftX(){
    if(joystick.getLeftX()< -0.1 || joystick.getLeftX() > 0.1)
      return joystick.getLeftX()*joystick.getLeftY();
    return 0;
  }

    public double joyRightX(){
    if(joystick.getRightX()< -0.1 || joystick.getRightX() > 0.1)
      return joystick.getRightX()*joystick.getLeftY();
    return 0;
  }

  public Command getAutonomousCommand() {
    // return new SequentialCommandGroup(new ParallelDeadlineGroup(new WaitCommand(5), drivetrain.applyRequest(()->point.withModuleDirection(Rotation2d.fromDegrees(0)))),new ParallelDeadlineGroup(new WaitCommand(5), drivetrain.applyRequest(()->point.withModuleDirection(Rotation2d.fromDegrees(90)))),new ParallelDeadlineGroup(new WaitCommand(5), drivetrain.applyRequest(()->point.withModuleDirection(Rotation2d.fromDegrees(180)))));
    /* First put the drivetrain into auto run mode, then run the auto */
    return runAuto;
  }
}

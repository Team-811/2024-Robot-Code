// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Util.FieldCentricFacingAngle180;
import frc.robot.commands.AmpShootingCommandGroup;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OneNoteAuto;
import frc.robot.commands.ShootingCommand2;
import frc.robot.commands.ShootingCommandGroup;
import frc.robot.commands.ThreeNoteAuto;
import frc.robot.commands.TwoNoteAuto;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoooter;

public class RobotContainer {
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driveController = new CommandXboxController(OperatorConstants.kDriverControllerPort); 
   private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);// My driveController
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final Shoooter shooter = new Shoooter();
  private final Intake intake = new Intake();

  private SlewRateLimiter slewwyY = new SlewRateLimiter(0.75);
  private SlewRateLimiter slewwyX = new SlewRateLimiter(.75);

  private SendableChooser<String> startPositionChooser = new SendableChooser<>();
  private SendableChooser<Integer> numberOfNotesChooser = new SendableChooser<>();
  private SendableChooser<String> note1Chooser = new SendableChooser<>();
  private SendableChooser<String> note2Chooser = new SendableChooser<>();
  private SendableChooser<String> note3Chooser = new SendableChooser<>();



  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed*0.05).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.FieldCentricFacingAngle driveFacing = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(MaxSpeed * 0.05).withDriveRequestType(DriveRequestType.OpenLoopVoltage);
      private final FieldCentricFacingAngle180 driveFacing180 = new FieldCentricFacingAngle180()
      .withDeadband(MaxSpeed * 0.2).withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final PhoenixPIDController steerController = new PhoenixPIDController(3, 0, 0.05);
  private final PhoenixPIDController steerController180 = new PhoenixPIDController(0.3, 0.001, 0.01);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private double speedScale = 0.45;
  private double slowSpeed = 0.1;

  /* Path follower */
  // private Command runAuto = drivetrain.getAutoPath("3 Note Auto");
  // private Command runAuto = drivetrain.getAutoPath("3 Note Auto");
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    driveFacing.HeadingController = steerController;
    driveFacing180.HeadingController = steerController180;
    operatorController.leftTrigger().whileTrue(new InstantCommand(()->intake.syncEncoder(), intake));
    operatorController.b().whileTrue(new AmpShootingCommandGroup(intake,shooter));
    intake.setDefaultCommand(new IntakeCommand(intake, ()-> operatorController.a().getAsBoolean(), ()-> operatorController.rightBumper().getAsBoolean(),()-> operatorController.leftBumper().getAsBoolean(),()->operatorController.y().getAsBoolean()));
    operatorController.x().whileTrue(new ShootingCommandGroup(intake,shooter));
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(slewwyY.calculate(joyLeftY()) * MaxSpeed * speedScale) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(slewwyX.calculate(joyLeftX()) * MaxSpeed * speedScale) // Drive left with negative X (left)
            .withRotationalRate(-joyRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));

    driveController.leftBumper().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(slewwyY.calculate(joyLeftY() * MaxSpeed * slowSpeed)) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(slewwyX.calculate(joyLeftX() * MaxSpeed * slowSpeed)) // Drive left with negative X (left)
            .withRotationalRate(-joyRightX() * MaxAngularRate*0.5) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));
    driveController.rightBumper().whileTrue(drivetrain.applyRequest(() -> brake));
    driveController.b().whileTrue(drivetrain.applyRequest(()-> driveFacing.withVelocityX(slewwyY.calculate(joyLeftY()) * MaxSpeed * speedScale) // Drive forward with
                                                                                                             // negative Y (forward)
            .withVelocityY(slewwyX.calculate(joyLeftX()) * MaxSpeed * speedScale) // Drive left with negative X (left)
            .withTargetDirection(Rotation2d.fromDegrees(90))
    ));
    driveController.x().whileTrue(drivetrain.applyRequest(()-> driveFacing.withVelocityX(slewwyY.calculate(joyLeftY()) * MaxSpeed * speedScale) // Drive forward with
                                                                                                             // negative Y (forward)
            .withVelocityY(slewwyX.calculate(joyLeftX()) * MaxSpeed * speedScale) // Drive left with negative X (left)
            .withTargetDirection(Rotation2d.fromDegrees(0))
    ));
    driveController.a().whileTrue(drivetrain.applyRequest(()-> driveFacing.withVelocityX(slewwyY.calculate(joyLeftY()) * MaxSpeed * speedScale) // Drive forward with
                                                                                                             // negative Y (forward)
            .withVelocityY(slewwyX.calculate(joyLeftX()) * MaxSpeed * speedScale) // Drive left with negative X (left)
            .withTargetDirection(Rotation2d.fromDegrees(-60))
    ));
    driveController.y().whileTrue(drivetrain.applyRequest(()-> driveFacing.withVelocityX(slewwyY.calculate(joyLeftY()) * MaxSpeed * speedScale) // Drive forward with
                                                                                                             // negative Y (forward)
            .withVelocityY(slewwyX.calculate(joyLeftX()) * MaxSpeed * speedScale) // Drive left with negative X (left)
            .withTargetDirection(Rotation2d.fromDegrees(60))
    ));

    // reset the field-centric heading on left bumper press
    driveController.leftTrigger().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(-90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    driveController.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    driveController.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
  }

  public RobotContainer() {
    // NamedCommands.registerCommand("LowerIntake", new LowerIntake(intake));
    configureBindings();
    SignalLogger.stop();

    startPositionChooser.setDefaultOption("Middle Side", "Mid");
    startPositionChooser.addOption("Amp Side", "Amp");
    startPositionChooser.addOption("Stage Side", "Stage");

    numberOfNotesChooser.addOption("0", 0);
    numberOfNotesChooser.addOption("1", 1);
    numberOfNotesChooser.setDefaultOption("2", 2);
    numberOfNotesChooser.addOption("3", 3);
    numberOfNotesChooser.addOption("4", 4);

    note1Chooser.addOption("Stage", "Stage");
    note1Chooser.addOption("Amp", "Amp");
    note1Chooser.setDefaultOption("Subwoofer", "Subwoofer");
    note1Chooser.addOption("Crazy", "GoCrazy");

    note2Chooser.setDefaultOption("Amp", "Amp");
    note2Chooser.addOption("Stage", "Stage");
    note2Chooser.addOption("Subwoofer", "Subwoofer");

    note3Chooser.setDefaultOption("Stage", "Stage");
    note3Chooser.addOption("Amp", "Amp");
    note3Chooser.addOption("Subwoofer", "Subwoofer");



    // autoChooser.addOption("Stage Note", new OneNoteAuto(intake, drivetrain, shooter, ""));
  }

  public double joyLeftY(){
    int sign = 1;
    if(driveController.getLeftY()<0)
      sign=-1;
    if(driveController.getLeftY()< -0.1 || driveController.getLeftY() > 0.1)
      return driveController.getLeftY()*driveController.getLeftY()*sign;
    return 0;
  }

  public double joyLeftX(){
    int sign = 1;
    if(driveController.getLeftX()<0)
      sign=-1;
    if(driveController.getLeftX()< -0.1 || driveController.getLeftX() > 0.1)
      return driveController.getLeftX()*driveController.getLeftX()*sign;
    return 0;
  }

  public double joyRightX(){
    int sign = 1;
    if(driveController.getRightX()<0)
      sign=-1;
    if(driveController.getRightX()< -0.1 || driveController.getRightX() > 0.1)
      return driveController.getRightX()*driveController.getRightX()*sign*0.6;
    return 0;
  }

  public Command getAutonomousCommand() {
    // return new SequentialCommandGroup(new ParallelDeadlineGroup(new WaitCommand(5), drivetrain.applyRequest(()->point.withModuleDirection(Rotation2d.fromDegrees(0)))),new ParallelDeadlineGroup(new WaitCommand(5), drivetrain.applyRequest(()->point.withModuleDirection(Rotation2d.fromDegrees(90)))),new ParallelDeadlineGroup(new WaitCommand(5), drivetrain.applyRequest(()->point.withModuleDirection(Rotation2d.fromDegrees(180)))));
    /* First put the drivetrain into auto run mode, then run the auto */
    // return new SequentialCommandGroup(new ShootingCommandGroup(intake, shooter),drivetrain.getAutoPath("Taxi Auto"));
    // return drivetrain.getAutoPath("Taxi Auto");
    if(note1Chooser.getSelected().equals("GoCrazy") && numberOfNotesChooser.getSelected().intValue()!=0)
      return new OneNoteAuto(intake,drivetrain,shooter,"Godjaosjo");
    String firstNote = startPositionChooser.getSelected() + note1Chooser.getSelected();
    String secondNote = "Mid" + note2Chooser.getSelected();
    String thirdNote = "Mid" + note3Chooser.getSelected();
    String taxi = startPositionChooser.getSelected() + "CrazyTaxi"+"DO NOT";
    SmartDashboard.putString("First Note Path", firstNote);
    switch (numberOfNotesChooser.getSelected().intValue()) {
      case 0:
        //Taxi
      case 1:
        return new OneNoteAuto(intake, drivetrain, shooter, taxi);
      case 2:
        return new OneNoteAuto(intake, drivetrain, shooter, firstNote);
      case 3:
        return new TwoNoteAuto(intake, drivetrain, shooter, firstNote, secondNote);
      case 4:
        return new ThreeNoteAuto(intake, drivetrain, shooter, firstNote, secondNote, thirdNote);
      default:
        break;
    }
    return null;

    // return new OneNoteAuto(intake, drivetrain, shooter, "1 Note Middle");
  }

  public void updateSmartdashboard(){
    intake.updateSmartdashboard();
    shooter.updateSmartdashboard();
    drivetrain.updateSmartdashboard();
    startPositionChooser.setDefaultOption("Middle Side", "Mid");
    startPositionChooser.addOption("Amp Side", "Amp");
    startPositionChooser.addOption("Stage Side", "Stage");

    numberOfNotesChooser.addOption("0", 0);
    numberOfNotesChooser.addOption("1", 1);
    numberOfNotesChooser.setDefaultOption("2", 2);
    numberOfNotesChooser.addOption("3", 3);
    numberOfNotesChooser.addOption("4", 4);

    note1Chooser.addOption("Stage", "Stage");
    note1Chooser.addOption("Amp", "Amp");
    note1Chooser.setDefaultOption("Subwoofer", "Subwoofer");
    note1Chooser.addOption("Crazy", "GoCrazy");

    note2Chooser.setDefaultOption("Amp", "Amp");
    note2Chooser.addOption("Stage", "Stage");
    note2Chooser.addOption("Subwoofer", "Subwoofer");

    note3Chooser.setDefaultOption("Stage", "Stage");
    note3Chooser.addOption("Amp", "Amp");
    note3Chooser.addOption("Subwoofer", "Subwoofer");

    SmartDashboard.putData("Start Position", startPositionChooser);
    SmartDashboard.putData("Number of Auto Notes", numberOfNotesChooser);
    SmartDashboard.putData("First Note", note1Chooser);
    SmartDashboard.putData("Second Note", note2Chooser);
    SmartDashboard.putData("Third Note", note3Chooser);
  }
}

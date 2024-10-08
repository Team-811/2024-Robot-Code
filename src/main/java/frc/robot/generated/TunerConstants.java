package frc.robot.generated;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.util.Units;
import frc.robot.CommandSwerveDrivetrain;

public class TunerConstants {
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(20).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(0.12781).withKI(0).withKD(0)
        .withKS(0.10201).withKV(0.1137).withKA(0.029163);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double kSlipCurrentA = 300.0;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double kSpeedAt12VoltsMps = 3.92;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3.5714285714285716;

    private static final double kDriveGearRatio = 8.142857142857142;
    private static final double kSteerGearRatio = 21.428571428571427;
    private static final double kWheelRadiusInches = 2;

    private static final boolean kSteerMotorReversed = true;
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final String kCANbusName = "";
    private static final int kPigeonId = 10;


    // These are only used for simulation
    private static final double kSteerInertia = 0.00001;
    private static final double kDriveInertia = 0.001;
    // Simulated voltage necessary to overcome friction
    private static final double kSteerFrictionVoltage = 0.25;
    private static final double kDriveFrictionVoltage = 0.25;

    private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withPigeon2Id(kPigeonId)
            .withCANbusName(kCANbusName);

    private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSlipCurrent(kSlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(kCoupleRatio)
            .withSteerMotorInverted(kSteerMotorReversed);


                // Front Left
    private static final int kFrontLeftDriveMotorId = 8;
    private static final int kFrontLeftSteerMotorId = 11;
    private static final int kFrontLeftEncoderId = 12;
    private static final double kFrontLeftEncoderOffset = -0.307373046875;

//     private static final double kFrontLeftXPosInches = 14;
//     private static final double kFrontLeftYPosInches = 14;

    // Front Right
    private static final int kFrontRightDriveMotorId = 6;
    private static final int kFrontRightSteerMotorId = 5;
    private static final int kFrontRightEncoderId = 16;
    private static final double kFrontRightEncoderOffset = 0.210205078125;

//     private static final double kFrontRightXPosInches = 14;
//     private static final double kFrontRightYPosInches = -14;

    // Back Left
    private static final int kBackLeftDriveMotorId = 13;
    private static final int kBackLeftSteerMotorId = 2;
    private static final int kBackLeftEncoderId = 14;
    private static final double kBackLeftEncoderOffset = 0.2353515625;

//     private static final double kBackLeftXPosInches = -14;
//     private static final double kBackLeftYPosInches = 14;

    // Back Right
    private static final int kBackRightDriveMotorId = 4;
    private static final int kBackRightSteerMotorId = 9;
    private static final int kBackRightEncoderId = 15;
    private static final double kBackRightEncoderOffset = 0.3115234375;
//     // Front Left
//     private static final int kFrontLeftDriveMotorId = 6;
//     private static final int kFrontLeftSteerMotorId = 5;
//     private static final int kFrontLeftEncoderId = 16;
//     private static final double kFrontLeftEncoderOffset = -0.03857421875;

    private static final double kFrontLeftXPosInches = 11;
    private static final double kFrontLeftYPosInches = 11;

//     // Front Right
//     private static final int kFrontRightDriveMotorId = 4;
//     private static final int kFrontRightSteerMotorId = 9;
//     private static final int kFrontRightEncoderId = 15;
//     private static final double kFrontRightEncoderOffset = -0.440185546875;

    private static final double kFrontRightXPosInches = 11;
    private static final double kFrontRightYPosInches = -11;

//     // Back Left
//     private static final int kBackLeftDriveMotorId = 8;
//     private static final int kBackLeftSteerMotorId = 11;
//     private static final int kBackLeftEncoderId = 12;
//     private static final double kBackLeftEncoderOffset = -0.0712890625;

    private static final double kBackLeftXPosInches = -11;
    private static final double kBackLeftYPosInches = 11;

//     // Back Right
//     private static final int kBackRightDriveMotorId = 13;
//     private static final int kBackRightSteerMotorId = 2;
//     private static final int kBackRightEncoderId = 14;
//     private static final double kBackRightEncoderOffset = -0.0126953125;

    private static final double kBackRightXPosInches = -11;
    private static final double kBackRightYPosInches = -11;


    private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
    private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
    private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
    private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);

    public static final CommandSwerveDrivetrain DriveTrain = new CommandSwerveDrivetrain(DrivetrainConstants, FrontLeft,
            FrontRight, BackLeft, BackRight);
}

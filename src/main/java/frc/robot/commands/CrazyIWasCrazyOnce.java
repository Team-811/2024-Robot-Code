// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CrazyIWasCrazyOnce extends ParallelCommandGroup {
  /** Creates a new CrazyIWasCrazyOnce. */
  public CrazyIWasCrazyOnce(Intake outie, CommandSwerveDrivetrain swervy, Shoooter spitout, String pathName) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(swervy.getAutoPath(pathName), new SpinIntake(outie).until(outie::limitSwitchOutput).andThen(new RaiseNShoot(outie, spitout)).repeatedly());
  }
}

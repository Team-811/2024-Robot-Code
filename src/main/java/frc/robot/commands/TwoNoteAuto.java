// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoNoteAuto extends SequentialCommandGroup {
  /** Creates a new TwoNoteAuto. */
  public TwoNoteAuto(Intake outie, CommandSwerveDrivetrain swervy, Shoooter spitout, String pathName1, String pathName2) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new OneNoteAuto(outie, swervy, spitout, pathName1), new SequencedNoteAuto(outie, swervy, spitout, pathName2));
  }
}

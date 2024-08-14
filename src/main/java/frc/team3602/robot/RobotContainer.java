/*
 * Copyright (C) 2023 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team3602.robot.subsystems.armSubsys;
import frc.team3602.robot.subsystems.intakeSubsys;
import frc.team3602.robot.subsystems.shooterSubsys;
import frc.team3602.robot.subsystems.pivotSubsys;
import frc.team3602.robot.subsystems.DrivetrainSubsystem;
import static frc.team3602.robot.Constants.DrivetrainConstants.*;


//import frc.team3602.robot.Superstructure;

import static frc.team3602.robot.Constants.ControllerPortConstants.*;

public class RobotContainer {
  public final CommandXboxController xboxController = new CommandXboxController(kXboxControllerPort);
  // Autonomous
  private final DrivetrainSubsystem driveSubsys = new DrivetrainSubsystem(
      kDrivetrainConstants,
      xboxController,
      kFrontLeftModuleConstants,
      kFrontRightModuleConstants,
      kBackLeftModuleConstants,
      kBackRightModuleConstants);
  SendableChooser<Command> sendableChooser = new SendableChooser<>();
  public final armSubsys armSubsys = new armSubsys();
  public final intakeSubsys intakeSubsys = new intakeSubsys();
  public final shooterSubsys shooterSubsys = new shooterSubsys();
  public final pivotSubsys pivotSubsys = new pivotSubsys();
  public final Superstructure superstructure = new Superstructure(intakeSubsys, shooterSubsys, pivotSubsys, driveSubsys);
  
  public RobotContainer() {
    configDefaultCommands();
    configButtonBindings();
    configAutonomous();
  }

  private void configDefaultCommands() {
    // driveSubsys
    //     .setDefaultCommand(driveSubsys.applyRequest(
    //         () -> driveSubsys.fieldCentricDrive
    //             .withVelocityX(polarityChooser.getSelected() * xboxController.getLeftY() *
    //                 _kMaxSpeed)
    //             .withVelocityY(polarityChooser.getSelected() * xboxController.getLeftX() *
    //                 _kMaxSpeed)
    //             .withRotationalRate(-xboxController.getRightX() *
    //                 _kMaxAngularRate)));
    armSubsys.setDefaultCommand(armSubsys.holdHeights());
  }


  private void configAutonomous() {
    SmartDashboard.putData(sendableChooser);
  }

  public Command getAutonomousCommand() {
    return sendableChooser.getSelected();
  }

  
  private void configButtonBindings() {

    //getNote
    xboxController.a().whileTrue(superstructure.getNote());
    xboxController.x().whileTrue(superstructure.manuallyGetNote());

    //shooter sequences
    xboxController.rightBumper().onTrue(superstructure.speakerPrepToShoot());
    xboxController.rightTrigger().onTrue(superstructure.shootCmd());
    xboxController.leftBumper().onTrue(superstructure.ampPrepToShoot());
    xboxController.leftTrigger().onTrue(superstructure.shootCmd());

    //intake forward and reverse
    xboxController.b().whileTrue(intakeSubsys.runIntake(() -> 0.6));
    xboxController.y().whileTrue(intakeSubsys.runIntake(() -> -0.6));

    //arm controls
    xboxController.pov(180).onTrue(armSubsys.setHeight(() -> 26.5));
    xboxController.pov(-180).onTrue(armSubsys.setHeight(() -> 47));
  }
}
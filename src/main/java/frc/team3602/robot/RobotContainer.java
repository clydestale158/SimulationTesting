/*
 * Copyright (C) 2023 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team3602.robot.generated.TunerConstants;

import frc.team3602.robot.Subsystems.PivotSubsystem;
import frc.team3602.robot.Subsystems.ShooterSubsystem;
import frc.team3602.robot.Constants.shooterConstants;
import frc.team3602.robot.Subsystems.IntakeSubsystem;
//import static frc.team3602.robot.Constants.DrivetrainConstants.*;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import static frc.team3602.robot.Constants.Drivetrain.*;


//import frc.team3602.robot.Superstructure;

import static frc.team3602.robot.Constants.ControllerPortConstants.*;

public class RobotContainer {
  public final CommandXboxController xboxController = new CommandXboxController(kXboxControllerPort);
  public final CommandJoystick joystick = new CommandJoystick(0);
  public final CommandJoystick portOneJoystick = new CommandJoystick(1);
  //public final CommandJoystick portTwoJoystick = new CommandJoystick(2);

    private final Drivetrain drivetrain = TunerConstants.DriveTrain;

  public final VisionSystem visSys = new VisionSystem(() -> drivetrain.getState().Pose);

  private final ShooterSubsystem shooterSubsys = new ShooterSubsystem();
    private final IntakeSubsystem intakeSubsys = new IntakeSubsystem();
  private final PivotSubsystem pivotSubsys = new PivotSubsystem(intakeSubsys, shooterSubsys);


   private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(Constants.Drivetrain.kMaxSpeed * Constants.Drivetrain.kDeadband)
          .withRotationalDeadband(
              Constants.Drivetrain.kMaxAngularRate * Constants.Drivetrain.kDeadband)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

          private final SendableChooser<Command> autoChooser;
          private SendableChooser<Double> polarityChooser = new SendableChooser<>();


  public RobotContainer() {
    configDefaultCommands();
    configJoystickBindings();
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Drive Polarity", polarityChooser);

    polarityChooser.setDefaultOption("Default", 1.0);
    polarityChooser.addOption("Positive", 1.0);
    polarityChooser.addOption("Negative", -1.0);
  }

  private void configDefaultCommands() {
    pivotSubsys.simHoldAngle();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

    
  private void configJoystickBindings() {
    joystick.button(1).onTrue(pivotSubsys.simRunSetAngle(() -> 85).until(() -> pivotSubsys.simIsAtPosition));
    joystick.button(2).onTrue(pivotSubsys.simRunSetAngle(() -> 15).until(() -> pivotSubsys.simIsAtPosition));
    joystick.button(3).whileTrue(shooterSubsys.runShooterSpeed(0.8, 0.8)).onFalse(shooterSubsys.stopMotorsCmd());
    joystick.button(4).whileTrue(intakeSubsys.runIntake(() -> 0.6));


 drivetrain.setDefaultCommand(
      drivetrain
          .applyRequest(
              () ->
                  drive
                      .withVelocityX(-joystick.getRawAxis(0) * Constants.Drivetrain.kMaxSpeed)// .withVelocityX(-controller.getLeftY() * Constants.Drivetrain.kMaxSpeed)
                      .withVelocityY(-joystick.getRawAxis(1) * Constants.Drivetrain.kMaxSpeed)//.withVelocityY(-controller.getLeftX() * Constants.Drivetrain.kMaxSpeed)
                      .withRotationalRate(
                          -portOneJoystick.getRawAxis(1) * Constants.Drivetrain.kMaxAngularRate))
          .ignoringDisable(true));

  //   //getNote
  //   joystick.button(0).whileTrue(superstructure.getNote());
  //   joystick.button(1).whileTrue(superstructure.manuallyGetNote());

  //   //shooter sequences
  //   portOneJoystick.button(0).onTrue(superstructure.speakerPrepToShoot());
  //   portOneJoystick.button(1).onTrue(superstructure.shootCmd());
  //   portOneJoystick.button(2).onTrue(superstructure.ampPrepToShoot());
  //   portOneJoystick.button(3).onTrue(superstructure.shootCmd());

  //   //intake forward and reverse
  //   joystick.button(2).whileTrue(intakeSubsys.runIntake(() -> 0.6));
  //   joystick.button(3).whileTrue(intakeSubsys.runIntake(() -> -0.6));

   }
   

}

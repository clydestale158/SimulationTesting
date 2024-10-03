//author - Caroline
package frc.team3602.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import frc.team3602.robot.generated.TunerConstants;

public class RobotContainer {
  //our subsystems and vars
    //in the sim gui dashboard, you can use parts of your keyboard as joysticks. 
  public final CommandJoystick joystick = new CommandJoystick(0);
  public final CommandJoystick portOneJoystick = new CommandJoystick(1);

  private final Drivetrain drivetrain = TunerConstants.DriveTrain;
  public final VisionSystem visSys = new VisionSystem(() -> drivetrain.getState().Pose); 
  public final Simulation simulation = new Simulation();


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
  drivetrain.setDefaultCommand(
      drivetrain
          .applyRequest(
              () ->
                  drive
                      .withVelocityX(-joystick.getRawAxis(0) * Constants.Drivetrain.kMaxSpeed)// .withVelocityX(-controller.getLeftY() * Constants.Drivetrain.kMaxSpeed)
                      .withVelocityY(joystick.getRawAxis(1) * Constants.Drivetrain.kMaxSpeed)//.withVelocityY(-controller.getLeftX() * Constants.Drivetrain.kMaxSpeed)
                      .withRotationalRate(
                          -portOneJoystick.getRawAxis(1) * Constants.Drivetrain.kMaxAngularRate))
          .ignoringDisable(true));
  }
    
  private void configJoystickBindings() {
        joystick.button(1).whileTrue(simulation.testIntake());
        joystick.button(2).whileTrue(simulation.testShooter());
        joystick.button(3).whileTrue(simulation.testPivot());
        joystick.button(4).whileTrue(simulation.testPivotReverse());
   }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

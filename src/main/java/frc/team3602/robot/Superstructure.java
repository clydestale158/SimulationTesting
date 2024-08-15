package frc.team3602.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team3602.robot.subsystems.intakeSubsys;
import frc.team3602.robot.subsystems.shooterSubsys;
import frc.team3602.robot.subsystems.pivotSubsys;
import frc.team3602.robot.subsystems.DrivetrainSubsystem;

public class Superstructure {
    /*variables */
    private final intakeSubsys intakeSubsys;
    private final shooterSubsys shooterSubsys;
    private final pivotSubsys pivotSubsys;
    private final DrivetrainSubsystem driveSubsys;

    /*the good stuff */
    public Superstructure(intakeSubsys intakeSubsys, shooterSubsys shooterSubsys, pivotSubsys pivotSubsys, DrivetrainSubsystem driveSubsys){
        this.intakeSubsys = intakeSubsys;
        this.shooterSubsys = shooterSubsys;
        this.pivotSubsys = pivotSubsys;
        this.driveSubsys = driveSubsys;
    }


    public Command shootCmd() {
        return Commands.sequence(
        intakeSubsys.runIntake(() -> 0.6).until(() -> intakeSubsys.hasNote == false),
        pivotSubsys.setPivot(() -> 10),
        shooterSubsys.stopShooters()

        );
    }

    public Command manuallyGetNote(){
        return Commands.sequence(
             pivotSubsys.setPivot(() -> 9).until(() -> pivotSubsys.atAngle),
            // angle 11>9
                intakeSubsys.runIntake(() -> 0.75).until(() -> intakeSubsys.beamSensor.get()),
            Commands.parallel(
                pivotSubsys.setPivot(() -> 25).until(() -> pivotSubsys.isAtPosition),
                intakeSubsys.runIntake(() -> 0.15).until(() -> intakeSubsys.colorSensor.get()))
        );
    }
    
    public Command getNote() {
        return Commands.sequence(
            pivotSubsys.setPivot(() -> 9).until(() -> pivotSubsys.atAngle),
            // angle 11>9
            Commands.parallel(
                intakeSubsys.runIntake(() -> 0.75).until(() -> intakeSubsys.beamSensor.get()),
                driveSubsys.driveTowardNote().until(() -> intakeSubsys.beamSensor.get())),
            Commands.parallel(
                pivotSubsys.setPivot(() -> 25).until(() -> pivotSubsys.isAtPosition),
                intakeSubsys.runIntake(() -> 0.15).until(() -> intakeSubsys.colorSensor.get())));
    
      }

      public Command ampPrepToShoot(){
        return Commands.parallel(
            pivotSubsys.setPivot(() -> 95),
            shooterSubsys.runShooter(() -> 0.3)
        );
      }

      public Command speakerPrepToShoot() {
        return Commands.parallel(
          pivotSubsys.setPivot(() -> 25),
          shooterSubsys.runShooter(() -> 0.8) 
        );
      }
}

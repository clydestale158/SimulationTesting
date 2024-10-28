package frc.team3602.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3602.robot.Subsystems.IntakeSubsystem;
import frc.team3602.robot.Subsystems.PivotSubsystem;
import frc.team3602.robot.Subsystems.ShooterSubsystem;

public class Superstructure extends SubsystemBase{
public PivotSubsystem pivotSubsys;
public IntakeSubsystem intakeSubsys;
public ShooterSubsystem shooterSubsys;
public Vision vision;

public Superstructure(PivotSubsystem pivotSubsys, IntakeSubsystem intakeSubsys, ShooterSubsystem shooterSubsys, Vision vision){
this.pivotSubsys = pivotSubsys;
this.intakeSubsys = intakeSubsys;
this.shooterSubsys = shooterSubsys;
this.vision = vision;
}

public Command getNote(){
    return Commands.sequence(
        pivotSubsys.setAngle(12)
        
    );
} 





}
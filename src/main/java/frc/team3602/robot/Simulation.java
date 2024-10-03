package frc.team3602.robot;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Simulation extends SubsystemBase{
/*our vars and such */
  //motors (These are just random fake motors)
  private static final TalonFX intakeMotor = new TalonFX(10);
  private static final TalonFX pivotMotor = new TalonFX(11);  
  private static final TalonFX shooterMotor = new TalonFX(12);

    public double pivotPos;
    public double simPivotPos;

  
//naming our sims and declairing what kind.
    // FlywheelSim is representing a flywheel that spins a lot(This is the best fit for our intake and shooter from 2024 Crescendo)
    // SingleJointedArmSim is representing an arm that moves within a set range(Min and max angle) (This is best fit for our pivot from 2024 Crescendo)
  //sim for intake
    private static final FlywheelSim intakeSim =
        new FlywheelSim(DCMotor.getNeo550(1), 5, 0.01);
  //sim for shooter
    private static final FlywheelSim shooterSim = 
        new FlywheelSim(DCMotor.getNEO(1), 1, 0.01);
  //sim for pivot
    private static final SingleJointedArmSim pivotSim = 
        new SingleJointedArmSim(
            DCMotor.getNEO(2),
             48, 0.1, 0.6,  0.0,  3.0, true, 1.57);

//2d visualizations
    //Creates the "background", is through smartdashboard
    private static final Mechanism2d mech = new Mechanism2d(1, 1);

    //places the objects in the background
      //the root2D gives smartdashboard a name and a coordinate for where each line will start relative to the background
      //the ligament2D makes the line that you see on smartdashboard extending from its correlating root
    private static final MechanismRoot2d pivot = mech.getRoot("Pivot Root", 0.7, 0.3); 
    private static final MechanismLigament2d pivotViz = pivot.append( new MechanismLigament2d("pivot ligament", 0.5, 10.0, 5.0, new Color8Bit(Color.kHoneydew)));

    private static final MechanismRoot2d shooter = mech.getRoot("shooter Root", 0.7, 0.3);
    private static final MechanismLigament2d shooterViz = shooter.append(new MechanismLigament2d("Shooter rollers", 0.1, 0.0, 10.0, new Color8Bit(Color.kAquamarine)));

      //Since the intake root will move to the end of the pivot, it doesn't matter where its root is. 
      //We move the intake root in periodic
    private static final MechanismRoot2d intake = mech.getRoot("intake Root", 0.0, 0.0);
    private static final MechanismLigament2d intakeViz = intake.append(new MechanismLigament2d("intake rollers", 0.12, 0.0, 5.0, new Color8Bit(Color.kDarkOrchid)));


//3D pose stuff, only used for AdvantageScope 3D Field, the instructions are in src/main/AdvantageScope_Assets
    //creates the 3D pose that we'll use for the pivot. It contains rotation and translation units
    public Pose3d pivotPose;

    //we use rotation to get the 2D mechanism's pivot position, then we set the pivotPose to this rotation (Done in periodic)
    public Rotation3d rotation;

    //puts the pose on smartdashboard so we can use it
    StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
    .getStructTopic("MyPose", Pose3d.struct).publish();





/* the good stuff, ie our commands and such */
    //our constructor
  public Simulation() {
      //In order to see the 2D mechanism, once the sim gui is running, click on NetworksTable, SmartDashboard then Arm Viz.
      //You may need to do the same to see the 2D field, or autonomous selector(Also, pathplanner does work, there's a basic path included)
            SmartDashboard.putData("Arm Viz", mech);
    }

//Commands to reference
    public Command testShooter() {
        return runEnd(() -> {
            shooterMotor.set(0.8);
        },
        () -> {
            shooterMotor.set(0.0);
        } );
    }


    public Command testIntake() {
        return runEnd(() -> {
            intakeMotor.set(0.8);
        },
        () ->{
            intakeMotor.set(0.0);
    });
    }


    public Command testPivot() {
        return run(() -> {
            pivotMotor.set(0.8);
           
        });
    }

   public Command testPivotReverse() {
        return runOnce(() -> {
            pivotMotor.set(-0.8);
           
        });
    }




    @Override
    public void periodic() {
      //2D Stuff
        intakeSim.setInput(intakeMotor.getMotorVoltage().getValueAsDouble()); 
        intakeSim.update(TimedRobot.kDefaultPeriod);

        shooterSim.setInput(shooterMotor.getMotorVoltage().getValueAsDouble());
        shooterSim.update(TimedRobot.kDefaultPeriod);

        pivotSim.setInput(pivotMotor.getMotorVoltage().getValueAsDouble());
        pivotSim.update(TimedRobot.kDefaultPeriod);

      //Updates the visual based on sim
        pivotViz.setAngle(Math.toDegrees(pivotSim.getAngleRads()));

        intake.setPosition((0.7 + 0.5 * Math.cos(pivotSim.getAngleRads())), (0.3 + 0.5 * Math.sin(pivotSim.getAngleRads())));
        intakeViz.setAngle(intakeViz.getAngle() + Math.toDegrees(intakeSim.getAngularVelocityRPM()) * 0.02 * 0.03);
    
        shooterViz.setAngle(shooterViz.getAngle() + Math.toDegrees(shooterSim.getAngularVelocityRPM()) * 0.02 * 0.03);

      //updates the pivot position
        pivotSim.getAngleRads();
        simPivotPos = pivotSim.getAngleRads();

    //3D Stuff
        rotation = new Rotation3d(0,( -simPivotPos + 90), 0);
        pivotPose  = new Pose3d(0, 0, 0, rotation);
        publisher.set(pivotPose);    
    }

}
 
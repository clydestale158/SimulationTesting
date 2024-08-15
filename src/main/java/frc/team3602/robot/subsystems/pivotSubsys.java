package frc.team3602.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import static frc.team3602.robot.Constants.pivotConstants.*;

import java.util.function.DoubleSupplier;

public class pivotSubsys extends SubsystemBase{
    /*variables */
      //motor
        public final CANSparkMax pivotLeader = new CANSparkMax(kPivotLeaderId, MotorType.kBrushless);
        public final CANSparkMax pivotFollower = new CANSparkMax(kPivotFollowerId, MotorType.kBrushless);

      //encoders
      private final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(2);
      //pid and ff
        private final PIDController pidController = new PIDController(kP, kI, kD);
        private final ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV, kA);
      //other vars
        public boolean isAtPosition;

        public double encoderValue;

        public double absoluteOffset = 77;

        public boolean bruh;

        public double effort;

        public double goalAngle = 90;
        public double direction;

        public boolean atAngle;




    /*the good stuff */
        public pivotSubsys() {
            configPivotSubsys();
        }

        public double getDegrees() {
            return (pivotEncoder.getAbsolutePosition() * 360) - absoluteOffset;
        }

        public boolean nearGoalAngle(){
            return ((Math.abs(encoderValue - goalAngle)) >= 10);
        }

        public boolean isAtAngle(){
            return ((Math.abs(encoderValue - goalAngle)) >= 2);
        }

        public double direction() {
            if(encoderValue >= goalAngle){
                return -1;
            }else{
                return 1;
            }
        }


        public Command holdAngle(){
            return run(() ->{
            var effort = getffEffort();
            this.effort = effort;

            pivotLeader.setVoltage(effort);
             } );
        }

        public double getffEffort() {
            var ffEffort = feedforward.calculate(Units.degreesToRadians(getDegrees()), 0);

            return ffEffort;
        }


        public Command setPivot(DoubleSupplier angle){
            return run(() ->{
                goalAngle = angle.getAsDouble();
                if(bruh = false){
                    bruh();
                } else if (atAngle = false){
                    runPivotVoltage(() -> 3).until(() -> atAngle);
                }
            });
        }

        public Command bruh(){
           return Commands.sequence(
                runPivotVoltage(() -> 7).until(() -> bruh),
                runPivotVoltage(() -> 2).until(() -> atAngle)
           );   
    }

        public Command runPivotVoltage(DoubleSupplier voltage){
            return run(() ->{
                pivotLeader.setVoltage(voltage.getAsDouble());
            });
        }

        //first attempt, was non functional
        // public Command setPivot(DoubleSupplier angle){
        //     return runEnd(() -> {
        //         goalAngle = angle.getAsDouble();
        //         while(bruh = false){
        //             pivotLeader.setVoltage(kHighVolts*direction);
        //         }
        //         while(atAngle = false){
        //             pivotLeader.setVoltage(kLowVolts*direction);
        //         }
                   
        //     },
        //     () -> {
        //         var effort = getffEffort();
        //         pivotLeader.setVoltage(effort);
        //     });
        // }



        @Override 
        public void periodic(){
            bruh = nearGoalAngle();
            encoderValue = getDegrees();
            atAngle = isAtAngle();
        }

    /*pivot subsystem configuration  */
    private void configPivotSubsys() {
        //pivot leader config
        pivotLeader.setIdleMode(IdleMode.kBrake);
        pivotLeader.setInverted(false);
        pivotLeader.setSmartCurrentLimit(kPivotCurrentLimit);
        pivotLeader.enableVoltageCompensation(pivotLeader.getBusVoltage());

        //pivot follower config
        pivotFollower.setIdleMode(IdleMode.kBrake);
        pivotFollower.follow(pivotLeader, true);
        pivotFollower.setSmartCurrentLimit(kPivotCurrentLimit);
        pivotFollower.enableVoltageCompensation(pivotFollower.getBusVoltage());

        pidController.setTolerance(1);

        //burning flashes
        pivotLeader.burnFlash();
        pivotFollower.burnFlash();
    } 
    
}
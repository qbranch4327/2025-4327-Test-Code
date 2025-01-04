package frc.robot.subsystems;

// import com.revrobotics.CANSparkFlex;
// import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.revrobotics.SparkPIDController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Robot;
import frc.robot.RobotConstants;

public class IntakeSubsystem extends SubsystemBase {
    
    SparkFlex intakeMotor;
    public DigitalInput sensor;

    public IntakeSubsystem()    {
        intakeMotor = new SparkFlex(RobotConstants.intakeCANid, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        intakeMotor.setInverted(true);
        sensor = new DigitalInput(RobotConstants.intakesensorDIOid);

        // SparkPIDController intakemotorPID = intakeMotor.getPIDController();
        // intakemotorPID.setP(RobotConstants.intakemotorP);
        // intakemotorPID.setI(RobotConstants.intakemotorI);
        // intakemotorPID.setD(RobotConstants.intakemotorD);
        // intakemotorPID.setFF(RobotConstants.intakemotorFF);

    }

    public void intakeOn(boolean forward)  {
        if (forward)    {
            intakeMotor.set(RobotConstants.intakeOnspeed);
        }
        else {
            intakeMotor.set(RobotConstants.intakeOutspeed);
        }
    }
    
    public void intakeOff() {
        intakeMotor.stopMotor();
    }


    public void intakeSlow(boolean forward)  {
        if (forward)    {
            intakeMotor.set(RobotConstants.intakeSlowspeed);
        }
        else    {
            intakeMotor.set(-RobotConstants.intakeSlowspeed);
        }
    }

}
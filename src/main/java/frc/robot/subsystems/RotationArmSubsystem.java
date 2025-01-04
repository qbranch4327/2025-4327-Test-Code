package frc.robot.subsystems;


//import com.revrobotics.CANSparkFlex;
import com.revrobotics.spark.SparkFlex;
//import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.SparkPIDController;
import com.revrobotics.spark.config.*;

//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

public class RotationArmSubsystem extends SubsystemBase{
    SparkFlex armMotor;
    DutyCycleEncoder armEncoder;
    private final double encoderOffset = RobotConstants.encoderOffset;
    private final double rangeOffset = RobotConstants.rangeOffset;

    public RotationArmSubsystem()   {
        armMotor = new SparkFlex(RobotConstants.armmotorCANid, MotorType.kBrushless);
        armEncoder = new DutyCycleEncoder(RobotConstants.armEncoderDIOid);

        // SparkPIDController armmotorPID = armMotor.getPIDController();
        // armmotorPID.setP(RobotConstants.armmotorP);
        // armmotorPID.setI(RobotConstants.armmotorI);
        // armmotorPID.setD(RobotConstants.armmotorD);
        // armmotorPID.setFF(RobotConstants.armmotorFF);

    }

    public void goTo(double degrees)  {
        
        var position = (armEncoder.get() + encoderOffset) % 1;
        
        if (position > (degrees + rangeOffset + encoderOffset) % 1) {
            this.goUp(RobotConstants.goUPpower);
        }
        else if (position < (degrees - rangeOffset + encoderOffset) % 1) {
            this.goDown(RobotConstants.goDOWNpower);
        }
        else    {
            this.stop();
        }
    }

    public boolean wentTo(double degrees)  {

        var pos = (armEncoder.get() + encoderOffset) % 1;
        var target = (degrees + rangeOffset + encoderOffset) % 1;

        if (pos > target) {
            this.goUp(RobotConstants.goUPpower);
            return false;
        }
        else if ((armEncoder.get() + encoderOffset) % 1 < (degrees - rangeOffset + encoderOffset) % 1) {
            this.goDown(RobotConstants.goDOWNpower);
            return false;
        }
        else    {
            this.stop();
            return true;
        }
    }

    public void goUp(double speed) {
        // PIDController pid = new PIDController(0, 0, 0);
        // pid.setTolerance(posTolerance, velocityTolerance);
        // var pidSpeed = pid.calculate(armEncoder.getAbsolutePosition(), setpoint);
        armMotor.set(speed);
    }

    public void goDown(double speed)    {
        armMotor.set(-speed);
    }

    public void stop()  {
        armMotor.stopMotor();
    }

    public boolean encoderCheck(double distance){
        if (armEncoder.get() == distance)  {
            return true;
        }
        return false;
    }

    @Override
    public void periodic()  {
        SmartDashboard.putNumber("Arm Encoder", (armEncoder.get()));
    }

}
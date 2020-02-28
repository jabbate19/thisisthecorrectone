package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.DrivetrainConstants.*;

import java.util.function.DoubleSupplier;

public class DrivetrainSubsystem extends SubsystemBase {

    private final PWMSparkMax left1;
    private final PWMSparkMax left2;
    private final PWMSparkMax right1;
    private final PWMSparkMax right2;

    

    public DrivetrainSubsystem() {
       

        left1 = new PWMSparkMax(kLeftMasterCANID);
        left2 = new PWMSparkMax(kLeftSlaveCANID);
        right1 = new PWMSparkMax(kRightMasterCANID);
        right2 = new PWMSparkMax(kRightSlaveCANID);
    }
       
    
    public void setLeftMotors(double speed) {
    
        left1.set(-speed);
        left2.set(-speed);
    
      }
    
      public void setRightMotors(double speed) {
      
        right1.set(speed);
        right2.set(speed);
     
    }

    /**
     * This combines both of your setMotor commands and uses DoubleSuppliers to be constantly updating for TeleOp.
     * @param left Lambda for left power
     * @param right Lambda for right power
     */
        public void setDrivePowers( DoubleSupplier left, DoubleSupplier right ){
            setLeftMotors( left.getAsDouble() );
            setRightMotors( right.getAsDouble() );
        }
}

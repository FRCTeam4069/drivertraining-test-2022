package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;

public class Drive {

    private static final int LEFT_MASTER = 6;
    private static final int LEFT_SLAVE = 5;

    private static final int RIGHT_MASTER = 2;
    private static final int RIGHT_SLAVE = 1;

    private static final double LIMIT_AMT = 0.2;
    
    private final SlewRateLimiter leftLimiter, rightLimiter;
    private final CANSparkMax leftMaster, leftSlave, rightMaster, rightSlave;

    public Drive() {
        leftMaster = new CANSparkMax(LEFT_MASTER, CANSparkMaxLowLevel.MotorType.kBrushless);
        leftSlave = new CANSparkMax(LEFT_SLAVE, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightMaster = new CANSparkMax(RIGHT_MASTER, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightSlave = new CANSparkMax(RIGHT_SLAVE, CANSparkMaxLowLevel.MotorType.kBrushless);

        rightMaster.setInverted(true);
        rightSlave.setInverted(true);   

        leftLimiter = new SlewRateLimiter(LIMIT_AMT);
        rightLimiter = new SlewRateLimiter(LIMIT_AMT);
    }

    /**
     * Sets the power of each side of the drivetrain (-1 to 1)
     * 
     * @param left Left side power
     * @param right Right side power
     */
    public void setPower(double left, double right) {
        leftMaster.set(leftLimiter.calculate(left));
        leftSlave.set(leftLimiter.calculate(left));
        rightMaster.set(rightLimiter.calculate(right));
        rightSlave.set(rightLimiter.calculate(right));
    }

    /** Stops the drivetrain */
    public void stop() {
        setPower(0, 0);
    }

    /**
     * Robot arcade drive
     * 
     * @param speed Speed of robot
     * @param turn Turn amount
     */
    public void arcadeDrive(double speed, double turn) {
        WheelSpeeds speeds = DifferentialDrive.arcadeDriveIK(speed, turn, false);
        setPower(speeds.left, speeds.right);
    }
}

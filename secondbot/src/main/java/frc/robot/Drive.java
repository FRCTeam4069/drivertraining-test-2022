package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;

public class Drive {

    private static final int LEFT_MASTER = 6;
    private static final int LEFT_SLAVE = 5;

    private static final int SHIFTER_FWD = 0;
    private static final int SHIFTER_BCK = 7;

    private static final int RIGHT_MASTER = 2;
    private static final int RIGHT_SLAVE = 1;

    private final CANSparkMax leftMaster, leftSlave, rightMaster, rightSlave;
    private final DoubleSolenoid shifter;

    private boolean highGear = true;

    public Drive() {
        leftMaster = new CANSparkMax(LEFT_MASTER, CANSparkMaxLowLevel.MotorType.kBrushless);
        leftSlave = new CANSparkMax(LEFT_SLAVE, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightMaster = new CANSparkMax(RIGHT_MASTER, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightSlave = new CANSparkMax(RIGHT_SLAVE, CANSparkMaxLowLevel.MotorType.kBrushless);

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        rightMaster.setInverted(true);
        rightSlave.setInverted(true);

        shifter = new DoubleSolenoid(PneumaticsModuleType.REVPH, SHIFTER_FWD, SHIFTER_BCK);
    }

    /**
     * Sets the power of each side of the drivetrain (-1 to 1)
     * 
     * @param left Left side power
     * @param right Right side power
     */
    public void setPower(double left, double right) {
        leftMaster.set(left);
        rightMaster.set(right);
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

    /**
     * Sets the gear to high or low
     * 
     * @param highGear True if high gear preferred
     */
    public void setGear(boolean highGear) {
        // Change gear if values differ
        if (highGear != this.highGear) changeGear();        
    }

    /** Inverts the gear state */
    public void changeGear() {
        // Flip gear state
        highGear = !highGear;

        if (highGear) shifter.set(Value.kForward);
        else shifter.set(Value.kReverse);
    }

}

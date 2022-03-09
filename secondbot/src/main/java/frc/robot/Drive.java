package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import frc.robot.Scheduler.RobotAsyncTask;

public class Drive {

    private static final int LEFT_MASTER = 6;
    private static final int LEFT_SLAVE = 5;

    // private static final int SHIFTER_FWD = 0;
    // private static final int SHIFTER_BCK = 7;

    private static final int RIGHT_MASTER = 2;
    private static final int RIGHT_SLAVE = 1;

    private static final double POWER_PER_SEC = 0.5;

    private final CANSparkMax leftMaster, leftSlave, rightMaster, rightSlave;
    // private final DoubleSolenoid shifter;

    private boolean highGear = true;

    private volatile double targetVelocityLeft, targetVelocityRight;
    private volatile boolean loop;
    private volatile long time;

    public Drive() {
        leftMaster = new CANSparkMax(LEFT_MASTER, CANSparkMaxLowLevel.MotorType.kBrushless);
        leftSlave = new CANSparkMax(LEFT_SLAVE, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightMaster = new CANSparkMax(RIGHT_MASTER, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightSlave = new CANSparkMax(RIGHT_SLAVE, CANSparkMaxLowLevel.MotorType.kBrushless);

        rightMaster.setInverted(true);
        rightSlave.setInverted(true);

        // shifter = new DoubleSolenoid(PneumaticsModuleType.REVPH, SHIFTER_FWD, SHIFTER_BCK);

        loop = true;

        new Scheduler().schedule(new RobotAsyncTask() {
            @Override
            public void run() {
                while (loop) {
                    time = System.currentTimeMillis() - time;
                    double change = (POWER_PER_SEC / 1000) * time;

                    if (targetVelocityLeft != leftMaster.get()) 
                        if (leftMaster.get() + change * sign(targetVelocityLeft) > Math.abs(leftMaster.get())) {
                            leftMaster.set(leftMaster.get() + (leftMaster.get() % (change * sign(targetVelocityLeft))));
                            leftSlave.set(leftMaster.get() + (leftMaster.get() % (change * sign(targetVelocityLeft))));
                        } else {
                            leftMaster.set(leftMaster.get() + (change * sign(targetVelocityLeft)));
                            leftSlave.set(leftMaster.get() + (change * sign(targetVelocityLeft)));
                        }

                    if (targetVelocityRight != rightMaster.get())
                        if (rightMaster.get() + change * sign(targetVelocityRight) > Math.abs(rightMaster.get())) {
                            rightMaster.set(rightMaster.get() + (rightMaster.get() % (change * sign(targetVelocityRight))));
                            rightSlave.set(rightMaster.get() + (rightMaster.get() % (change * sign(targetVelocityRight))));
                        } else {
                            rightMaster.set(rightMaster.get() + (change * sign(targetVelocityRight)));
                            rightSlave.set(rightMaster.get() + (change * sign(targetVelocityRight)));
                        }
                }
            }
        });
    }

    double sign(double x) {
        return x < 0 ? -1 : 1;
    }

    /**
     * Sets the power of each side of the drivetrain (-1 to 1)
     * 
     * @param left Left side power
     * @param right Right side power
     */
    public void setPower(double left, double right) {
        targetVelocityLeft = left;
        targetVelocityRight = right;
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

        // if (highGear) shifter.set(Value.kForward);
        // else shifter.set(Value.kReverse);
    }

}

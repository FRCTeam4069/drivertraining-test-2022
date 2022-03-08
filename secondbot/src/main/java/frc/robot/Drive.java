package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;

public class Drive {

    private static final int LEFT_MASTER = 1;
    private static final int LEFT_SLAVE = 2;

    private static final int SHIFTER_FWD = 0;
    private static final int SHIFTER_BCK = 7;

    private static final int RIGHT_MASTER = 5;
    private static final int RIGHT_SLAVE = 6;

    private static final int RIGHT_ENCODER_A = 4;
    private static final int RIGHT_ENCODER_B = 5;

    private static final int LEFT_ENCODER_A = 6;
    private static final int LEFT_ENCODER_B = 7;

    private final TalonFX leftMaster, leftSlave, rightMaster, rightSlave;
    private final Encoder leftEncoder, rightEncoder;
    private final DoubleSolenoid shifter;

    private boolean highGear = true;

    public Drive() {
        leftMaster = new TalonFX(LEFT_MASTER);
        leftSlave = new TalonFX(LEFT_SLAVE);
        rightMaster = new TalonFX(RIGHT_MASTER);
        rightSlave = new TalonFX(RIGHT_SLAVE);

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        rightMaster.setInverted(true);
        rightSlave.setInverted(true);

        leftEncoder = new Encoder(LEFT_ENCODER_A, LEFT_ENCODER_B, true, EncodingType.k1X);
        rightEncoder = new Encoder(RIGHT_ENCODER_A, RIGHT_ENCODER_B, false, EncodingType.k1X);

        shifter = new DoubleSolenoid(PneumaticsModuleType.REVPH, SHIFTER_FWD, SHIFTER_BCK);
    }

    /** Average rate between both encoders */
    public double getAvgVelocity() {
        return leftEncoder.getRate() + rightEncoder.getRate() / 2;
    }
    
    /** Rate of left encoder */
    public double getLeftVelocity() {
        return leftEncoder.getRate();
    }

    /** Rate of right encoder */
    public double getRightVelocity() {
        return rightEncoder.getRate();
    }
    
    /**
     * Sets the power of each side of the drivetrain (-1 to 1)
     * 
     * @param left Left side power
     * @param right Right side power
     */
    public void setPower(double left, double right) {
        leftMaster.set(ControlMode.PercentOutput, left);
        rightMaster.set(ControlMode.PercentOutput, right);
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

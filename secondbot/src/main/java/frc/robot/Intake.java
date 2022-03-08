package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

public class Intake {

    private static final int SPARK_ID = 13;
    private static final int PIVOT_SPARK_ID = 41;

    private final CANSparkMax intakeSpark, pivotSpark;
    private final RelativeEncoder pivotEncoder;

    private PivotPosition pivotState;

    public Intake() {
        intakeSpark = new CANSparkMax(SPARK_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        pivotSpark = new CANSparkMax(PIVOT_SPARK_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        pivotEncoder = pivotSpark.getEncoder();

        pivotSpark.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0F);
        pivotSpark.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 27F);
        pivotSpark.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        pivotSpark.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        pivotEncoder.setPosition(0);

        pivotState = PivotPosition.RETRACTED;
    }

    void setDutyCycle(double demand) {
        intakeSpark.set(demand);
    }

    void periodic() {
        if(Math.abs(pivotEncoder.getPosition() - 27) < 1.0 && pivotState == PivotPosition.EXTENDED) {
            pivotSpark.set(0.0);
        }
        if(pivotEncoder.getPosition() < 0.5 && pivotState == PivotPosition.RETRACTED) {
            pivotSpark.set(0.0);
        }
    }

    void setPivotState(PivotPosition pos) {
        pivotState = pos;

        switch (pos) {
            case RETRACTED:
                pivotSpark.set(-0.4);
                break;
            case EXTENDED:
                pivotSpark.set(0.4);
                break;
        }
    }

    public enum PivotPosition {
        EXTENDED,
        RETRACTED
    }
    
}

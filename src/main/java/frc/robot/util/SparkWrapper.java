package frc.robot.util;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

public class SparkWrapper extends CANSparkMax {
    public SparkWrapper(int deviceId, CANSparkLowLevel.MotorType type) {
        super(deviceId, type);
    }

    @Override
    public void setInverted(boolean isInverted) {
        super.setInverted(isInverted);
        for (int i = 0; i <= 5; i++) {
            if (super.getInverted() == isInverted) {
                break;
            }
        }
    }

    public SparkPIDWrapper getPIDWrapper() {
        return new SparkPIDWrapper(super.getPIDController());
    }

}

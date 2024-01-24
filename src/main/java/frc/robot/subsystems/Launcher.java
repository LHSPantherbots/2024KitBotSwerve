package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.RIO_Channels_CAN_MOTOR;

public class Launcher extends SubsystemBase {
    private final CANSparkMax m_Launcher;
    RelativeEncoder launcherEncoder;
    private final CANSparkMax m_Feeder;
    // private double lastSetpoint = 0;
    //private double setPoint = 0;
    private double launcherspeed = 0;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, allowableError;
    private SparkPIDController pidController;

    public Launcher() {
        m_Launcher = new CANSparkMax(RIO_Channels_CAN_MOTOR.LauncherMotor, MotorType.kBrushed);
        m_Feeder = new CANSparkMax(RIO_Channels_CAN_MOTOR.FeederMotor, MotorType.kBrushed);

        m_Launcher.restoreFactoryDefaults();
        m_Feeder.restoreFactoryDefaults();

        m_Launcher.setInverted(false);
        m_Feeder.setInverted(false);

        m_Launcher.setIdleMode(IdleMode.kCoast);
        m_Feeder.setIdleMode(IdleMode.kBrake);

        m_Launcher.setSmartCurrentLimit(60);
        m_Feeder.setSmartCurrentLimit(50);

        m_Launcher.setClosedLoopRampRate(0.25);

        //launcherEncoder = m_Launcher.getEncoder();

        pidController = m_Launcher.getPIDController();

        // PID coefficients these will need to be tuned
        kP = 0.00015;// 0.00025; //5e-5;
        kI = 0;// 1e-6;
        kD = 0.0008;// 0.0004;
        kIz = 0;
        kFF = 0.00017;// 0.00019;
        kMaxOutput = 1;
        kMinOutput = -1;
        maxRPM = 5700;
        allowableError = 100; // 50 //Lets the system known when the velocity is close enough to launch

        // set PID coefficients
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIz);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);
    }

    @Override
    public void periodic() {
        // Smart Dashboard Items
        // SmartDashboard.putNumber("Launcher Velocity", getLauncherVelocity());
        // SmartDashboard.putBoolean("At Set Velocity", isAtVelocity());
        // SmartDashboard.putNumber("Launcher Setpoint", getVelocitySetpoint());
    }

    // public double getVelocitySetpoint() {
    //     return setPoint;
    // }

    // public double getLauncherVelocity() {
    //     return launcherEncoder.getVelocity();
    // }

    // public boolean isAtVelocity() {
    //     double error = getLauncherVelocity() - setPoint;
    //     return (Math.abs(error) < allowableError);
    // }

    // public void setVelocitySetPoint(double vel) {
    //     setPoint = vel;
    // }

    public void StopAll() {
        launcherspeed = 0;
        m_Feeder.set(0);
        m_Launcher.set(0);
        // lastSetpoint = setPoint;
        // setPoint = 0;
        //closedLoopLaunch();
    }

    public void intake() {
        m_Feeder.set(0.25);
        m_Launcher.set(0.5);

    }

    // public void closedLoopLaunch() {
    //     pidController.setP(kP);
    //     pidController.setI(kI);
    //     pidController.setD(kD);
    //     pidController.setIZone(kI);
    //     pidController.setFF(kFF);
    //     pidController.setOutputRange(kMinOutput, kMaxOutput);

    //     pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    // }

    public void speakerCloseShot() {
        // lastSetpoint = setPoint;
        // setPoint = 2000;
        //closedLoopLaunch();
    }

    public void launcherRpmUp() {
        launcherspeed = launcherspeed-0.25;
        m_Launcher.set(launcherspeed);
        // closedLoopLaunch();

        //pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    }

    public void launcherRpmDown() {
        launcherspeed = launcherspeed+0.25;
        m_Launcher.set(launcherspeed);
        // lastSetpoint = setPoint;
        // setPoint = lastSetpoint - 250;
        // closedLoopLaunch();

        //pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    }

    public void launcherStop() {
        // lastSetpoint = setPoint;
        // setPoint = 0;
        // closedLoopLaunch();

        //pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    }

    public void launcherResume() {
        // var tmp = lastSetpoint;
        // lastSetpoint = setPoint;
        // setPoint = tmp;
        // closedLoopLaunch();

        //pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    }

    public void newIntake() {
        // lastSetpoint = setPoint;
        // setPoint = -150;
        // m_Feeder.set(-0.25);
        // pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    }

    public void newResume() {
        m_Feeder.set(0.0);
        launcherResume();
    }

    public void setLauncher(double speed) {
        m_Launcher.set(speed);
    }

    public void setFeed(double speed) {
        m_Feeder.set(speed);
    }

    public void feed() {
        // if (isAtVelocity()) {
        // m_Feeder.set(0.15);
        // }
        m_Feeder.set(-0.9);

    }
}

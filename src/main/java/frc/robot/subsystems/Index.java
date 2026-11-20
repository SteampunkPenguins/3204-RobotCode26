package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Index extends SubsystemBase{

    private SparkMax indexMotor = new SparkMax(Constants.Index.kIntakeIndexMotor, MotorType.kBrushless);

    //name of the last command that ran the Index (shown on the dashboard)
    private String lastCaller = "";

    //---------- Class Instance Creation ----------//
    public static final Index instance;

    static {
        instance = new Index();
    }

    public static Index getInstance() {
        return instance;
    }


    public Index() {

       //---------- Index Motor Configuration and Set Current Limit ----------//
        SparkMaxConfig indexConfig = new SparkMaxConfig();
        indexConfig.smartCurrentLimit(Constants.kMaxCurrent);
        indexConfig.idleMode(IdleMode.kBrake); //stops motor from moving when button is not pressed
        indexMotor.configure(indexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    //---------- Dashboard Accessors (read by Dashboard.java) ----------//

    //true while the Index motor is being told to spin (either direction)
    public boolean isRunning() {
        return indexMotor.get() != 0;
    }

    //name of the last command that called the Index with a caller name
    public String getLastCaller() {
        return lastCaller;
    }

    //---------- Teleop Index Commands ----------//

    //Hold down button to spin Index motor
    public Command spinIndex(double speed)
    {
        return this.runOnce( () -> indexMotor.set(-speed));
    }

    //Records who called the Index Motor so it shows on the dashboard
    public Command spinIndex(double speed,String caller)
    {
        return this.runOnce( () -> {
            lastCaller = caller;
            indexMotor.set(-speed);
        });
    }

    //spins Index Motor in Reverse
    public Command spinIndexReverse(double speed)
    {
        return this.runOnce( () -> indexMotor.set(speed));
    }

    //---------- Command to Set Index Motor Speed to Zero ----------//
    public Command spinIndexZero(){
        return this.runOnce(() -> indexMotor.set(0));
    }

    //---------- Auto Routine Index Commands ----------//

    //method used to set the speed of the Index during Auto continuously
    public Command spinIndexAuto(double autoSpeed)
    {
        return this.run( () -> indexMotor.set(-autoSpeed));
    }

    //Records who called the Index Motor during Auto so it shows on the dashboard
    public Command spinIndexAuto(double autoSpeed, String caller)
    {
        return this.run( () -> {
            lastCaller = caller;
            indexMotor.set(-autoSpeed);
        });
    }

}

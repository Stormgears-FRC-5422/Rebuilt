package frc.robot.subsystems.drive.tunerConstants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.drive.ctrDrive.CTRDriveInternal;

import java.lang.reflect.Method;

/**
 * A wrapper that extracts public static fields from a given ReefscapeTunerConstants–like class
 * and allows static methods to be invoked via instance calls.
 */
@SuppressWarnings("unchecked")
public class TunerConstantsWrapper {
    private final Class<?> constantsClass;

    // Public static fields from ReefscapeTunerConstants
    public final CANBus kCANBus;
    public final LinearVelocity kSpeedAt12Volts;
    public final SwerveDrivetrainConstants DrivetrainConstants;
    public final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft;
    public final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight;
    public final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft;
    public final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight;

    /**
     * Constructs the wrapper by extracting the public static fields from the specified class.
     *
     * @param constantsClass a class like ReefscapeTunerConstants
     */
    public TunerConstantsWrapper(Class<?> constantsClass) {
        this.constantsClass = constantsClass;
        try {
            this.kCANBus = (CANBus) constantsClass.getField("kCANBus").get(null);
            this.kSpeedAt12Volts = (LinearVelocity) constantsClass.getField("kSpeedAt12Volts").get(null);
            this.DrivetrainConstants = (SwerveDrivetrainConstants) constantsClass.getField("DrivetrainConstants").get(null);
            this.FrontLeft = (SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>)
                constantsClass.getField("FrontLeft").get(null);
            this.FrontRight = (SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>)
                constantsClass.getField("FrontRight").get(null);
            this.BackLeft = (SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>)
                constantsClass.getField("BackLeft").get(null);
            this.BackRight = (SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>)
                constantsClass.getField("BackRight").get(null);
        } catch (ReflectiveOperationException e) {
            throw new RuntimeException("Failed to extract ReefscapeTunerConstants fields", e);
        }
    }

    /**
     * Calls a static method on the underlying constants class.
     *
     * @param methodName the name of the static method
     * @param paramTypes the parameter types of the method
     * @param args       the arguments to pass to the method
     * @return the result of the method call
     */
    public Object callStaticMethod(String methodName, Class<?>[] paramTypes, Object... args) {
        try {
            Method method = constantsClass.getMethod(methodName, paramTypes);
            return method.invoke(null, args);
        } catch (ReflectiveOperationException e) {
            throw new RuntimeException("Failed to invoke static method: " + methodName, e);
        }
    }

    /**
     *  Ideally we need to unify the approaches for creating tha AKDrive and the CTRDrive
     *
     * @return the result of TunerConstants.createDrivetrain()
     *
     */
    public CTRDriveInternal createDrivetrain() {
        return (CTRDriveInternal) callStaticMethod("createDrivetrain", new Class<?>[]{});
    }

}

package frc.robot.elastic;
import frc.utils.configfile.Option;
import frc.utils.configfile.OptionsController;
import frc.utils.configfile.OptionsRegistry;
public class ElasticMotorController extends OptionsController {
  public final Option<Boolean> homeButton;
  public final Option<Boolean> goToIdle;
  public final Option<Boolean> rumble;
  public final Option<Boolean> intakeDown;
  public final Option<Boolean> tester;
  public final Option<Boolean> shootButton;
  public final Option<Double> shootSpeed;
  public final Option<Double> hopperSpeed;
  public final Option<Double> intakeRollerSpeed;
  public final Option<Boolean> homeIntake;
  public final Option<Boolean> visionCmd;
  public final Option<Double> intakeAngle;

  public ElasticMotorController() {
    super("Motor tester with elastic");
    homeButton = createBooleanOption("HOME THE INTAKE", false);
    goToIdle = createBooleanOption("INTAKE IDLE", false);
    rumble = createBooleanOption("RUMBLE", false);
    intakeDown = createBooleanOption("INTAKE DOWN", false);
    tester = createBooleanOption("wtv", false);
    shootButton = createBooleanOption("shoot", false);
    shootSpeed = createNumberOption("shooter speed diagnostic", 50.0);
    intakeRollerSpeed = createNumberOption("intake speed diagnostic", 0.0);
    hopperSpeed = createNumberOption("hopper speed diagnostic", 0.0);
    homeIntake = createBooleanOption("homeintake", false);
    intakeAngle = createNumberOption("angle", 150.0);
    visionCmd = createBooleanOption("vision cmd", false);
    OptionsRegistry.register(this);
  }
}

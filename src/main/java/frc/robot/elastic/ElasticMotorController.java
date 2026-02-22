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
  public final Option<Boolean> outtakeIntake;
  public final Option<Boolean> outtakeShooter;
  public final Option<Boolean> usePose;
  public final Option<Boolean> enabled;
  public final Option<Double> motor1;
  public final Option<Boolean> hopperIsEmpty;
  // public final Option<Double> motor2;
  // public final Option<String> lightState;

  public ElasticMotorController() {
    super("Motor tester with elastic");

    homeButton = createBooleanOption("HOME THE INTAKE", false);
    goToIdle = createBooleanOption("INTAKE IDLE", false);
    rumble = createBooleanOption("RUMBLE", false);
    intakeDown = createBooleanOption("INTAKE DOWN", false);
    tester = createBooleanOption("wtv", false);
    shootButton = createBooleanOption("shoot", false);
    shootSpeed = createNumberOption("shooter speed diagnostic", 0.50);
    intakeRollerSpeed = createNumberOption("intake speed diagnostic", 0.0);
    hopperSpeed = createNumberOption("hopper speed diagnostic", 0.0);
    homeIntake = createBooleanOption("homeintake", false);
    intakeAngle = createNumberOption("angle", 150.0);
    visionCmd = createBooleanOption("vision cmd", false);
    outtakeIntake = createBooleanOption("outtake intake", false);
    outtakeShooter = createBooleanOption("outtakeshooter", false);
    usePose = createBooleanOption("usePose", false);
    enabled = createBooleanOption("Enabled", false);
    motor1   = createNumberOption("Motor 1 Output", 0.0);
    hopperIsEmpty = createBooleanOption("Toggle lights tester", false);

    // lightState = createStringOption("Light State", "IDLE");
    // motor2   = createNumberOption("Motor 2 Output", 0.0);

    OptionsRegistry.register(this);
  }
}

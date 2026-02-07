package frc.robot.elastic;
import frc.utils.configfile.Option;
import frc.utils.configfile.OptionsController;
import frc.utils.configfile.OptionsRegistry;
public class ElasticMotorController extends OptionsController {
  public final Option<Boolean> enabled;
  public final Option<Double> motor1;
  public final Option<Boolean> hopperIsEmpty;
  // public final Option<Double> motor2;
  // public final Option<String> lightState;
  public ElasticMotorController() {
    super("Motor tester with elastic");
    enabled = createBooleanOption("Enabled", false);
    motor1   = createNumberOption("Motor 1 Output", 0.0);
    // lightState = createStringOption("Light State", "IDLE");
    hopperIsEmpty = createBooleanOption("Toggle lights tester", false);
    // motor2   = createNumberOption("Motor 2 Output", 0.0);
    OptionsRegistry.register(this);

  }
}
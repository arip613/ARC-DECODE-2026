package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoChooser {
  private final SendableChooser<AutoSelection> chooser = new SendableChooser<>();

  public AutoChooser() {
    SmartDashboard.putData("Autos/SelectedAuto", chooser);

    for (AutoSelection selection : AutoSelection.values()) {
      chooser.addOption(selection.toString(), selection);
    }

    chooser.setDefaultOption(AutoSelection.A_TO_B.toString(), AutoSelection.A_TO_B);
  }

  public AutoSelection getSelectedAuto() {
    return chooser.getSelected();
  }
}

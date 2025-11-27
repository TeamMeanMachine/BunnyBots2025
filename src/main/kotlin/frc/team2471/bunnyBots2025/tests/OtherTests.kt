package frc.team2471.bunnyBots2025.tests

import com.ctre.phoenix6.controls.DutyCycleOut
import edu.wpi.first.wpilibj2.command.Command
import frc.team2471.bunnyBots2025.OI
import frc.team2471.bunnyBots2025.Turret
import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.ctre.setCANCoderAngle
import org.team2471.frc.lib.units.degrees


fun turretPercentOutputTest(): Command = runCommand {
//    Turret.turretMotor.setControl(DutyCycleOut(OI.driverController))
}
fun zeroPivot(): Command = runCommand {
    Turret.pivotEncoder.setCANCoderAngle(0.0.degrees)
}
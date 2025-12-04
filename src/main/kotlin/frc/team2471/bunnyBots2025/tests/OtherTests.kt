package frc.team2471.bunnyBots2025.tests

import com.ctre.phoenix6.controls.DutyCycleOut
import edu.wpi.first.wpilibj2.command.Command
import frc.team2471.bunnyBots2025.OI
import frc.team2471.bunnyBots2025.Turret
import frc.team2471.bunnyBots2025.Turret.pivotEncoder
import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.control.commands.runOnceCommand
import org.team2471.frc.lib.ctre.setCANCoderAngle
import org.team2471.frc.lib.units.asDegrees
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.rotations


fun turretPercentOutputTest(): Command = runCommand {
//    Turret.turretMotor.setControl(DutyCycleOut(OI.driverController))
}
fun zeroPivot(): Command = runOnceCommand {
    println("Zeroing Pivot angle: ${Turret.pivotEncoderAngle.asDegrees}")
    Turret.pivotEncoderOffset = 25.0 - pivotEncoder.position.valueAsDouble.rotations.asDegrees
    println("Zeroing Pivot angle: ${Turret.pivotEncoderAngle.asDegrees}")

}
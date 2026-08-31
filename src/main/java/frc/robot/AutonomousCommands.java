// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;

/** Builds the autonomous commands displayed in the robot's auto chooser. */
public class AutonomousCommands {
    private final CommandSwerveDrivetrain drivetrain;
    private final Intake intakeSubsystem;

    public AutonomousCommands(CommandSwerveDrivetrain drivetrain, Intake intakeSubsystem) {
        this.drivetrain = drivetrain;
        this.intakeSubsystem = intakeSubsystem;
    }

    public Command buildLeftBallPathAuto() { // Starts left trench
        // Intake timing (tune these to match actual path duration)
        // Estimated total path time: ~12-15 s. Start intake at ~40%, stop at ~80%.
        final double INTAKE_START_DELAY = 5.0;  // seconds before intake turns on
        final double INTAKE_DURATION = 5.0;     // seconds to run intake
        final double INTAKE_POWER = 0.75;

        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile("leftballpickup");
            return Commands.sequence(
                Commands.runOnce(() ->
                    drivetrain.resetPose(path.getStartingHolonomicPose().orElse(new Pose2d()))
                ),
                Commands.parallel(
                    AutoBuilder.followPath(path).deadlineFor(drivetrain.run(() -> {})),
                    Commands.sequence(
                        Commands.waitSeconds(INTAKE_START_DELAY),
                        intakeSubsystem.runIntakeAutoCommand(INTAKE_POWER)
                            .withTimeout(INTAKE_DURATION)
                    )
                )
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to load path 'd': " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    public Command buildRightBallPathAuto() { // Starts right trench
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile("rightballpickup");
            return Commands.sequence(
                Commands.runOnce(() ->
                    drivetrain.resetPose(path.getStartingHolonomicPose().orElse(new Pose2d()))
                ),
                AutoBuilder.followPath(path).deadlineFor(drivetrain.run(() -> {}))
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to load path 'b': " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    public Command buildMidLeftPathAuto() { // Starts middle, goes to left trench
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile("midleftcarry");
            return Commands.sequence(
                Commands.runOnce(() ->
                    drivetrain.resetPose(path.getStartingHolonomicPose().orElse(new Pose2d()))
                ),
                AutoBuilder.followPath(path).deadlineFor(drivetrain.run(() -> {}))
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to load path 'b': " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    public Command buildMidRightPathAuto() { // Starts middle, goes to right trench
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile("midrightcarry");
            return Commands.sequence(
                Commands.runOnce(() ->
                    drivetrain.resetPose(path.getStartingHolonomicPose().orElse(new Pose2d()))
                ),
                AutoBuilder.followPath(path).deadlineFor(drivetrain.run(() -> {}))
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to load path 'b': " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    public Command buildBackPathAuto() { // Starts left trench
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile("moveback");
            return Commands.sequence(
                Commands.runOnce(() ->
                    drivetrain.resetPose(path.getStartingHolonomicPose().orElse(new Pose2d()))
                ),
                AutoBuilder.followPath(path).deadlineFor(drivetrain.run(() -> {}))
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to load path 'd': " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }
}

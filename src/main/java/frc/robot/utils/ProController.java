package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;

public class ProController {

    private final XboxController xboxController;
    private double deadzone = 0.1;  // Default deadzone for joystick axes

    // Constructor, initializing the Xbox controller
    public ProController(int port) {
        this.xboxController = new XboxController(port);
    }

    // ------------------ Button Access (A, B, X, Y) ------------------ //

    public Trigger a() {
        return new Trigger(xboxController::getAButton);
    }

    public Trigger b() {
        return new Trigger(xboxController::getBButton);
    }

    public Trigger x() {
        return new Trigger(xboxController::getXButton);
    }

    public Trigger y() {
        return new Trigger(xboxController::getYButton);
    }

    // ------------------ Bumpers ------------------

    public Trigger leftBumper() {
        return new Trigger(xboxController::getLeftBumper);
    }

    public Trigger rightBumper() {
        return new Trigger(xboxController::getRightBumper);
    }

    // ------------------ Triggers (With Axis Threshold) ------------------ //

    public Trigger leftTrigger() {
        return new Trigger(() -> xboxController.getLeftTriggerAxis() > 0.5);
    }

    public Trigger rightTrigger() {
        return new Trigger(() -> xboxController.getRightTriggerAxis() > 0.5);
    }

    // Custom trigger threshold for more control over trigger sensitivity
    public Trigger customLeftTrigger(double threshold) {
        return new Trigger(() -> xboxController.getLeftTriggerAxis() > threshold);
    }

    public Trigger customRightTrigger(double threshold) {
        return new Trigger(() -> xboxController.getRightTriggerAxis() > threshold);
    }

    // ------------------ Joystick Axes with Deadzone ------------------ //

    public double getLeftX() {
        return applyDeadzone(xboxController.getLeftX());
    }

    public double getLeftY() {
        return applyDeadzone(xboxController.getLeftY());
    }

    public double getRightX() {
        return applyDeadzone(xboxController.getRightX());
    }

    public double getRightY() {
        return applyDeadzone(xboxController.getRightY());
    }

    private double applyDeadzone(double value) {
        return Math.abs(value) < deadzone ? 0 : value;
    }

    // Set deadzone threshold for joystick axes
    public void setDeadzone(double deadzone) {
        this.deadzone = deadzone;
    }

    // ------------------ D-pad (POV) Support with Flexible Mapping ------------------ //

    public Trigger povDirection(int direction) {
        return new Trigger(() -> xboxController.getPOV() == direction);
    }

    public Trigger povUp() {
        return povDirection(0);
    }

    public Trigger povUpRight() {
        return povDirection(45);
    }

    public Trigger povRight() {
        return povDirection(90);
    }

    public Trigger povDownRight() {
        return povDirection(135);
    }

    public Trigger povDown() {
        return povDirection(180);
    }

    public Trigger povDownLeft() {
        return povDirection(225);
    }

    public Trigger povLeft() {
        return povDirection(270);
    }

    public Trigger povUpLeft() {
        return povDirection(315);
    }

    // ------------------ Command Binding Methods ------------------ //

    public void bindCommandToTrigger(Trigger trigger, Command command) {
        trigger.onTrue(command);
    }

    public void bindCommandOnRelease(Trigger trigger, Command command) {
        trigger.onFalse(command);
    }

    public void runWhileHeld(Trigger trigger, Command command) {
        trigger.whileTrue(command);
    }

    public void toggleCommand(Trigger trigger, Command command) {
        trigger.toggleOnTrue(command);
    }

    public void axisTriggerCommand(double axisValue, double threshold, Command command) {
        Trigger axisTrigger = new Trigger(() -> Math.abs(axisValue) > threshold);
        axisTrigger.onTrue(command);
    }

    // ------------------ Advanced Trigger Combinations ------------------ //

    // Combine multiple triggers (e.g., for button combos)
    public Trigger buttonCombo(Trigger... triggers) {
        return new Trigger(() -> {
            for (Trigger trigger : triggers) {
                if (!trigger.getAsBoolean()) return false;
            }
            return true;
        });
    }

    // ------------------ Rumble Support ------------------

    public void setRumble(GenericHID.RumbleType type, double value) {
        xboxController.setRumble(type, value);
    }

    public void rumblePatternSuccess() {
        setRumble(GenericHID.RumbleType.kLeftRumble, 1);
        setRumble(GenericHID.RumbleType.kRightRumble, 0.5);
    }

    public void rumblePatternWarning() {
        setRumble(GenericHID.RumbleType.kLeftRumble, 0.5);
        setRumble(GenericHID.RumbleType.kRightRumble, 1);
    }

    // ------------------ Back Paddles ------------------ //

    public Trigger backPaddleLeft() {
        return new Trigger(xboxController::getBackButton);
    }

    public Trigger backPaddleRight() {
        return new Trigger(xboxController::getStartButton);
    }

    public void toggleBackPaddleLeftCommand(Command command) {
        backPaddleLeft().toggleOnTrue(command);
    }

    public void toggleBackPaddleRightCommand(Command command) {
        backPaddleRight().toggleOnTrue(command);
    }

    public void bindBackPaddles(Command leftPaddleCommand, Command rightPaddleCommand) {
        toggleBackPaddleLeftCommand(leftPaddleCommand);
        toggleBackPaddleRightCommand(rightPaddleCommand);
    }

    // ------------------ Command on Joystick Axis Threshold ------------------ //

    public void runCommandOnLeftX(double threshold, Command command) {
        axisTriggerCommand(xboxController.getLeftX(), threshold, command);
    }

    public void runCommandOnLeftY(double threshold, Command command) {
        axisTriggerCommand(xboxController.getLeftY(), threshold, command);
    }

    public void runCommandOnRightX(double threshold, Command command) {
        axisTriggerCommand(xboxController.getRightX(), threshold, command);
    }

    public void runCommandOnRightY(double threshold, Command command) {
        axisTriggerCommand(xboxController.getRightY(), threshold, command);
    }

    // ------------------ Example: Configuring Multiple Bindings Together ------------------ //

    public void configureAdvancedBindings(Command toggleCommand, Command heldCommand) {
        toggleCommand(rightBumper(), toggleCommand);
        runWhileHeld(leftBumper(), heldCommand);
    }
}

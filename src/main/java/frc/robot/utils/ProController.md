# ProController Documentation

## Package: `frc.robot.utils`

## Class: `ProController`

A custom class for handling Xbox controller inputs with support for advanced configurations and command bindings.

### Constructors

#### `ProController(int port)`

Constructs an instance of the controller.

**Parameters:**
- `port` - The port index on the Driver Station that the controller is plugged into.

---

### Methods

#### Button Access

- **`Trigger a()`**  
  Constructs an event instance around the A button's digital signal.

- **`Trigger b()`**  
  Constructs an event instance around the B button's digital signal.

- **`Trigger x()`**  
  Constructs an event instance around the X button's digital signal.

- **`Trigger y()`**  
  Constructs an event instance around the Y button's digital signal.

- **`Trigger leftBumper()`**  
  Constructs an event instance around the left bumper's digital signal.

- **`Trigger rightBumper()`**  
  Constructs an event instance around the right bumper's digital signal.

#### Triggers

- **`Trigger leftTrigger()`**  
  Constructs a Trigger instance around the axis value of the left trigger. The returned trigger will be true when the axis value is greater than 0.5.

- **`Trigger rightTrigger()`**  
  Constructs a Trigger instance around the axis value of the right trigger. The returned trigger will be true when the axis value is greater than 0.5.

- **`Trigger customLeftTrigger(double threshold)`**  
  Constructs a Trigger instance around the axis value of the left trigger. The returned trigger will be true when the axis value is greater than the specified threshold.

  **Parameters:**
  - `threshold` - The minimum axis value for the returned Trigger to be true.

- **`Trigger customRightTrigger(double threshold)`**  
  Constructs a Trigger instance around the axis value of the right trigger. The returned trigger will be true when the axis value is greater than the specified threshold.

  **Parameters:**
  - `threshold` - The minimum axis value for the returned Trigger to be true.

#### Joystick Axes

- **`double getLeftX()`**  
  Gets the X axis value of the left side of the controller, applying the deadzone.

- **`double getLeftY()`**  
  Gets the Y axis value of the left side of the controller, applying the deadzone.

- **`double getRightX()`**  
  Gets the X axis value of the right side of the controller, applying the deadzone.

- **`double getRightY()`**  
  Gets the Y axis value of the right side of the controller, applying the deadzone.

- **`void setDeadzone(double deadzone)`**  
  Sets the deadzone threshold for joystick axes.

  **Parameters:**
  - `deadzone` - The deadzone threshold to be applied.

#### POV (D-pad) Directions

- **`Trigger povDirection(int direction)`**  
  Constructs an event instance around the specified POV direction.

  **Parameters:**
  - `direction` - The direction value of the POV (e.g., 0, 45, 90, etc.).

- **`Trigger povUp()`**  
  Constructs an event instance around the POV up direction (0 degrees).

- **`Trigger povUpRight()`**  
  Constructs an event instance around the POV up-right direction (45 degrees).

- **`Trigger povRight()`**  
  Constructs an event instance around the POV right direction (90 degrees).

- **`Trigger povDownRight()`**  
  Constructs an event instance around the POV down-right direction (135 degrees).

- **`Trigger povDown()`**  
  Constructs an event instance around the POV down direction (180 degrees).

- **`Trigger povDownLeft()`**  
  Constructs an event instance around the POV down-left direction (225 degrees).

- **`Trigger povLeft()`**  
  Constructs an event instance around the POV left direction (270 degrees).

- **`Trigger povUpLeft()`**  
  Constructs an event instance around the POV up-left direction (315 degrees).

#### Command Binding Methods

- **`void bindCommandToTrigger(Trigger trigger, Command command)`**  
  Binds a command to a trigger such that the command is activated when the trigger is true.

- **`void bindCommandOnRelease(Trigger trigger, Command command)`**  
  Binds a command to a trigger such that the command is activated when the trigger is false.

- **`void runWhileHeld(Trigger trigger, Command command)`**  
  Binds a command to a trigger such that the command is activated while the trigger is true.

- **`void toggleCommand(Trigger trigger, Command command)`**  
  Binds a command to a trigger such that the command toggles on and off when the trigger is true.

- **`void axisTriggerCommand(double axisValue, double threshold, Command command)`**  
  Binds a command to a custom axis-triggered condition.

  **Parameters:**
  - `axisValue` - The value of the axis to be checked.
  - `threshold` - The threshold value for the axis.
  - `command` - The command to be activated.

- **`Trigger buttonCombo(Trigger... triggers)`**  
  Constructs a Trigger instance that is true when all the specified triggers are true.

#### Rumble Support

- **`void setRumble(GenericHID.RumbleType type, double value)`**  
  Sets the rumble intensity for a specified rumble type.

  **Parameters:**
  - `type` - The type of rumble (e.g., left or right).
  - `value` - The intensity of the rumble (0 to 1).

- **`void rumblePatternSuccess()`**  
  Sets a rumble pattern indicating success (left rumble stronger than right rumble).

- **`void rumblePatternWarning()`**  
  Sets a rumble pattern indicating a warning (right rumble stronger than left rumble).

#### Back Paddles

- **`Trigger backPaddleLeft()`**  
  Constructs an event instance around the left back paddle button's digital signal.

- **`Trigger backPaddleRight()`**  
  Constructs an event instance around the right back paddle button's digital signal.

- **`void toggleBackPaddleLeftCommand(Command command)`**  
  Binds a command to the left back paddle such that the command toggles on and off when the left back paddle is pressed.

- **`void toggleBackPaddleRightCommand(Command command)`**  
  Binds a command to the right back paddle such that the command toggles on and off when the right back paddle is pressed.

- **`void bindBackPaddles(Command leftPaddleCommand, Command rightPaddleCommand)`**  
  Binds commands to both back paddles.

  **Parameters:**
  - `leftPaddleCommand` - The command for the left back paddle.
  - `rightPaddleCommand` - The command for the right back paddle.

#### Command on Joystick Axis Threshold

- **`void runCommandOnLeftX(double threshold, Command command)`**  
  Binds a command to the left X axis such that the command is activated when the axis value exceeds the threshold.

- **`void runCommandOnLeftY(double threshold, Command command)`**  
  Binds a command to the left Y axis such that the command is activated when the axis value exceeds the threshold.

- **`void runCommandOnRightX(double threshold, Command command)`**  
  Binds a command to the right X axis such that the command is activated when the axis value exceeds the threshold.

- **`void runCommandOnRightY(double threshold, Command command)`**  
  Binds a command to the right Y axis such that the command is activated when the axis value exceeds the threshold.

#### Example: Configuring Multiple Bindings Together

- **`void configureAdvancedBindings(Command toggleCommand, Command heldCommand)`**  
  Configures multiple command bindings together for a more complex control scheme.

  **Parameters:**
  - `toggleCommand` - The command to be toggled with the right bumper.
  - `heldCommand` - The command to be run while the left bumper is held.

---

### Inheritance

The `ProController` class does not extend any classes directly related to command-based controllers but utilizes `XboxController` for its base functionality.

### See Also

- [XboxController](https://docs.wpilib.org/en/stable/java/docs/api/edu/wpi/first/wpilibj/XboxController.html)
- [CommandGenericHID](https://docs.wpilib.org/en/stable/java/docs/api/edu/wpi/first/wpilibj2/command/button/CommandGenericHID.html)
- [Trigger](https://docs.wpilib.org/en/stable/java/docs/api/edu/wpi/first/wpilibj2/command/button/Trigger.html)
- [Command](https://docs.wpilib.org/en/stable/java/docs/api/edu/wpi/first/wpilibj2/command/Command.html)


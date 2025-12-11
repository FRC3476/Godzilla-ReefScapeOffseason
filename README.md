Code Orange's (3476) Reefscape 2025 Offseason Bot's repository

See also https://github.com/FRC3476/StreamDeck2025 for the driver station side Stream Deck code, thanks mainly to Robocubs, https://github.com/Robocubs/2024-StreamDeck.

Much of the robot code is inspired by Team 254's 2025 code at https://github.com/Team254/FRC-2025-Public.

`Field`
Information and trackers specific to the Reefscape field.

`arbitraryTriggers`
Robot triggers that are independent of human input. As highlights, these include automatic dejamming of the coral intake and some automatic state transitions.

`auto`
Setup for named commands for Pathplanner autonomous mode.

`commands`
Subclassed commands, especially autoalign and score sequence commands.

`generated`
Files from CTRE's swerve generator.

`humanControls`
Setup for controls via the driver's xbox controller, the operator's Stream Deck, and various test and debug controls available on an elastic dashboard (`dashboard/elastic-layout.json` contains an Elastic layout to use these buttons).

`subsystems`
Contains a folder for each subsystem with an IO-style structure. Contains `superstructure`, a state machine that controls the elevator and end effector. State transition paths are solved using A*. See also Team 254's implementation. Contains `vision`, which includes full pose estimation for both the robot and detected game pieces.

`util`
Contains various utility functions. As a highlight, see MotorStallDetection used for various automated robot behaviors.

`util/Controls/ElasticButton`
Convenient setup for buttons intended for use on an Elastic dashboard.

`util/Controls/StreamDeck`
Robot-side code for using a Stream Deck as a controller. See also the driver station side above.

# ChargedUp-1806
FRC 1806's code for the 2023 competition season

As of the 2023 year, Team 1806 has made the switch from "Timed Based" or "Iterative Based" programming to "Command Based" programming. As a result our robot project looks very different than previous years.

(`~/` = `src/main/java/frc/robot`)
## Notable Files
 - `~/Robot.java`: Very minimal Robot file due to command based. It contains a couple of things that we still need to run periodically such as our Shuffleboard manager
 - `~/RobotContainer.java`: Contains all the Subsystem initalizations and where we register triggers
 - `~/Constants.java`: Contains all the robot constants in a convinient place to where we can tune later.
 - `~/RobotMap.java`: All the constants for our motor ports, DIO ports, etc
 - `~/commands/`: This folder contains all the commands that allow components of our robot to run
 - `~/subsystems/`: All the subsystems on our robot which has all the major components and the driver controls
 - `~/shuffleboard/`: Contains the shuffleboard tabs and the shuffleboard manager that updates our shufflebord tabs periodically
 - `~/util/SWATXboxController.java`: Custom Xbox Controller class Team 1806 uses. It allows us to have some custom functionality we need and it allows us to tune important values to the Xbox controller on shuffleboard.

## References
 - `~/shuffleboard/`: Inspiration from 1678's C2022.
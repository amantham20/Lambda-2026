# Common FRC Reference Guides

Curated starting points for Team 5509 (Java, WPILib 2026, command-based).
Verify the year segment in URLs (`/2026`) when a new season is released.

## Official / core
- WPILib docs: https://docs.wpilib.org
- WPILib installation guide: https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html
- Zero-to-Robot (full first-time setup): https://docs.wpilib.org/en/stable/docs/zero-to-robot/introduction.html
- Command-based programming: https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html
- WPILib Java API (javadoc): https://github.wpilib.org/allwpilib/docs/release/java/index.html
- WPILib vendor dependencies: https://docs.wpilib.org/en/stable/docs/software/vscode-overview/3rd-party-libraries.html
- FRC Game Manual / Q&A / Team Updates: https://frc-events.firstinspires.org and https://www.firstinspires.org/resource-library/frc/competition-manual-qa-system
- FRC Driver Station and radio setup: https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-3/index.html
- roboRIO imaging: https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-3/imaging-your-roborio.html
- Robot Inspection (what inspectors check): https://www.firstinspires.org/robotics/frc/game-and-season

## Libraries used in this repo
- CTRE Phoenix 6: https://v6.docs.ctr-electronics.com
- Swerve (Phoenix 6 SwerveDrivetrain / Tuner X generator): https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
- PathPlanner: https://pathplanner.dev
- PhotonVision: https://docs.photonvision.org
- AdvantageKit (logging/replay): https://docs.advantagekit.org
- AdvantageScope (log viewer): https://docs.advantagescope.org
- Elastic dashboard: https://frc-elastic.gitbook.io/docs

## Programming concepts
- PID, feedforward, motion profiling: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/index.html
- Controls Engineering in FRC (Tyler Veness): https://file.tavsys.net/control/controls-engineering-in-frc.pdf
- Kinematics and odometry / pose estimation: https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/index.html
- Units library: https://docs.wpilib.org/en/stable/docs/software/basic-programming/java-units.html
- Simulation: https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/introduction.html
- Unit testing: https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/unit-testing.html
- SysId (characterization): https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/index.html
- Java learning (Oracle tutorials): https://dev.java/learn/

## Vision and autonomous
- AprilTags / pose estimation with PhotonVision: https://docs.photonvision.org/en/latest/docs/programming/photonlib/index.html
- WPILib AprilTag field layouts: https://docs.wpilib.org/en/stable/docs/software/vision-processing/apriltag/apriltag-intro.html
- PathPlanner autos and pathfinding: https://pathplanner.dev/home.html
- Limelight (alternative): https://docs.limelightvision.io

## Electrical, mechanical, CAD, strategy
- WPILib wiring guide (robot): https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-1/index.html
- Wiring best practices (CTRE): https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/index.html
- FRC Design: https://www.chiefdelphi.com (forum, build threads, white papers)
- The Blue Alliance (match data/scouting): https://www.thebluealliance.com
- Statbotics (EPA metrics): https://www.statbotics.io
- Onshape for FRC (free CAD): https://www.onshape.com/en/education/frc
- FRC Discord/Reddit-style communities: Chief Delphi is the canonical archive.

## Team workflow
- Git basics: https://git-scm.com/book
- GitHub Flow: https://docs.github.com/en/get-started/using-github/github-flow
- WPILib on GitHub issues: https://github.com/wpilibsuite/allwpilib
- AI coding assistants: see ADMIN_ALLOWLIST.md for the hosts needed; always review generated code and test in simulation before deploying to a robot.

## This repo, quick commands
```
./gradlew build            # compile + test
./gradlew simulateJava     # desktop simulation
./gradlew deploy           # deploy to roboRIO (on robot network)
./gradlew build --offline  # after dependencies are cached
```

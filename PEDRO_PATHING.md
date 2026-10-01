# Pedro Pathing setup

This repository is wired for Pedro Pathing 3.0.0 with the Pedro tuning procedures
from the official Quickstart. `build.dependencies.gradle` supplies the Dairy Maven
repository and the `revhub`/`tuning` dependencies; the starter source lives under
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro`.

The constants match the current Control Hub configuration:

- drive motors: `FLeft`, `FRight`, `BLeft`, `BRight`
- goBILDA Pinpoint: `odo`

The Pinpoint offsets are intentionally `0.0` placeholders and the drive directions
are conventional starter values. Run the tuners before pathing, then use their
measured output to replace the corresponding values in `Constants.java`.

## Tuning order

1. Deploy and select **Pedro: Hardware Smoke Test**. It initializes the follower and
   reports the Pinpoint pose without commanding movement.
2. Run **Mecanum Tuner** and verify each wheel direction. Keep the robot safely lifted
   or in a clear area: tuning procedures drive the motors.
3. Run **Pinpoint Tuner** to determine pod directions, offsets, and resolution.
4. Run the localization/odometry checks from **Tests**.
5. Run **Foresight Tuner**, then the remaining **Tests** before writing autonomous paths.

The Pedro AutoTune dashboard is normally available at
`http://192.168.43.1:10158` while the Control Hub is connected over Wi-Fi. Deploy
from Android Studio by selecting the TeamCode run configuration and your Control Hub, then clicking Run. Build without deploying from a terminal with:

```powershell
.\gradlew.bat :TeamCode:assembleDebug
```

Do not run a moving tuner or autonomous OpMode until the robot is secured and there
is enough space around it.

## Shared project

Clone https://github.com/B3Codin/FtcRobotController.git on each computer and open
the cloned project folder in Android Studio. This project uses FTC SDK 12.0.0,
Pedro Pathing 3.0.0, tuning 1.0.0, and Panels 1.0.13.
Install Android SDK Platform 34 if Android Studio requests it during sync. Let Android Studio configure each computer's local Android SDK path; local.properties
is intentionally excluded from Git. Pull before editing, commit and push your changes,
and have the deployment computer pull before installing TeamCode on the robot.
Panels is available at http://192.168.43.1:8001 while connected to the Hub Wi-Fi.

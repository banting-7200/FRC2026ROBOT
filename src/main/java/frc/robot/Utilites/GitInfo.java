package frc.robot.Utilites;

import frc.robot.BuildConstants;

public class GitInfo {

  public static final String ANSI_RESET = "\u001B[0m";
  public static final String ANSI_BLACK = "\u001B[30m";
  public static final String ANSI_RED = "\u001B[31m";
  public static final String ANSI_GREEN = "\u001B[32m";
  public static final String ANSI_YELLOW = "\u001B[33m";
  public static final String ANSI_BLUE = "\u001B[34m";
  public static final String ANSI_PURPLE = "\u001B[35m";
  public static final String ANSI_CYAN = "\u001B[36m";
  public static final String ANSI_WHITE = "\u001B[37m";
  public static final String ANSI_BLACK_BACKGROUND = "\u001B[40m";
  public static final String ANSI_RED_BACKGROUND = "\u001B[41m";
  public static final String ANSI_GREEN_BACKGROUND = "\u001B[42m";
  public static final String ANSI_YELLOW_BACKGROUND = "\u001B[43m";
  public static final String ANSI_BLUE_BACKGROUND = "\u001B[44m";
  public static final String ANSI_PURPLE_BACKGROUND = "\u001B[45m";
  public static final String ANSI_CYAN_BACKGROUND = "\u001B[46m";
  public static final String ANSI_WHITE_BACKGROUND = "\u001B[47m";

  /**
   * Print out the Build Constants, so we know which code is running on the robot at any given time
   *
   * <p>We need to use @SuppressWarnings because the compiler complains when we compare the constant
   * BuildConstants.DIRTY to the same value it maybe already has. The compiler doesn't know this
   * constant is dynamically generated.
   */
  @SuppressWarnings("all")
  public static void printGitInfo() {

    String uncommittedCode =
        (BuildConstants.DIRTY == 1)
            ? "\n"
                + ANSI_RED_BACKGROUND
                + "WARNING: The robot is running uncomitted code!"
                + ANSI_RESET
            : "";

    String gitInfo =
        String.format(
            "-------- Git Info --------\n"
                + "Project:\t%s\n"
                + "Build date:\t%s\n"
                + "Built on:\t%s\n"
                + "Git Repository:\t%s\n"
                + "Git Branch:\t%s\n"
                + "Git Hash:\t%s\n"
                + "Git Date:\t%s\n"
                + "Last Committer:\t%s%s\n"
                + "--------------------------",
            BuildConstants.MAVEN_NAME,
            BuildConstants.BUILD_DATE,
            BuildConstants.HOSTNAME,
            BuildConstants.GIT_REMOTE,
            BuildConstants.GIT_BRANCH,
            BuildConstants.GIT_SHA,
            BuildConstants.GIT_DATE,
            BuildConstants.GIT_COMMITER,
            uncommittedCode);

    System.out.println(gitInfo);
  }
}

package frc.robot.utils;

public class CubicRanger implements Ranger {

  // THESE NUMBERS ARE ALL COMPLETELY WRONG
  private final int[][] KNOWN_POINTS = new int[][] {
      { 3490, 0, 5 },
      { 3400, 10, 10 },
      { 3730, 20, 15 },
      { 3900, 30, 18 },
      { 4090, 40, 20 },
      { 4100, 30, 23 },
      { 4390, 20, 25 }
  };

  private final int RPM_COLUMN = 0; // column index for RPM values
  private final int ANGLE_COLUMN = 1; // column index for angle values
  private final int DISTANCE_COLUMN = 2; // column index for distance values

  public FiringSolution getFiringSolution(double range) {

    return new FiringSolution(0, 0.0);
  }

  public int calculateRPM(double distance, boolean rpm) {
    if (distance < KNOWN_POINTS[ANGLE_COLUMN][DISTANCE_COLUMN]) {
      return (cubicInterpolation(0, 1, 2, 3, distance, rpm));
    }
    if (distance < KNOWN_POINTS[ANGLE_COLUMN + 1][DISTANCE_COLUMN]) {
      return (cubicInterpolation(0, 1, 2, 3, distance, rpm));
    }
    if (distance >= KNOWN_POINTS[KNOWN_POINTS.length - 1][DISTANCE_COLUMN]) {
      return cubicInterpolation(KNOWN_POINTS.length, KNOWN_POINTS.length - 1, KNOWN_POINTS.length - 2,
          KNOWN_POINTS.length - 3, distance, rpm);
    }
    for (int i = 0; i < KNOWN_POINTS.length - 1; i++) {
      if (distance == KNOWN_POINTS[i][DISTANCE_COLUMN]) {
        if (rpm) {
          return KNOWN_POINTS[i][RPM_COLUMN];
        }
        return KNOWN_POINTS[i][ANGLE_COLUMN];
      } else if (distance < KNOWN_POINTS[i + 1][DISTANCE_COLUMN]) {
        return (cubicInterpolation(i - 2, i - 1, i, i + 1, distance, rpm));
      }
    }
    if (rpm) {
      return KNOWN_POINTS[KNOWN_POINTS.length - 3][RPM_COLUMN];
    }
    return KNOWN_POINTS[KNOWN_POINTS.length - 3][ANGLE_COLUMN];
  }

  public FiringSolution getFiringSolution(String name)
  {
      return null;
  }
  
  private int cubicInterpolation(int a, int b, int c, int d, double distance, boolean rpm) {
    // solve A(distance) + B(distance) + C(distance) + D(distance) = y
    // return y

    double[] distances = { KNOWN_POINTS[a][DISTANCE_COLUMN], KNOWN_POINTS[b][DISTANCE_COLUMN],
        KNOWN_POINTS[c][DISTANCE_COLUMN], KNOWN_POINTS[d][DISTANCE_COLUMN], distance };
    double[] rpms = { KNOWN_POINTS[a][RPM_COLUMN], KNOWN_POINTS[b][RPM_COLUMN], KNOWN_POINTS[c][RPM_COLUMN],
        KNOWN_POINTS[d][RPM_COLUMN], distance };
    if (!rpm) {
      rpms[0] = KNOWN_POINTS[a][ANGLE_COLUMN];
      rpms[1] = KNOWN_POINTS[b][ANGLE_COLUMN];
      rpms[2] = KNOWN_POINTS[c][ANGLE_COLUMN];
      rpms[3] = KNOWN_POINTS[d][ANGLE_COLUMN];
    }
    double[][] matrix1 = {
        { (distances[0] * distances[0] * distances[0]), (distances[0] * distances[0]), distances[0], 1 },
        { (distances[1] * distances[1] * distances[1]), (distances[1] * distances[1]), distances[1], 1 },
        { (distances[2] * distances[2] * distances[2]), (distances[2] * distances[2]), distances[2], 1 },
        { (distances[3] * distances[3] * distances[3]), (distances[3] * distances[3]), distances[3], 1 }
    };

    System.out.print(matrix1[1][1]);
    double[][] matrix2 = {
        { rpms[0] },
        { rpms[1] },
        { rpms[2] },
        { rpms[3] }
    };

    Matrix m1 = new Matrix(matrix1);
    Matrix m2 = new Matrix(matrix2);

    double[] abcd = m1.solve(m2);

    double y = abcd[0] * (distance * distance * distance) + abcd[1] * (distance * distance) + abcd[2] * distance
        + abcd[3];

    return (int) y;
  }
}

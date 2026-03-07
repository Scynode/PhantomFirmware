/*
 * trajectory.cpp
 * Trajectory planning for chess piece moves.
 *
 * COORDINATE SYSTEM:
 * ───────────────────
 * The physical coordinate system places the board centre at (0, 0) mm:
 *   X increases toward column 9 (right side)
 *   Y increases toward row 0 (top side)
 *   Each square is 50mm × 50mm
 *   Board extents: X ∈ [-225, +225], Y ∈ [-225, +225]
 *
 * Grid-to-mm conversion:
 *   X_mm = 50 * col - 225
 *   Y_mm = -50 * row + 225
 *
 * TRAJECTORY TYPES (movementType()):
 * ─────────────────────────────────────
 *   0 → Direct: the path between start and end is clear (no pieces in the way).
 *       A straight line of interpolated waypoints is generated.
 *
 *   1 → Between-lines: pieces block a straight path (or the move is a knight move).
 *       Two or three Bézier curve segments with straight-line connectors are used
 *       to route the magnet around the occupied squares.
 *
 * BEZIER CURVES:
 * ───────────────
 * Each curve is a quadratic Bézier with the midpoint of the knight-step segment
 * as the control point.  The first and last control points are snapped 25mm
 * toward the central control point so the curve starts and ends along the
 * segment direction.
 *
 * ELECTROMAGNET OFFSETS (calculateOffsets()):
 * ────────────────────────────────────────────
 * The four electromagnets are physically offset from the carriage centre:
 *   1 (upper-left):  X+20.58mm, Y±0
 *   2 (lower-left):  X+10.54mm, Y−39.96mm
 *   3 (upper-right): X−25.42mm, Y±0
 *   4 (lower-right): X+10.54mm, Y+39.96mm
 * These offsets are added to all waypoints before inverse kinematics so that
 * the magnet (not the carriage centre) passes over the correct board square.
 */

#include "../../include/scultpureMode.h"
#include <Arduino.h>
#include "../../include/config.h"
#include "mech_state.h"
#include "../sensors/sensor_state.h"

// ─────────────────────────────────────────────────────────────────────────────
// calculateOffsets()
// Returns the physical XY offset (mm) from the carriage centre to the
// specified electromagnet's effective pickup point.
//
// activeElectro:  1=upper-left, 2=lower-left, 3=upper-right, 4=lower-right
// ─────────────────────────────────────────────────────────────────────────────
void calculateOffsets(int activeElectro, float &electroOffsetX, float &electroOffsetY)
{
    switch (activeElectro)
    {
    case 1:
        electroOffsetX = 20.58;
        electroOffsetY = 0;
        break;
    case 2:
        electroOffsetY = -39.96;
        electroOffsetX = 10.54;
        break;
    case 3:
        electroOffsetX = -25.42;
        electroOffsetY = 0;
        break;
    case 4:
        electroOffsetY = 39.96;
        electroOffsetX = 10.54;
        break;
    default:
        electroOffsetX = 0;
        electroOffsetY = 0;
        break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// bezierCurvePoints()
// Computes a quadratic Bézier curve through sizeArray points.
//
// The three control points (P1, P2, P3) define the curve shape.  Before
// computing, P1 and P3 are snapped 25mm toward P2 so the curve blends
// smoothly into the straight-line segments that connect the curves.
//
// Results are written into arrayTestX[] and arrayTestY[].
// ─────────────────────────────────────────────────────────────────────────────
void bezierCurvePoints(double xP1, double yP1, double xP2, double yP2, double xP3, double yP3, double *arrayTestX, double *arrayTestY, int sizeArray)
{
    // Snap P1 and P3 25mm toward P2 for smooth curve entry/exit
    xP1 = (xP2 > xP1) ? xP2 - 25 : ((xP2 < xP1) ? xP2 + 25 : xP1);
    yP1 = (yP2 > yP1) ? yP2 - 25 : ((yP2 < yP1) ? yP2 + 25 : yP1);
    xP3 = (xP2 > xP3) ? xP2 - 25 : ((xP2 < xP3) ? xP2 + 25 : xP3);
    yP3 = (yP2 > yP3) ? yP2 - 25 : ((yP2 < yP3) ? yP2 + 25 : yP3);
    for (int i = 0; i < sizeArray; i++)
    {
        double t = i / (double)(sizeArray - 1); // Distribute t values uniformly between 0 and 1
        // Apply the Bézier equation to calculate points on the curve
        arrayTestX[i] = (1 - t) * (1 - t) * xP1 + 2 * (1 - t) * t * xP2 + t * t * xP3;
        arrayTestY[i] = (1 - t) * (1 - t) * yP1 + 2 * (1 - t) * t * yP2 + t * t * yP3;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// interpolatePoints()
// Creates a set of uniformly-spaced waypoints along the straight line from
// (x1, y1) to (x2, y2).
//
// The number of points is chosen so that adjacent waypoints are approximately
// distanceBetweenPoints mm apart.  A minimum of 2 points is always returned.
//
// Returns a heap-allocated float[numPoints][2] array; the caller is responsible
// for freeing it.
// ─────────────────────────────────────────────────────────────────────────────
float **interpolatePoints(float x1, float y1, float x2, float y2, int &numPoints)
{
    // Calculate the hypotenuse
    float hypotenuse = std::sqrt(std::pow((x2 - x1), 2) + std::pow((y2 - y1), 2));

    // Round the number of points to the nearest integer
    numPoints = static_cast<int>(hypotenuse / distanceBetweenPoints + 0.5f);

    // Ensure numPoints is at least 2 to avoid division by zero
    if (numPoints < 2)
    {
        numPoints = 2;
    }

    // Create a dynamic matrix on the heap
    float **interpolatedPoints = new float *[numPoints];
    for (int i = 0; i < numPoints; i++)
    {
        interpolatedPoints[i] = new float[2];
    }

    // Perform the interpolation
    for (int i = 0; i < numPoints; i++)
    {
        float t = static_cast<float>(i) / (numPoints - 1);
        interpolatedPoints[i][0] = x1 + (x2 - x1) * t;
        interpolatedPoints[i][1] = y1 + (y2 - y1) * t;
    }

    return interpolatedPoints;
}

// ─────────────────────────────────────────────────────────────────────────────
// movementType()
// Determines whether a move can be executed as a straight trajectory (0) or
// requires an around-the-pieces detour trajectory (1).
//
// A direct move (type 0) is possible when:
//   - The move is diagonal AND all intervening diagonal squares are empty, OR
//   - The move is along a rank AND all intervening rank squares are empty, OR
//   - The move is along a file AND all intervening file squares are empty.
//
// Any other move (e.g. knight L-shape) always returns type 1.
// ─────────────────────────────────────────────────────────────────────────────
int movementType(int squareRowInit, int squareColInit, int squareRowEnd, int squareColEnd)
{
    if (squareRowInit - squareRowEnd == 0 || squareColInit - squareColEnd == 0 || abs(squareRowInit - squareRowEnd) == abs(squareColInit - squareColEnd)) // If the movement is in a straight line or diagonal
    {
        // moveType = 0 -> direct movement
        // moveType = 1 -> between-line movement
        if (abs(squareRowInit - squareRowEnd) == abs(squareColInit - squareColEnd)) // If the movement is in a diagonal
        {
            int minX = min(squareRowInit, squareRowEnd);
            int maxX = max(squareRowInit, squareRowEnd);
            int emptyCount = 0;
            detectChessBoard(sensorMatrixSc);
            for (int x = minX + 1; x < maxX; x++) // Explore the diagonal from the initial square to the final square
            {
                int y = -1;
                if (squareRowEnd > squareRowInit)
                {
                    if (squareColEnd > squareColInit) // bottom-right
                    {
                        y = squareColInit + (x - squareRowInit);
                    }
                    else // top-right
                    {
                        y = squareColInit - (x - squareRowInit);
                    }
                }
                else
                {
                    if (squareColEnd > squareColInit) // bottom-left
                    {
                        y = squareColInit + (squareRowInit - x);
                    }
                    else // top-left
                    {
                        y = squareColInit - (squareRowInit - x);
                    }
                }

                if (sensorMatrixSc[x][y] == 1) // && matriztomovement[x][y] == '.')
                {
                    emptyCount++;
                }
            }

            if (emptyCount == (maxX - minX) - 1)
            {
                return 0;
            }
            else
            {
                return 1;
            }
        }
        else if (squareRowInit - squareRowEnd == 0) // If the movement is in a straight line in the Y axis
        {
            int minY = min(squareColInit, squareColEnd);
            int maxY = max(squareColInit, squareColEnd);
            int emptyCount = 0;
            int x = squareRowInit;
            for (int y = minY + 1; y < maxY; y++)
            {

                if (sensorMatrixSc[x][y] == 1) // && matriztomovement[x][y] == '.')
                {
                    emptyCount++;
                }
            }
            if (emptyCount == (maxY - minY) - 1)
            {
                return 0;
            }
            else
            {
                return 1;
            }
        }
        else if (squareColInit - squareColEnd == 0) // If the movement is in a straight line in the X axis
        {
            int minX = min(squareRowInit, squareRowEnd);
            int maxX = max(squareRowInit, squareRowEnd);
            int emptyCount = 0;
            int y = squareColEnd;
            for (int x = minX + 1; x < maxX; x++)
            {
                if (sensorMatrixSc[x][y] == 1) // && matriztomovement[x][y] == '.')
                {
                    emptyCount++;
                }
            }
            if (emptyCount == (maxX - minX) - 1)
            {
                return 0;
            }
            else
            {
                return 1;
            }
        }
    }
    else // If the movement is not in a straight line or diagonal
    {
        return 1;
    }
    return 1;
}

// ─────────────────────────────────────────────────────────────────────────────
// moveOnTheLinev2Sc()
// Computes the intermediate control points for a between-line (around-pieces)
// trajectory and populates finalPointsArray with the Bézier curve samples.
//
// The algorithm selects an intermediate waypoint 25mm off the direct path
// to route the magnet around pieces.  The direction of the detour is chosen
// based on the relative positions of the start and end squares.
//
// Two cases for the detour geometry:
//   diffX > diffY: primary motion is horizontal → detour vertically first
//   diffX ≤ diffY: primary motion is vertical   → detour horizontally first
//
// Special cases (same row/column, single-square moves, equal-distance moves)
// reduce the number of curve segments from 3 to 2.
//
// Returns totalCurves (2 or 3) via reference.
// ─────────────────────────────────────────────────────────────────────────────
void moveOnTheLinev2Sc(double xIni, double yIni, double xFin, double yFin, int &totalCurves, double finalPointsArray[][2])
// Returns the total number of points sent.
{
    //=======Intermediate points for knight trajectory=========
    float diffX;
    float diffY;
    float interPointX;
    float interPointY;

    int vectInterPointsX[5] = {-1, -1, -1, -1, -1};
    int vectInterPointsY[5] = {-1, -1, -1, -1, -1};

    /*======================================TRAJECTORY CONDITIONS==========================================*/

    diffX = abs(xIni - xFin); // distance traveled in X
    diffY = abs(yIni - yFin); // distance traveled in Y

    vectInterPointsX[0] = xIni;
    vectInterPointsY[0] = yIni;

    if (diffX > diffY) // If the distance in X is greater than the distance in Y
    {
        if (yFin < yIni) // Electromagnet moving from top to bottom
        {
            interPointX = xIni;
            interPointY = yIni - 25;

            vectInterPointsX[1] = interPointX;
            vectInterPointsY[1] = interPointY;
        }
        if (yFin > yIni) // Electromagnet moving from bottom to top
        {
            interPointX = xIni;
            interPointY = yIni + 25;

            vectInterPointsX[1] = interPointX;
            vectInterPointsY[1] = interPointY;
        }

        if (xFin < xIni) // Electromagnet moving from right to left
        {
            interPointX = xFin + 25;

            vectInterPointsX[2] = interPointX;
            vectInterPointsY[2] = interPointY;
        }
        if (xFin > xIni) // Electromagnet moving from left to right
        {
            interPointX = xFin - 25;

            vectInterPointsX[2] = interPointX;
            vectInterPointsY[2] = interPointY;
        }
        interPointY = yFin; //

        vectInterPointsX[3] = interPointX;
        vectInterPointsY[3] = interPointY;
    }
    else // If the distance in Y is greater than the distance in X
    {
        if (xFin < xIni) // Electromagnet moving from right to left
        {
            interPointX = xIni - 25;
            interPointY = yIni;

            vectInterPointsX[1] = interPointX;
            vectInterPointsY[1] = interPointY;
        }
        if (xFin > xIni) // Electromagnet moving from left to right
        {
            interPointX = xIni + 25;
            interPointY = yIni;

            vectInterPointsX[1] = interPointX;
            vectInterPointsY[1] = interPointY;
        }
        if (yFin < yIni) // Electromagnet moving from top to bottom
        {
            interPointY = yFin + 25;

            vectInterPointsX[2] = interPointX;
            vectInterPointsY[2] = interPointY;
        }
        if (yFin > yIni) // Electromagnet moving from bottom to top
        {
            interPointY = yFin - 25;

            vectInterPointsX[2] = interPointX;
            vectInterPointsY[2] = interPointY;
        }
        interPointX = xFin;

        vectInterPointsX[3] = interPointX;
        vectInterPointsY[3] = interPointY;
    }

    vectInterPointsX[4] = xFin;
    vectInterPointsY[4] = yFin;

    if (diffX < diffY)
    {
        if (diffX == 50)
        {
            vectInterPointsX[2] = vectInterPointsX[1];
            vectInterPointsY[2] = yFin;

            vectInterPointsX[3] = xFin;
            vectInterPointsY[3] = yFin;

            vectInterPointsX[4] = -1;
            vectInterPointsY[4] = -1;
        }
        if (diffX == 0)
        {
            // The case where they are in the same column of the board
            if (xIni > 0) // This condition avoids the motors
            {
                vectInterPointsX[1] = xIni - 25;
                vectInterPointsY[1] = yIni;

                vectInterPointsX[2] = vectInterPointsX[1];
                vectInterPointsY[2] = yFin;

                vectInterPointsX[3] = xFin;
                vectInterPointsY[3] = yFin;

                vectInterPointsX[4] = -1;
                vectInterPointsY[4] = -1;
            }
            else // The original implementation only included the following
            {
                vectInterPointsX[1] = xIni + 25;
                vectInterPointsY[1] = yIni;

                vectInterPointsX[2] = vectInterPointsX[1];
                vectInterPointsY[2] = yFin;

                vectInterPointsX[3] = xFin;
                vectInterPointsY[3] = yFin;

                vectInterPointsX[4] = -1;
                vectInterPointsY[4] = -1;
            }
        }
    }

    if (diffX > diffY)
    {
        if (diffY == 50)
        {
            vectInterPointsX[2] = xFin;
            vectInterPointsY[2] = vectInterPointsY[1];

            vectInterPointsX[3] = xFin;
            vectInterPointsY[3] = yFin;

            vectInterPointsX[4] = -1;
            vectInterPointsY[4] = -1;
        }
        if (diffY == 0)
        {
            // The case where they are in the same column of the board
            if (yIni > 0) // This condition avoids the motors
            {
                vectInterPointsX[1] = xIni;
                vectInterPointsY[1] = yIni - 25;

                vectInterPointsX[2] = xFin;
                vectInterPointsY[2] = vectInterPointsY[1];

                vectInterPointsX[3] = xFin;
                vectInterPointsY[3] = yFin;

                vectInterPointsX[4] = -1;
                vectInterPointsY[4] = -1;
            }
            else // The original implementation only included the following
            {
                vectInterPointsX[1] = xIni;
                vectInterPointsY[1] = yIni + 25;

                vectInterPointsX[2] = xFin;
                vectInterPointsY[2] = vectInterPointsY[1];

                vectInterPointsX[3] = xFin;
                vectInterPointsY[3] = yFin;

                vectInterPointsX[4] = -1;
                vectInterPointsY[4] = -1;
            }
        }
    }
    if (diffX == 50 && diffY == 50)
    {
        if (xIni < xFin)
        {
            vectInterPointsX[1] = xIni + 25;
            vectInterPointsY[1] = yIni;

            vectInterPointsX[2] = vectInterPointsX[1];
            vectInterPointsY[2] = yFin;

            vectInterPointsX[3] = xFin;
            vectInterPointsY[3] = yFin;

            vectInterPointsX[4] = -1;
            vectInterPointsY[4] = -1;
        }
        else
        {
            vectInterPointsX[1] = xIni - 25;
            vectInterPointsY[1] = yIni;

            vectInterPointsX[2] = vectInterPointsX[1];
            vectInterPointsY[2] = yFin;

            vectInterPointsX[3] = xFin;
            vectInterPointsY[3] = yFin;

            vectInterPointsX[4] = -1;
            vectInterPointsY[4] = -1;
        }
    }
    /*======================================TRAJECTORY CONDITIONS==========================================*/
    double pointsArrayX[totalPointsInCurve] = {0};
    double pointsArrayY[totalPointsInCurve] = {0};

    if (vectInterPointsX[4] == -1 || vectInterPointsY[4] == -1) // If there is no curve endpoint
    {
        totalCurves = 2;
        bezierCurvePoints(vectInterPointsX[0], vectInterPointsY[0], vectInterPointsX[1], vectInterPointsY[1], vectInterPointsX[2], vectInterPointsY[2], pointsArrayX, pointsArrayY, totalPointsInCurve);
        for (int i = 0; i < totalPointsInCurve; i++)
        {
            finalPointsArray[i][0] = pointsArrayX[i];
            finalPointsArray[i][1] = pointsArrayY[i];
        }
        bezierCurvePoints(vectInterPointsX[1], vectInterPointsY[1], vectInterPointsX[2], vectInterPointsY[2], vectInterPointsX[3], vectInterPointsY[3], pointsArrayX, pointsArrayY, totalPointsInCurve);
        for (int i = 0; i < totalPointsInCurve; i++)
        {
            finalPointsArray[i + totalPointsInCurve][0] = pointsArrayX[i];
            finalPointsArray[i + totalPointsInCurve][1] = pointsArrayY[i];
        }
    }
    else
    {
        totalCurves = 3;
        bezierCurvePoints(vectInterPointsX[0], vectInterPointsY[0], vectInterPointsX[1], vectInterPointsY[1], vectInterPointsX[2], vectInterPointsY[2], pointsArrayX, pointsArrayY, totalPointsInCurve);
        for (int i = 0; i < totalPointsInCurve; i++)
        {
            finalPointsArray[i][0] = pointsArrayX[i];
            finalPointsArray[i][1] = pointsArrayY[i];
        }
        bezierCurvePoints(vectInterPointsX[1], vectInterPointsY[1], vectInterPointsX[2], vectInterPointsY[2], vectInterPointsX[3], vectInterPointsY[3], pointsArrayX, pointsArrayY, totalPointsInCurve);
        for (int i = 0; i < totalPointsInCurve; i++)
        {
            finalPointsArray[i + totalPointsInCurve][0] = pointsArrayX[i];
            finalPointsArray[i + totalPointsInCurve][1] = pointsArrayY[i];
        }
        bezierCurvePoints(vectInterPointsX[2], vectInterPointsY[2], vectInterPointsX[3], vectInterPointsY[3], vectInterPointsX[4], vectInterPointsY[4], pointsArrayX, pointsArrayY, totalPointsInCurve);
        for (int i = 0; i < totalPointsInCurve; i++)
        {
            finalPointsArray[i + totalPointsInCurve * 2][0] = pointsArrayX[i];
            finalPointsArray[i + totalPointsInCurve * 2][1] = pointsArrayY[i];
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// generateTrajectory()
// Top-level trajectory planner: converts a chess move (grid coordinates) into
// a dense array of physical XY waypoints ready for accelRampV3().
//
// Steps:
//  1. Determine move type (0=direct, 1=around-pieces).
//  2. Generate curve points (Bézier or straight-line interpolation).
//  3. Append an "overshoot" extension beyond the target square by generalOffset
//     mm so the piece is fully centred before the magnet is cut.
//
// Returns a heap-allocated float[numPointsFinal][2] array.  The caller must
// free each row and the outer array when done.
// ─────────────────────────────────────────────────────────────────────────────
float **generateTrajectory(int squareRowInit, int squareColInit, int squareRowEnd, int squareColEnd, int &numPointsFinal)
{

    int totalCurves = 0;
    int numPointsCurve = 0;
    int numPoints = 0; // Will be updated based on the hypotenuse
    int numPointsAux = 0;
    int numPointsAux2 = 0;
    double curvesArray[totalPointsInCurve * 3][2];
    float **interpolatedPoints = nullptr;
    float **interpolatedPoints2 = nullptr;
    float **interpolatedPoints3 = nullptr;
    float **finalTrajectory = nullptr;
    // 0,0 = -225, 225 9,9 = 225, -225
    int coordXInit = (50 * squareRowInit) - 225;
    int coordYInit = (-50 * squareColInit) + 225;

    int coordXEnd = (50 * squareRowEnd) - 225;
    int coordYEnd = (-50 * squareColEnd) + 225;

    int moveType = movementType(squareRowInit, squareColInit, squareRowEnd, squareColEnd);

    printf("%s X%d Y%d -> X%d Y%d\n", (moveType == 0) ? "Direct" : "Between Lines", squareRowInit, squareColInit, squareRowEnd, squareColEnd);
    const float generalOffset = BleChess.getOffsetPieces();

    //   ==============================================================trajectory generators ==============================================================
    //   -------------------------------------------------------------------Direct Movement --------------------------------------------------------------------
    if (moveType == 0) // direct movement
    {

        interpolatedPoints = interpolatePoints(coordXInit, coordYInit, coordXEnd, coordYEnd, numPoints); // Returns a matrix with interpolated points and also sets numPoints.

        numPointsFinal = numPoints + (generalOffset / distanceBetweenPoints); // the +1 accounts for the value added at the end of the trajectory

        finalTrajectory = new float *[numPointsFinal];
        for (int i = 0; i < numPointsFinal; i++)
        {
            finalTrajectory[i] = new float[2];
        }
        for (int i = 0; i < numPoints; i++)
        {
            finalTrajectory[i][0] = interpolatedPoints[i][0];
            finalTrajectory[i][1] = interpolatedPoints[i][1];
        }
    }
    // -------------------------------------------------------------------Direct Movement --------------------------------------------------------------------

    // -----------------------------------------------------------------Between-Line Movement --------------------------------------------------------------------
    else if (moveType == 1) // between-line movement
    {
        moveOnTheLinev2Sc(coordXInit, coordYInit, coordXEnd, coordYEnd, totalCurves, curvesArray); // Returns an integer numPoints with the total number of points (16 or 24) and an curvesArray array with the point coordinates

        numPointsCurve = totalCurves * totalPointsInCurve;

        if (totalCurves == 2) // short between-line movement with 2 curves
        {
            interpolatedPoints = interpolatePoints(curvesArray[totalPointsInCurve - 1][0], curvesArray[totalPointsInCurve - 1][1], curvesArray[totalPointsInCurve][0], curvesArray[totalPointsInCurve][1], numPoints); // get the points from the end of the first curve to the start of the second curve

            numPointsFinal = numPoints + numPointsCurve + (generalOffset / distanceBetweenPoints); // the +1 accounts for the value added at the end of the trajectory

            finalTrajectory = new float *[numPointsFinal];
            for (int i = 0; i < numPointsFinal; i++)
            {
                finalTrajectory[i] = new float[2];
            }

            for (int i = 0; i < totalPointsInCurve; i++)
            {
                finalTrajectory[i][0] = curvesArray[i][0];
                finalTrajectory[i][1] = curvesArray[i][1];
            }
            for (int i = 0; i < numPoints; i++)
            {
                finalTrajectory[i + totalPointsInCurve][0] = interpolatedPoints[i][0];
                finalTrajectory[i + totalPointsInCurve][1] = interpolatedPoints[i][1];
            }
            for (int i = 0; i < totalPointsInCurve; i++)
            {
                finalTrajectory[i + totalPointsInCurve + numPoints][0] = curvesArray[i + totalPointsInCurve][0];
                finalTrajectory[i + totalPointsInCurve + numPoints][1] = curvesArray[i + totalPointsInCurve][1];
            }
        }

        if (totalCurves == 3) // long between-line movement with 3 curves
        {
            interpolatedPoints = interpolatePoints(curvesArray[totalPointsInCurve - 1][0], curvesArray[totalPointsInCurve - 1][1], curvesArray[totalPointsInCurve][0], curvesArray[totalPointsInCurve][1], numPoints);                         // get the points from the end of the first curve to the start of the second curve
            interpolatedPoints2 = interpolatePoints(curvesArray[(totalPointsInCurve * 2) - 1][0], curvesArray[(totalPointsInCurve * 2) - 1][1], curvesArray[totalPointsInCurve * 2][0], curvesArray[totalPointsInCurve * 2][1], numPointsAux); // get the points from the end of the second curve to the start of the third curve

            numPointsFinal = numPoints + numPointsAux + numPointsCurve + (generalOffset / distanceBetweenPoints); // the +1 accounts for the value added at the end of the trajectory

            finalTrajectory = new float *[numPointsFinal];
            for (int i = 0; i < numPointsFinal; i++)
            {
                finalTrajectory[i] = new float[2];
            }

            for (int i = 0; i < totalPointsInCurve; i++)
            {
                finalTrajectory[i][0] = curvesArray[i][0];
                finalTrajectory[i][1] = curvesArray[i][1];
            }
            for (int i = 0; i < numPoints; i++)
            {
                finalTrajectory[i + totalPointsInCurve][0] = interpolatedPoints[i][0];
                finalTrajectory[i + totalPointsInCurve][1] = interpolatedPoints[i][1];
            }
            for (int i = 0; i < totalPointsInCurve; i++)
            {
                finalTrajectory[i + totalPointsInCurve + numPoints][0] = curvesArray[i + totalPointsInCurve][0];
                finalTrajectory[i + totalPointsInCurve + numPoints][1] = curvesArray[i + totalPointsInCurve][1];
            }
            for (int i = 0; i < numPointsAux; i++)
            {
                finalTrajectory[i + totalPointsInCurve + numPoints + totalPointsInCurve][0] = interpolatedPoints2[i][0];
                finalTrajectory[i + totalPointsInCurve + numPoints + totalPointsInCurve][1] = interpolatedPoints2[i][1];
            }
            for (int i = 0; i < totalPointsInCurve; i++)
            {
                finalTrajectory[i + totalPointsInCurve + numPoints + totalPointsInCurve + numPointsAux][0] = curvesArray[i + totalPointsInCurve * 2][0];
                finalTrajectory[i + totalPointsInCurve + numPoints + totalPointsInCurve + numPointsAux][1] = curvesArray[i + totalPointsInCurve * 2][1];
            }
        }
    }
    // -----------------------------------------------------------------Between-Line Movement --------------------------------------------------------------------
    int numPointsAdded = generalOffset / distanceBetweenPoints;       // Number of points to add at the end of the trajectory
    int lastPointIndex = numPointsFinal - numPointsAdded - 1;        // Index of the last point
    int penultimatePointIndex = numPointsFinal - numPointsAdded - 2; // Index of the second-to-last point

    float xLast = finalTrajectory[lastPointIndex][0];
    float yLast = finalTrajectory[lastPointIndex][1];
    float xPenultimate = finalTrajectory[penultimatePointIndex][0];
    float yPenultimate = finalTrajectory[penultimatePointIndex][1];

    float theta = atan2(yLast - yPenultimate, xLast - xPenultimate);

    float xNew = xLast + generalOffset * cos(theta);
    float yNew = yLast + generalOffset * sin(theta);

    if (coordXEnd > 200 || coordXEnd < -200 || coordYEnd > 200 || coordYEnd < -200)
    {
        // Graveyard move: just repeat the last point rather than extending further
        for (int i = 0; i < numPointsAdded; i++)
        {
            finalTrajectory[i + numPointsFinal - numPointsAdded][0] = xLast;
            finalTrajectory[i + numPointsFinal - numPointsAdded][1] = yLast;
        }
    }

    else
    {
        // Normal move: extend the trajectory slightly past the target square
        interpolatedPoints3 = interpolatePoints(xLast, yLast, xNew, yNew, numPointsAux2); // Returns a matrix with interpolated points and also sets numPoints.

        for (int i = 0; i < numPointsAux2; i++)
        {
            finalTrajectory[i + numPointsFinal - numPointsAdded][0] = interpolatedPoints3[i][0];
            finalTrajectory[i + numPointsFinal - numPointsAdded][1] = interpolatedPoints3[i][1];
        }
    }

    // ----------------------------------------------------------centering offset compensation----------------------------------------------------------------
    if (interpolatedPoints != nullptr)
    {
        for (int i = 0; i < numPoints; i++)
        {
            delete[] interpolatedPoints[i];
        }
        delete[] interpolatedPoints;
        interpolatedPoints = nullptr; // Avoid dangling pointer
    }
    if (interpolatedPoints2 != nullptr)
    {
        for (int i = 0; i < numPointsAux; i++)
        {
            delete[] interpolatedPoints2[i];
        }
        delete[] interpolatedPoints2;
        interpolatedPoints2 = nullptr;
    }
    if (interpolatedPoints3 != nullptr)
    {
        for (int i = 0; i < numPointsAux2; i++)
        {
            delete[] interpolatedPoints3[i];
        }
        delete[] interpolatedPoints3;
        interpolatedPoints3 = nullptr;
    }

    // ==============================================================trajectory generators ==============================================================

    return finalTrajectory;
}

#pragma once
#include "Arduino.h"
#include "FS.h"

void sculptureMain();

void centerInitialPiecesSc(int, char[10][10]);
void reorderChessboardPlus(int mode, char targetMatrixAux[10][10], char currentMatrix[10][10]);
int findNearestEmptyPosition(int, int, char[10][10]);
int findNearestPiecePosition(int, int, char, char[10][10]);

void sensorsDir(void);
void updateSensors();
void readRawSensors(bool[10][10]);
void detectChessBoard(bool[10][10]);
void sensorOffsetCalib();
int compareMatrixVsSensorsPlus(int, char [10][10]);

void setCurrentPosition(float pos1, float pos2);
String decodeMovement(char, char, char, char, char, char, char [10][10], bool);
void simplifiedMovement(int, int, int, int, char[10][10]);
int movementType(int squareRowInit, int squareRowEnd, int squareColInit, int squareColEnd);
float **generateTrajectory(int coordXInit, int coordYInit, int coordXEnd, int coordYEnd, int &numPointsFinal);
float **interpolatePoints(float x1, float y1, float x2, float y2, int &numPoints);
void moveOnTheLinev2Sc(double, double, double, double, int &, double[][2]);
void bezierCurvePoints(double, double, double, double, double, double, double *, double *, int);

void accelRampV3(float **finalTrajectory, int numPointsFinal, double speed);
void rawMovement(float rowEnd, float colEnd, int magnet, float &, float &);
void rawMovementStallGuard(float rowEnd, float colEnd, float speed, int mode, int &threshold, int &thresholdD2);
void configAndCurrentPosManager(int setSpeedbyUser, float &driverMicroSteps);
//void reInitVariables(void);
void activateElectromagnetV2(int optionElectro, int power);
void deactivateAllMagnets();
void calculateOffsets(int activeElectro, float &electroOffsetX, float &electroOffsetY);

void mechanicalCalibration(int calibType);


void sensorsCalibration(int rowEnd, int colEnd, int mode, float &totalX, float &totalY);
void configDrivers();
bool testDrivers();


void IRAM_ATTR moveRawStallGuard();
void IRAM_ATTR timeEnabled();

void printGenericMatrix(char[][10], int, int);
String readFromFileSc(int);
//String decodeChessMove(char movChess[7][250], int moveCount);
String decodeChessMove(char movChess[7], int moveCount);
//void guardarMatriz(char[][10]);
//void leerMatriz(char[][10]);

void soundHandler(int);
//int batteryCheck(int mode);

void initMatrizPlus(char matrixToInit[10][10]);
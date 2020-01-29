#pragma once

// sensor/actuator units construct a 2-layer neural network
// to learn the positions of actuator units

//これらの値や関数をユニットのclassが参照できるようにする

/************* constants ***************/
#define LNum 500000	//学習上限回数
//const int outNum = DoF;	//出力層のユニット数
//#define HidNum 3	//中間層のユニット数
//const int inNum = DoF;	//入力層のユニット数
//#define PatternNum 4	//パターン数(XORは4通り)
#define alpha 0.2	//重み用学習係数
#define beta 0.2	//閾値用学習係数
#define aSig 0.5	//シグモイド関数のゲイン
#define aTanh 1.0	//Tanh関数のゲイン
#define bTanh 0.0	//Tanh関数のゲイン
//#define V_ini 0.3	//V初期値の絶対値
//#define W_ini 0.3	//W初期値の絶対値
//#define gamma_ini 5.0	//gamma初期値の絶対値
//#define theta_ini 5.0	//theta初期値の絶対値
#define w_ini 0.01 //重み係数の初期値範囲
#define b_ini 0.01 //閾値の初期値範囲


float sigmoid(float x);

float tanh_NN(float x);

// for sensor units
float inputXweight(float input, float weight);

float returnReward_sigmoid(float A_before, float A_after, int attachedNum);

float returnReward_tanh(float A_before, float A_after, int attachedNum);

void updateWeight(float &weight, float sensorValue, float dEdb);

//for actuator units(estimate reward unit)
float calc_forward(float(&fromSensors)[DoF], float &bias);

float calc_dEdb_sigmoid(float y, float t);

float calc_dEdb_tanh(float y, float t);

void updateBias(float &bias, float dEdb);


#pragma once
#include "Calculation.h"


#define DoF 5	//number of DoF
#define VIBMODE 2	//specify vibration mode for simulation here(from 1 to DoF)1

#define MASS 1.5      //[kg]
#define DAMPER 1.7    //[N s/m]
#define SPRING 4800.0 //[N/m]

// external force
#define SUPERPOSED false	// if you want to use single sinusoidal wave, turn this into "false"

/* !LEARN時に変更しなくていいように改良したい */

const int waveNum = 5;
const float freqSet[waveNum] = {
	// must be in ascending order for "setInputNum()" in SensorUnit class
	// "waveNum" elements needed
	// plantのインスタンスが生成された後にp.natural_freqを入れられるようにしたい．

	//2.5
	//,
	2.56257
	,
	//3.5
	//,
	5.02
	,
	7.4801
	,
	//7.5
	//,
	9.64
	,
	//11.7
	//,
	11.7916
};

//const int waveNum = 3;
//const float freqSet[waveNum] = {
//	// must be in ascending order for "setInputNum()" in SensorUnit class
//	// "waveNum" elements needed
//	// plantのインスタンスが生成された後にp.natural_freqを入れられるようにしたい．
//
//	2.5
//	,
//	7.5
//	,
//	11.7
//
//};

const float length = 0.35; //各質点間の距離
const float MinofMax_value = 0.003;	// [m] Minimum of Max displacement(condition of simulation termination) 

const float steadyStateTime = 90.0; //確実に定常状態になりそうな時間

class Plant
{
public:
  Plant(float m, float c, float k); // constructor
  ~Plant();                         // destructor

  // variables
  Eigen::VectorXf x, vel, accel, modeAmp; //modeAmp: amplitude of each DoF at VIBMODE
  std::vector<float> m; //メンバ変数宣言の時点では空で持っとくのが普通？
  std::vector<float> c;
  std::vector<float> k;
  MatrixXf M = MatrixXf::Zero(DoF, DoF);
  MatrixXf C = MatrixXf::Zero(DoF, DoF);
  MatrixXf K = MatrixXf::Zero(DoF, DoF);
  ArrayXf natural_freq;
  MatrixXcf natural_modes;
  float simFreq;	//simulation frequency
  float simT;	//simulation period
  Eigen::VectorXf MinofMax_vector;
  int vibTimes;
  bool vibTimesChanged;

  Eigen::VectorXf biggestDisps;

	//external forces (at ground)
  float y;
  float y_0;
  float vy;
  float ay;

  // functions
  void M_matrix(MatrixXf& M);
  void C_matrix(MatrixXf& C);
  void K_matrix(MatrixXf& K);
  Eigen::ArrayXf calc_natural_freq(MatrixXf& M, MatrixXf& K);
  //std::vector<Eigen::VectorXcf> calc_natural_modes(MatrixXf& M, MatrixXf& K);
  Eigen::MatrixXcf calc_natural_modes(MatrixXf& M, MatrixXf& K);
  void set_simFreq();
  void cal_vibTimes(float time);
  void getBiggestDisps();
  void update_external(float t);
};

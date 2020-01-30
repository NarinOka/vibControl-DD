#pragma once
#include "Calculation.h"

// forward declaration of classes
//reference: https://repeatedly.hatenadiary.org/entry/20081011/1223667807
//			 https://qiita.com/shuheilocale/items/31923586ab495217742a
class ActUnit;
class SensorUnit;

const int actUnitNum = 3;	
const int sensorUnitNum = DoF + 1; // at ground and each DoF
const int workActUnitNum = 3;	// 使わないように!

/* common variables for sensor units */
const int getAmp_steps = 150;	// get biggestDisplacement within this value
//static std::vector<float> biggestDisps(DoF, 0.0);	// vector of biggest displacements of all DoF


/* common variables for actuator units */
const int addUnitCoef = 0;	//add actuator units at every addUnitCoef * getAmp_steps

/* Vibration control method */
const bool useDD = true; //false -> skyhook



/*******************************************/
/****************Sensor Unit****************/
/*******************************************/
//センサ個体のクラスであり，群れ(集合体)のクラスではないことに注意
//Plantから値を獲得し，Actuatorに送る
class SensorUnit
{

public:
	SensorUnit();	//default constructor
	SensorUnit(int ID, Plant& p);
	~SensorUnit();

	// variables
	int sensorID;
	float acceleration;
	float acceleration_past;
	float velocity;
	float velocity_past;
	float displacement;
	float displacement_past;
	int readDoF_pos;
	
	float biggestDisp;
	float biggestDisp_past; //→input_past
	int countSteps;

	// functions
	void readAcceleration(Plant& p);	// acceleration sensor
	void cal_velocity(float dt);	// dtという情報は渡してしまっていいのか(オラクル？)
	void cal_displacement(float dt);
	std::tuple<float, bool> getBiggestDisp_steps();// tupleじゃなくて↓2つでよいかも
	void getBiggestDisp();
	bool checkSteps();

	/*** for learning ***/
	std::vector<float> weights;
	float input;
	float input_past;
	int whatNumatDoF;
	float bias;
	float dEdb;

	float calc_inXwei(int ID);
	float calc_reward();
	void changeInputValue();


	//after 12.24
	float output; // for sensor 1-DoF
	float normalizedBiggestDisp;
	void teach_sysID(SensorUnit &s);
	void calc_dEdb_sensor();
	void calc_output(SensorUnit &s);	// sensor 1-DoF
	void updateWeight_sensor(SensorUnit(&sensorUnits)[sensorUnitNum]);
	void updateBias_sensor();
	float calc_error();
	// sensor 0 memory
	std::vector<vector<float>> weights_s0;	//for sensor 0
								// [inputNum][DoF]
	float inputWaveRange; // [s]
	float input_dt; // [s]
	int inputNum;
	std::vector<float> inputValues;	// element num = inputNum
	std::vector<vector<float>> inXwei;	//for sensor 0
	std::vector<index_value> sortedBiggestDisps;	//for normalizing teach signals
	std::vector<index_value> sortedNNoutputs;	//for normalizing teach signals
	
	void setInputNum();
	void setInputValues(int inputIndex);
	void calc_inXwei();	// sensor 0
	void sort_biggestDisps(SensorUnit(&sensorUnits)[sensorUnitNum]);
	void sort_NNoutputs(SensorUnit(&sensorUnits)[sensorUnitNum]);

};


/*********************************************/
/****************Actuator Unit****************/
/*********************************************/
//アクチュエータ個体のクラスであり，群れ(集合体)のクラスではないことに注意
//Sensorから値をもらい，Plantへ制御出力する
class ActUnit
{

public:
	ActUnit();
	ActUnit(int ID);
	~ActUnit();

	// variables
	int currentPos;
	float force;
	float biggestDispofAll; //各センサのMax値を集め，その中のMaxの値を代入
	bool check_step;	//sensorから情報を受け取ったか
	Eigen::VectorXf biggestDisps;	// vector of biggest displacements of all DoF
										//plantの値ではなくsensorの読み取り値なのでここで宣言
	std::vector<index_value> in_disp;	//indexとdisplacementのpair配列(vector)
	float dampingCoef;	//instead of force

	int whatNumAtDoF;	//for display 
	int actID;

	// functions
	int calcDistance(SensorUnit& s);
	int getNeighborSensorID(SensorUnit(&sensorUnits)[sensorUnitNum]);
	float outputForce(SensorUnit& s);
	int judgeBiggestPos(SensorUnit(&sensorUnits)[sensorUnitNum]);	// 不使用
	void moveTo(int DoFpos);
	void getEachBiggestDisp(SensorUnit(&sensorUnits)[sensorUnitNum]);

	/*** dynamic damper ***/
	float m_DD;
	float k_DD;
	float c_DD;
	void attachDD(Plant& p, Eigen::VectorXf& f1);

	/*** leader ***/
	// 2019.12.19 abolish leader(still some values and functions can be used)
	std::vector<int> attachedNum_max;
	std::vector<int> attachedNum_now;
	std::vector<float> expectedRewards;
	std::vector<index_value> in_reward;

};



// 配列の参照渡し
// https://pknight.hatenablog.com/entry/20100317/1268796690
// 範囲for分
// https://cpprefjp.github.io/lang/cpp11/range_based_for.html
// C++:基底クラスの配列から派生クラスのメソッドを呼び出す
// https://qiita.com/Complex/items/dc723e6cc236963990c0
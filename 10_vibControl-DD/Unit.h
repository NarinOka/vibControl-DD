#pragma once
#include "Calculation.h"

// forward declaration of classes
//reference: https://repeatedly.hatenadiary.org/entry/20081011/1223667807
//			 https://qiita.com/shuheilocale/items/31923586ab495217742a
class ActUnit;
class SensorUnit;
class RewardUnit;

const int actUnitNum = 3;	
const int sensorUnitNum = DoF + 1; // at ground and each DoF
const int rewardUnitNum = DoF;
const int workActUnitNum = 3;	// �g��Ȃ��悤��!

/* common variables for sensor units */
const int getAmp_steps = 150;	// get biggestDisplacement within this value
//static std::vector<float> biggestDisps(DoF, 0.0);	// vector of biggest displacements of all DoF


/* common variables for actuator units */
const int addUnitCoef = 0;	//add actuator units at every addUnitCoef * getAmp_steps



/*******************************************/
/****************Sensor Unit****************/
/*******************************************/
//�Z���T�̂̃N���X�ł���C�Q��(�W����)�̃N���X�ł͂Ȃ����Ƃɒ���
//Plant����l���l�����CActuator�ɑ���
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
	float biggestDisp_past; //��input_past
	int countSteps;

	// functions
	void readAcceleration(Plant& p);	// acceleration sensor
	void cal_velocity(float dt);	// dt�Ƃ������͓n���Ă��܂��Ă����̂�(�I���N���H)
	void cal_displacement(float dt);
	std::tuple<float, bool> getBiggestDisp_steps();// tuple����Ȃ��ā�2�ł悢����
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
	void updateWeight_sensor(ActUnit(&actUnits)[actUnitNum]);
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
//�A�N�`���G�[�^�̂̃N���X�ł���C�Q��(�W����)�̃N���X�ł͂Ȃ����Ƃɒ���
//Sensor����l�����炢�CPlant�֐���o�͂���
class ActUnit
{

public:
	ActUnit();
	ActUnit(int ID);
	~ActUnit();

	// variables
	int currentPos;
	float force;
	float biggestDispofAll; //�e�Z���T��Max�l���W�߁C���̒���Max�̒l����
	bool check_step;	//sensor��������󂯎������
	Eigen::VectorXf biggestDisps;	// vector of biggest displacements of all DoF
										//plant�̒l�ł͂Ȃ�sensor�̓ǂݎ��l�Ȃ̂ł����Ő錾
	std::vector<index_value> in_disp;	//index��displacement��pair�z��(vector)
	float dampingCoef;	//instead of force

	int whatNumAtDoF;	//for display 
	int actID;

	// functions
	int calcDistance(SensorUnit& s);
	int getNeighborSensorID(SensorUnit(&sensorUnits)[sensorUnitNum]);
	float outputForce(SensorUnit& s);
	int judgeBiggestPos(SensorUnit(&sensorUnits)[sensorUnitNum]);	// �s�g�p
	void moveTo(int DoFpos);
	void getEachBiggestDisp(SensorUnit(&sensorUnits)[sensorUnitNum]);
	float outputDampingCoef();


	/*** for learning ***/
	float expectedReward; // output of network
	float bias;
	float dEdb;
	bool isInNetwork;

	void calc_output(SensorUnit(&sensorUnits)[sensorUnitNum]);
	void calc_dEdb_act(SensorUnit s);
	void updateBias_act();


	/*** leader ***/
	// 2019.12.19 abolish leader(still some values and functions can be used)
	std::vector<int> attachedNum_max;
	std::vector<int> attachedNum_now;
	std::vector<float> expectedRewards;
	std::vector<index_value> in_reward;

	void collectRewards(ActUnit(&actUnits)[actUnitNum]);
	void sort_inreward();

};


/*******************************************/
/****************Reward Unit****************/
/*******************************************/
class RewardUnit
{

public:
	RewardUnit();	//default constructor
	RewardUnit(int ID);
	~RewardUnit();

	/*** for learning ***/
	// variables
	int rewardID;
	float bias;
	float expectedReward; // output of network
	float dEdb;

	// functions
	void calc_output_rwd(SensorUnit(&sensorUnits)[sensorUnitNum]);
	void calc_dEdb_rwd(SensorUnit s);
	void updateBias_rwd();

};

// �z��̎Q�Ɠn��
// https://pknight.hatenablog.com/entry/20100317/1268796690
// �͈�for��
// https://cpprefjp.github.io/lang/cpp11/range_based_for.html
// C++:���N���X�̔z�񂩂�h���N���X�̃��\�b�h���Ăяo��
// https://qiita.com/Complex/items/dc723e6cc236963990c0
#include "Calculation.h"
#include "definedHeaders.h"



/*******************************************/
/****************Sensor Unit****************/
/*******************************************/
SensorUnit::SensorUnit()
{
	//default constructor
	// cout << "this is defalt constructor" << endl;
}


SensorUnit::SensorUnit(int ID, Plant& p)
{
	//IDは(各DoF-1)の値であることに注意
	////sensorIDは各DoFであることに注意
	this->sensorID = ID;
	// cout << this->displacement << endl;	//debug

	if (this->sensorID < DoF + 1) this->readDoF_pos = this->sensorID;
	else {
		//自由度数よりもユニットが多ければランダムに配置
		this->readDoF_pos = rnd(0, DoF);
	}

	this->acceleration_past = 0.0;
	this->acceleration = 0.0;
	this->readAcceleration(p);
	this->velocity_past = 0.0;
	this->displacement_past = 0.0;
	this->velocity = 0.0;
	this->displacement = 0.0;

	this->biggestDisp = 0.0;
	this->biggestDisp_past = 0.0;
	this->countSteps = 0;


	/*** for learning ***/
	this->input = 0.0;
	this->input_past = 0.0;
	this->whatNumatDoF = 0;
	this->bias = rnd(-b_ini, b_ini);
	this->dEdb = 0.0;
	this->output = 0.0;
	

	if (this->readDoF_pos == 0) {
		this->setInputNum();
		this->inputValues.resize(this->inputNum);
		this->weights_s0
			= std::vector<vector<float>>(this->inputNum, vector<float>(DoF, rnd(-w_ini, w_ini)));
		this->inXwei
			= std::vector<vector<float>>(this->inputNum, vector<float>(DoF, 0));
		
		this->velocity = p.y_0 * 2 * M_PI * p.simFreq;	//ここは要検討

		this->sortedBiggestDisps.resize(DoF);
		this->sortedNNoutputs.resize(DoF);

		// 2次元配列の初期化： https://qiita.com/alchemist/items/6cd2a86db7377ad8d236
	}

}


SensorUnit::~SensorUnit()
{
}


// 加速度センサ用
void SensorUnit::readAcceleration(Plant& p)
{
	this->acceleration_past = this->acceleration;

	if (this->readDoF_pos == 0) { // ground
		this->acceleration = p.ay;
	}
	else {
		this->acceleration = p.accel[this->readDoF_pos - 1];
	}

}



void SensorUnit::cal_velocity(float dt)
{
	this->velocity_past = this->velocity;
	
	//矩形則
	//this->velocity = this->velocity_past + this->acceleration * dt;

	//台形則
	if (this->readDoF_pos == 0)
		this->velocity = this->velocity_past
			+ (this->acceleration + this->acceleration_past) * dt / 2;
	else
		this->velocity = this->velocity_past
			+ (this->acceleration + this->acceleration_past) * dt / 2;



}


void SensorUnit::cal_displacement(float dt)
{
	this->displacement_past = this->displacement;
	
	//矩形則
	//this->displacement = this->displacement_past + this->velocity * dt;

	//台形則
	this->displacement = this->displacement_past
		+ (this->velocity + this->velocity_past) * dt / 2;

}


std::tuple<float, bool> SensorUnit::getBiggestDisp_steps()
{	// https://qiita.com/dfukunaga/items/e2375369d5b280e4a939
	// tuple<float(biggestDisp), bool(countSteps reached getAmp_steps)>
	float biggestDispforReturn;

	if (this->biggestDisp < this->displacement) {
		this->biggestDisp = this->displacement;
	}

	if (this->countSteps > getAmp_steps) {
		biggestDispforReturn = this->biggestDisp;
		this->countSteps = 0;
		this->biggestDisp = 0.0;

		//cout << "getBiggestDisp_steps" << endl;
		//cout << "sensor biggestDisp resetted" << endl;

		return{ biggestDispforReturn, true };
	}
	else {
		this->countSteps++;
		biggestDispforReturn = this->biggestDisp;

		return{ biggestDispforReturn, false };	//とりあえず値も返す
	}
}


void SensorUnit::getBiggestDisp()
{

	if (this->biggestDisp < this->displacement) {
		this->biggestDisp = this->displacement;
	}
}


bool SensorUnit::checkSteps()
{
	if (this->countSteps % getAmp_steps == 0) {
		// cout << "checkSteps" << endl;
		return true;
	}
	else return false;

}


/*** for learning ***/
float SensorUnit::calc_inXwei(int ID)
{
	float value;
	// for (float w : this->weights) {
	// value = inputXweight(this->input, w);
	value = inputXweight(this->input, this->weights[ID - 1]);
	// }


	return value;
}

float SensorUnit::calc_reward() 
{// sensor 1-DoF

	return returnReward_sigmoid(this->input, this->biggestDisp, this->whatNumatDoF);

}


void SensorUnit::updateWeight_sensor(SensorUnit(&sensorUnits)[sensorUnitNum])
{// sensor 0
	for (int in = 0; in < this->inputNum; in++) {
		for (int dof = 0; dof < DoF; dof++) {
			//updateWeight(this->weights_s0[in][dof], 
			//	this->inputValues[in], sensorUnits[dof + 1].dEdb);
			updateWeight(this->weights_s0[in][dof],
				sigmoid(this->inputValues[in]), sensorUnits[dof + 1].dEdb);
		}
	}
}


void SensorUnit::changeInputValue()
{// 使わない
	this->input_past = this->input;
	this->input = this->biggestDisp;
}

void SensorUnit::teach_sysID(SensorUnit &s)
{// sensor 1-DoF *** 引数にはsensor 0
	// return sigmoid(this->biggestDisp);	//normalized by sigmoid
	this->normalizedBiggestDisp 
		= this->biggestDisp / s.sortedBiggestDisps[0].second;
}


void SensorUnit::calc_dEdb_sensor()
{
	this->dEdb = calc_dEdb_sigmoid(this->output, this->normalizedBiggestDisp);
}

void SensorUnit::calc_output(SensorUnit &s)
{// 引数はsensor 0
	float value = 0.0;

	for (int i = 0; i < s.inputNum; i++) {
		value += s.inXwei[i][this->sensorID - 1];
	}
	value += this->bias;

	this->output = sigmoid(value);
}

void SensorUnit::updateBias_sensor()
{
	updateBias(this->bias, this->dEdb);
}

float SensorUnit::calc_error()
{// sensor 1-DoF
	return (this->normalizedBiggestDisp - this->output);
}

void SensorUnit::setInputNum()
{// for sensor 0
	// set "inputWaveRange", "input_dt", and "inputNum"
	this->inputWaveRange = 2.0 * 1.0 / freqSet[0];	// 2 periods of smallest freq
	this->input_dt = 1.0 / freqSet[waveNum - 1] / 10.2;	// devide periods of biggest freq into 5.5
	this->inputNum = this->inputWaveRange / this->input_dt;


	cout << "inputWaveRange = " << this->inputWaveRange << endl;
	cout << "input_dt = " << this->input_dt << endl;
	cout << "inputNum = " << this->inputNum << endl;
		
}

void SensorUnit::setInputValues(int inputIndex)
{// for sensor 0
	//this->inputValues[inputIndex] = this->displacement;
	this->inputValues[inputIndex] = this->acceleration;	//この値が結構大きい
}

void SensorUnit::calc_inXwei()
{// for sensor 0
	for (int in = 0; in < this->inputNum; in++) {
		for (int dof = 0; dof < DoF; dof++) {
			//this->inXwei[in][dof] = this->inputValues[in] * this->weights_s0[in][dof];
			this->inXwei[in][dof] = sigmoid(this->inputValues[in]) * this->weights_s0[in][dof];
			//inputvalueの値が大きく，そのままだと学習がうまくできないため
		}
	}
}

void SensorUnit::sort_biggestDisps(SensorUnit(&sensorUnits)[sensorUnitNum])
{// for sensor 0
	for (int i = 0; i < this->sortedBiggestDisps.size(); i++) {
		this->sortedBiggestDisps[i] 
			= index_value(i + 1, sensorUnits[i + 1].biggestDisp);
	}

	std::sort(this->sortedBiggestDisps.begin(), 
		this->sortedBiggestDisps.end(), greaterPair);
}

void SensorUnit::sort_NNoutputs(SensorUnit(&sensorUnits)[sensorUnitNum])
{// for sensor 0
	for (int i = 0; i < this->sortedNNoutputs.size(); i++) {
		this->sortedNNoutputs[i]
			= index_value(i + 1, sensorUnits[i + 1].output);
	}

	std::sort(this->sortedNNoutputs.begin(),
		this->sortedNNoutputs.end(), greaterPair);
}


/*********************************************/
/****************Actuator Unit****************/
/*********************************************/
ActUnit::ActUnit()
{
	//default constructor
}

ActUnit::ActUnit(int ID)
{
	//initial position of an actuator unit.
	//select actual DoF number(not DoF - 1).
	//this->currentPos = rnd(1, DoF);
	this->currentPos = 0;	// at ground

	this->force = 0.0;
	this->biggestDispofAll = 0.0;
	this->check_step = false;
	this->biggestDisps = Eigen::VectorXf::Zero(DoF);
	this->in_disp.resize(DoF);
	// this->dampingCoef = 5.0 * M_PI;
	this->dampingCoef = 15.0;

	this->whatNumAtDoF = 0;
	this->actID = ID + 1;


	/*** leader ***/
	this->attachedNum_max.resize(DoF);
	this->attachedNum_now.resize(DoF);
	this->expectedRewards.resize(DoF);
	this->in_reward.resize(DoF);

}


ActUnit::~ActUnit()
{

}


int ActUnit::calcDistance(SensorUnit& s)
{
	int dist = 0;
	dist = fabs(this->currentPos - s.sensorID);

	return dist;
}


int ActUnit::getNeighborSensorID(SensorUnit(&sensorUnits)[sensorUnitNum])
{
	//センサユニットと通信して距離を知り，そのセンサユニットのIDも取得する．

	int minDist = sensorUnitNum; //絶対この値以上大きくはならないから
	int neighborSensorID = 0;
	for (int i = 0; i < sensorUnitNum; i++) {
		if (minDist > this->calcDistance(sensorUnits[i])) {
			minDist = this->calcDistance(sensorUnits[i]);
			neighborSensorID = i;
		}
	}

	return neighborSensorID;
}


float ActUnit::outputForce(SensorUnit& s)
{
	//とりあえず決め打ち
	//this->force = -2.0 * M_PI * 100.0 * s.displacement;	//これだとシステムの剛性が変わる
	this->force = -2.0 * M_PI * s.velocity;	//add damping force

	return this->force;
}


int ActUnit::judgeBiggestPos(SensorUnit(&sensorUnits)[sensorUnitNum])
{	//judge the biggest amplitude. return sensorID at the DoF with the biggest amplitude.
	int atBiggest = DoF - 1;	
	//bool check_step;
	//std::vector<index_value> in_disp;	//indexとdisplacementのpair配列(vector)→メンバー変数に変更

	this->getEachBiggestDisp(sensorUnits);

		for (int i = 0; i < this->biggestDisps.size(); i++) {
			//this->in_disp.emplace_back(index_value(i, this->biggestDisps[i]));
			this->in_disp[i] = index_value(i, this->biggestDisps[i]);
		}

		std::sort(this->in_disp.begin(), this->in_disp.end(), greaterPair);
		// for (index_value id : in_disp) {
		// 	cout << id.second << endl;
		// }
		// cout << endl;

	if (this->check_step) {

		//reference: https://cpprefjp.github.io/reference/utility/pair.html
		atBiggest = this->in_disp[0].first;	//index
		this->biggestDispofAll = this->in_disp[0].second;	//displacement(この変数要らないかも)

		// cout << "atBiggestDoFPos : " << atBiggest + 1 << endl;
		// cout << this->biggestDisps << endl << endl;

		return sensorUnits[atBiggest].sensorID;
	}

	else {
		return this->currentPos;	//without update
	}
}


void ActUnit::moveTo(int DoFpos)
{
	this->currentPos = DoFpos;
}


void ActUnit::getEachBiggestDisp(SensorUnit (&sensorUnits)[sensorUnitNum])
{
	//対象のステップに達したか知りたいだけ
	for (int i = 0; i < DoF; i++) {	
		//今は近場とか関係なくすべての自由度の情報を集める
		//いずれ，近場だけとかの条件で情報集めたいので，plantが保持するbiggestDispsとは区別する
		//std::tie(this->biggestDisps[i], this->check_step) = sensorUnits[i].getBiggestDisp_steps();
		// check_stepは上書きされてOK
			sensorUnits[i].getBiggestDisp();
			sensorUnits[i].biggestDisp_past = this->biggestDisps[i]; // conflict注意
			this->biggestDisps[i] = sensorUnits[i].biggestDisp;
		if (sensorUnits[i].checkSteps()) {
			this->check_step = true;
		}
		else {
			this->check_step = false;
		}
	}
		// cout << "getEachBiggestDisp" << endl;
		// cout << this->biggestDisps << endl << endl;
}


float ActUnit::outputDampingCoef()
{
	//あとで設計方法を考慮
	this->dampingCoef = 0.5;
	return this->dampingCoef;
}



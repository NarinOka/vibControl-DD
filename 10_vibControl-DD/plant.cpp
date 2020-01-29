#include "Calculation.h"
#include "definedHeaders.h"

Plant::Plant(float m, float c, float k) //constructor
{
	this->x = Eigen::VectorXf::Zero(DoF); //initialize x vector
	this->vel = Eigen::VectorXf::Zero(DoF); //initialize velocity vector
	this->accel = Eigen::VectorXf::Zero(DoF); //initialize acceleration vector
	this->modeAmp = Eigen::VectorXf::Zero(DoF); //initialize mode amplitude vector
	this->MinofMax_vector = Eigen::VectorXf::Zero(DoF);
	this->m.resize(DoF); //—v‘f”‚Ì‰Šú‰»‚ªéŒ¾Žž‚É‚Å‚«‚È‚©‚Á‚½‚©‚ç...
	this->c.resize(DoF);
	this->k.resize(DoF);
	std::fill(this->m.begin(), this->m.end(), m);	//initialize vectors
	std::fill(this->c.begin(), this->c.end(), c);
	std::fill(this->k.begin(), this->k.end(), k);
	this->MinofMax_vector.fill(MinofMax_value);
	//reference for initialization of vector or array: 
	//http://program.station.ez-net.jp/special/handbook/cpp/stl/fill.asp

	this->M_matrix(this->M);
	this->C_matrix(this->C);
	this->K_matrix(this->K);

	this->natural_freq = this->calc_natural_freq(this->M, this->K);
	this->natural_modes = this->calc_natural_modes(this->M, this->K);
	this->simFreq = 0.0;
	this->simT = 0.0;
	this->vibTimes = 0;
	this->vibTimesChanged = false;

	this->biggestDisps = Eigen::VectorXf::Zero(DoF);

		// external forces
	this->y = 0.0;
	this->y_0 = 0.0175;
	this->vy = 0.0;
	this->ay = 0.0;

}

Plant::~Plant() 
{

}

void Plant::M_matrix(MatrixXf& M)	//fill diagonal compionents with m
{
	for (int i = 0; i < DoF; i++)
		M(i, i) = this->m[i];
}

void Plant::C_matrix(MatrixXf& C)
{
	for (int i = 0; i < DoF; i++)
	{
		C(i, i) += this->c[i];
		if (i > 0)
		{
			C(i, i - 1) = -(this->c[i]);
		}
		if (i < (DoF - 1))
		{
			C(i, i) += this->c[i + 1];
			C(i, i + 1) = -(this->c[i + 1]);
		}
	}
}

void Plant::K_matrix(MatrixXf& K)
{
	for (int i = 0; i < DoF; i++)
	{
		K(i, i) += this->k[i];
		if (i > 0)
		{
			K(i, i - 1) = -(this->k[i]);
		}
		if (i < (DoF - 1))
		{
			K(i, i) += this->k[i + 1];
			K(i, i + 1) = -(this->k[i + 1]);
		}
	}
}

Eigen::ArrayXf Plant::calc_natural_freq(MatrixXf& M, MatrixXf& K)
{
	Eigen::ArrayXf eigenvalues; //natural_freq^2
	Eigen::ArrayXf natural_freq;
	eigenvalues.resize(DoF);
	natural_freq.resize(DoF);
	MatrixXf A(DoF, DoF);
	A = M.inverse() * K;

	SelfAdjointEigenSolver<MatrixXf> eigensolver(A);
	if (eigensolver.info() != Success) abort();
	eigenvalues = eigensolver.eigenvalues();
	natural_freq = eigenvalues.sqrt() / 2.0 / M_PI;
	//natural_freq = eigenvalues.sqrt()/2.0/3.141592;

	return natural_freq;
}

Eigen::MatrixXcf Plant::calc_natural_modes(MatrixXf& M, MatrixXf& K)
{
	//std::vector<Eigen::VectorXcf> natural_modes;
	//natural_modes.resize(DoF);
	//
	//Eigen::VectorXcf each_mode;
	//each_mode.resize(DoF);


	MatrixXf A(DoF, DoF);
	A = M.inverse() * K;

	SelfAdjointEigenSolver<MatrixXf> eigensolver(A);
	if (eigensolver.info() != Success) abort();

	MatrixXcf natural_modes = eigensolver.eigenvectors();

	//for (int i = 0; i < DoF; i++) {
	//	natural_modes[i] = eigensolver.eigenvectors().col(i);
	//}

	//natural_modes.emplace_back(each_mode);

	return natural_modes;
}


void Plant::set_simFreq()
{
	// freqSet‚ðŽg‚¤ê‡‚É‚Íã‘‚«‚³‚ê‚é

	if (VIBMODE > 0 && VIBMODE < DoF + 1) {
		this->simFreq = this->natural_freq[VIBMODE - 1];
		this->simT = 1.0 / this->simFreq;
	}
	else 
	{ 
		float inFre = 0;
		cout << "Enter simulation frequency [Hz]: ";
		cin >> inFre;
		this->simFreq = inFre;
		this->simT = 1.0 / this->simFreq;
	}
}


void Plant::cal_vibTimes(float time)
{
	if (time > (this->simT * (this->vibTimes + 1))) {
		this->vibTimes += 1;
		this->vibTimesChanged = true;
		// cout << this->vibTimes << " period" << endl;
	}
	else {
		this->vibTimesChanged = false;
	}
}


void Plant::getBiggestDisps()
{

	//if (this->vibTimesChanged == false) {
		//this->biggestDisps = Eigen::VectorXf::Zero(DoF);
		for (int i = 0; i < DoF; i++) {
			if (this->biggestDisps[i] < this->x[i]) {
				this->biggestDisps[i] = this->x[i];
			}
		}
	//}

}

void Plant::update_external(float t)
{
	if (SUPERPOSED) {
		this->y = 0.0;
		this->vy = 0.0;
		this->ay = 0.0;
		for (int i = 0; i < waveNum; i++) {
			this->y += y_0 * sin(2.0 * M_PI * freqSet[i] * t);
			this->vy += y_0 * 2.0 * M_PI * freqSet[i]
				* cos(2.0 * M_PI * freqSet[i] * t);
			this->ay += -y_0 * pow(2.0 * M_PI * freqSet[i], 2)
				* sin(2.0 * M_PI * freqSet[i] * t);
		}
	}
	else {
		this->y = y_0 * sin(2.0 * M_PI * this->simFreq * t);
		this->vy = y_0 * 2.0 * M_PI * this->simFreq * cos(2.0 * M_PI * this->simFreq * t);
		this->ay = -y_0 * pow(2.0 * M_PI * this->simFreq, 2)
			* sin(2.0 * M_PI * this->simFreq * t);
	}

}
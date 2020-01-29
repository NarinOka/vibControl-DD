#include "definedHeaders.h"



float sigmoid(float x) 
{
	return 1 / (1 + exp(-aSig*x));
}

float tanh_NN(float x) 
{
	return tanh((x - bTanh) / aTanh);
}

// for sensor units
float inputXweight(float input, float weight) 
{
	return input * weight;
}

float returnReward_sigmoid(float A_before, float A_after, int attachedNum)
{
	// reward from each
	if (A_before > A_after)
		return 0.5 + 0.1 * attachedNum;
	else if (A_before == A_after)
		return 0.0;
	else
		return -0.5 - 0.1 * attachedNum;
}

float returnReward_tanh(float A_before, float A_after, int attachedNum)
{
	// reward from each
	if (A_before > A_after)
		return 0.5 + 0.1 * attachedNum;
	else if (A_before == A_after)
		return 0.0;
	else
		return -0.5 -0.1 * attachedNum;
}

void updateWeight(
	//std:: vector<float> &weights, 
	float &weight, float sensorValue, float dEdb)
{
	// float dEdw[DoF] = { 0 };
	float dEdw = 0.0;
	// for (float dedw : dEdw) {
		// dedw = dEdb * sensorValue;
		dEdw = dEdb * sensorValue;
	//}
	// for (int i = 0; i < DoF; i++) {
	// 	weights[i] -= alpha * dEdw[i];
	// }
		weight -= alpha * dEdw;
}


//for actuator units
float calc_forward(float (&fromSensors)[DoF], float &bias) 
{

	float para_for_output = 0.0;
	para_for_output += bias;
	for (float s : fromSensors) {
		// input x weight = from sensors
		para_for_output += s;
	}

	return tanh_NN(para_for_output) * workActUnitNum / DoF;
}

float calc_dEdb_sigmoid(float y, float t)
{
	float dEdb = 0.0;

	dEdb = (y - t) * (1 - y) * y;

	// dE/db‚ð•Ô‚·
	return dEdb;
}

float calc_dEdb_tanh(float y, float t)
{
	float dEdb = 0.0;

	dEdb = (y - t) * (1 - y * y);

	// dE/db‚ð•Ô‚·
	return dEdb;
}

void updateBias(float &bias, float dEdb)
{
	bias -= beta * dEdb;
}
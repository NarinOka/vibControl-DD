#pragma once

// sensor/actuator units construct a 2-layer neural network
// to learn the positions of actuator units

//�����̒l��֐������j�b�g��class���Q�Ƃł���悤�ɂ���

/************* constants ***************/
#define LNum 500000	//�w�K�����
//const int outNum = DoF;	//�o�͑w�̃��j�b�g��
//#define HidNum 3	//���ԑw�̃��j�b�g��
//const int inNum = DoF;	//���͑w�̃��j�b�g��
//#define PatternNum 4	//�p�^�[����(XOR��4�ʂ�)
#define alpha 0.2	//�d�ݗp�w�K�W��
#define beta 0.2	//臒l�p�w�K�W��
#define aSig 0.5	//�V�O���C�h�֐��̃Q�C��
#define aTanh 1.0	//Tanh�֐��̃Q�C��
#define bTanh 0.0	//Tanh�֐��̃Q�C��
//#define V_ini 0.3	//V�����l�̐�Βl
//#define W_ini 0.3	//W�����l�̐�Βl
//#define gamma_ini 5.0	//gamma�����l�̐�Βl
//#define theta_ini 5.0	//theta�����l�̐�Βl
#define w_ini 0.01 //�d�݌W���̏����l�͈�
#define b_ini 0.01 //臒l�̏����l�͈�


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


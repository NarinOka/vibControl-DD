#pragma once
#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <random>
#include <cstdint>
#include <tuple>
#include <string>
#include <numeric>	//for iota
#include <iomanip>	// to get time
#include <windows.h>
// #include <time.h>


using namespace Eigen;
using namespace std;

//reference: https://teramonagi.hatenablog.com/entry/20130226/1361886714
typedef std::pair<int, float> index_value;
bool lessPair(const index_value& l, const index_value& r);
bool greaterPair(const index_value& l, const index_value& r);

// functions
uint64_t get_rand_range(uint64_t min_val, uint64_t max_val);

float get_float_rand_range(float min_val, float max_val);


//reference: https://qiita.com/Gaccho/items/dc312fb5a056505f0a9f
class Rand {
private:
	//32�r�b�g�Ń����Z���k�E�c�C�X�^
	std::mt19937 mt;
	//�񌈒�_�I�ȗ���
	std::random_device rd;

public:
	//�R���X�g���N�^(������)
	Rand() { mt.seed(rd()); }

	//�����l
	void seed() {
		mt.seed(rd());
	}
	void seed(const std::uint_fast32_t seed_) {
		mt.seed(seed_);
	}

	//�ʏ�̗���
	std::uint_fast32_t operator()() {
		return mt();
	}
	//0�`�ő�l-1 (�]��͈̔͂̈�l���z����)(int)
	std::int_fast32_t operator()(const std::int_fast32_t max_) {
		std::uniform_int_distribution<> uid(0, ((max_ > 0) ? (std::int_fast32_t)max_ - 1 : 0));
		return uid(mt);
	}
	//�ŏ��l�`�ő�l(int)
	std::int_fast32_t operator()(const std::int_fast32_t min_, const std::int_fast32_t max_) {
		std::uniform_int_distribution<> uid((min_ <= max_) ? min_ : max_, (min_ <= max_) ? max_ : min_);
		return uid(mt);
	}
	//�ŏ��l�`�ő�l(float)
	double operator()(const double min_, const double max_) {
		std::uniform_real_distribution<> uid((min_ <= max_) ? min_ : max_, (min_ <= max_) ? max_ : min_);
		return uid(mt);
	}
	//�m��
	bool randBool(const double probability_) {
		std::bernoulli_distribution uid(probability_);
		return uid(mt);
	}
	bool randBool() {
		std::uniform_int_distribution<> uid(0, 1);
		return ((uid(mt)) ? true : false);
	}
};
static thread_local Rand rnd;

std::tuple<float, float> calc_DDparams(float m_dd, float mass, float simFreq);	//fixed point theory
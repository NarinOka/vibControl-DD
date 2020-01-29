#include"Calculation.h"

//�^�������𔭐������Ă���̂ł��܂�g�������Ȃ�
uint64_t get_rand_range(uint64_t min_val, uint64_t max_val) 
//reference: https://www.albow.net/entry/random-range
{
	// ����������
	static std::mt19937_64 mt64(0);

	// [min_val, max_val] �̈�l���z���� (int) �̕��z������
	std::uniform_int_distribution<uint64_t> get_rand_uni_int(min_val, max_val);

	// �����𐶐�
	return get_rand_uni_int(mt64);
}

float get_float_rand_range(float min_val, float max_val)
//reference: https://www.albow.net/entry/random-range
{
	// ����������
	static std::mt19937_64 mt64(0);

	// [min_val, max_val] �̈�l���z���� (int) �̕��z������
	std::uniform_real_distribution<float> get_rand_uni_float(min_val, max_val);

	// �����𐶐�
	return get_rand_uni_float(mt64);
}


bool lessPair(const index_value& l, const index_value& r) { return l.second < r.second; }
bool greaterPair(const index_value& l, const index_value& r) { return l.second > r.second; }
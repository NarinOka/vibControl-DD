#include"Calculation.h"

//疑似乱数を発生させているのであまり使いたくない
uint64_t get_rand_range(uint64_t min_val, uint64_t max_val) 
//reference: https://www.albow.net/entry/random-range
{
	// 乱数生成器
	static std::mt19937_64 mt64(0);

	// [min_val, max_val] の一様分布整数 (int) の分布生成器
	std::uniform_int_distribution<uint64_t> get_rand_uni_int(min_val, max_val);

	// 乱数を生成
	return get_rand_uni_int(mt64);
}

float get_float_rand_range(float min_val, float max_val)
//reference: https://www.albow.net/entry/random-range
{
	// 乱数生成器
	static std::mt19937_64 mt64(0);

	// [min_val, max_val] の一様分布整数 (int) の分布生成器
	std::uniform_real_distribution<float> get_rand_uni_float(min_val, max_val);

	// 乱数を生成
	return get_rand_uni_float(mt64);
}


bool lessPair(const index_value& l, const index_value& r) { return l.second < r.second; }
bool greaterPair(const index_value& l, const index_value& r) { return l.second > r.second; }


std::tuple<float, float> calc_DDparams(float m_dd, float mass, float simFreq)
{
	float c_dd, k_dd;
	float mu = m_dd / mass;
	float alpha_opt = 1 / (1 + mu);
	float zeta_opt = sqrt(3.0 * mu / (8.0*(1 + mu)));
	float omega_dd = simFreq * 2.0 * M_PI;

	k_dd = pow(alpha_opt * omega_dd, 2) * m_dd;
	c_dd = 2.0 * sqrt(m_dd * k_dd) * zeta_opt;

	return{ c_dd, k_dd };
}
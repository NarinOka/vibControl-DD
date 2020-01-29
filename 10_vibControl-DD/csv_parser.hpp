// https://qiita.com/Teramonte4/items/6e5ed3c2ab414ec3fc26
#pragma once

#include <string>
#include <vector>
#include <fstream>

class csv_parser
{
public:
	const std::vector<std::vector<std::string>> &rows() {
		return rows_;
	}

	template <typename Function>
	inline void read(const std::string &file_name, Function strinc)
	{
		std::ifstream f(file_name, std::ios::binary);

		std::string str((std::istreambuf_iterator<char>(f)),
			std::istreambuf_iterator<char>());

		std::string::const_iterator it = str.begin();

		auto it_begin = it;

		while (it != str.end())
		{
			if (*it == '"')
			{
				it_begin = ++it;

				for (; !is_double_quotation(it, str);)
					strinc(it);

				str.erase(it);
			}

			for (;;)
			{
				if (it == str.end())
				{
					push_field_with_row(it_begin, it);
					break;
				}
				if (is_delimiter(it))
				{
					push_field(it_begin, it);
					increment_delimiter(it);
					break;
				}
				if (is_crlf(it))
				{
					push_field_with_row(it_begin, it);
					increment_crlf(it);
					break;
				}

				strinc(it);
			}

			it_begin = it;
		}
	}

	inline void read(const std::string &file_name) {
		read(file_name, [](std::string::const_iterator &it) { ++it; });
	}

private:
	std::vector<std::vector<std::string>> rows_;
	std::vector<std::string> fields_;

	void push_field_with_row(const std::string::const_iterator &_First, const std::string::const_iterator &_Last) {
		fields_.push_back(std::string(_First, _Last));
		rows_.push_back(fields_);
		fields_.erase(fields_.begin(), fields_.end());
	}

	void push_field(const std::string::const_iterator &_First, const std::string::const_iterator &_Last) {
		fields_.push_back(std::string(_First, _Last));
	}

	bool is_delimiter(const std::string::const_iterator &it) const {
		return (*it == ',');
	}

	bool is_crlf(const std::string::const_iterator &it) const {
		return ((*it == '\r') && (*(it + 1) == '\n'));
	}

	void increment_delimiter(std::string::const_iterator &it) const {
		++it;
	}

	void increment_crlf(std::string::const_iterator &it) const {
		it += 2;
	}

	bool is_double_quotation(const std::string::const_iterator &it, std::string &str) const {
		if (*it == '"') {
			if ((it + 1) == str.end())
				return true;

			if (*(it + 1) != '"')
				return true;
			else
				str.erase(it);
		}
		return false;
	}
};
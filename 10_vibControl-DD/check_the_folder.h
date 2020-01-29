#pragma once
// https://tanjoin.hatenablog.com/entry/20121009/1349802759

#ifndef CHECK_THE_FOLDER_H_
#define CHECK_THE_FOLDER_H_

#include <string>
#include <direct.h>

class CheckTheFolder
{
public:

	CheckTheFolder(void) {}

	virtual ~CheckTheFolder(void) {}

	static bool checkExistenceOfFolder(const std::string folder_name) {
		if (_mkdir(folder_name.c_str()) == 0) {
			return true;
		}
		else {
			return false;
		}
	}
};

#endif // CHECK_THE_FOLDER_H_
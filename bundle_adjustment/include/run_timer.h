// Copyright (C) 2019 Dongsheng Yang <ydsf16@buaa.edu.cn>
// (Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#ifndef RUN_TIMER_H
#define RUN_TIMER_H
#include <chrono>

class Runtimer{
public:
	inline void start()
	{
		t_s_  = std::chrono::steady_clock::now();
	}
	
	inline void stop()
	{
		t_e_ = std::chrono::steady_clock::now();
	}
	
	inline double duration()
	{
		return std::chrono::duration_cast<std::chrono::duration<double>>(t_e_ - t_s_).count() * 1000.0;
	}
	
private:
	std::chrono::steady_clock::time_point t_s_; //start time ponit
	std::chrono::steady_clock::time_point t_e_; //stop time point
};

#endif

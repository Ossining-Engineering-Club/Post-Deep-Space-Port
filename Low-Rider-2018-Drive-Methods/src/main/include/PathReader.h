#pragma once
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>

class PathReader{
    public:
    void ReadFile(std::string path);
    std::vector<double> GetPositions();
    std::vector<double> GetVelocities();
    
    private:
    std::vector<double> velocity;
	std::vector<double> position;
};
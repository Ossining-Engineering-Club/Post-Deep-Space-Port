#include "PathReader.h"

void PathReader::ReadFile(std::string path){
    std::fstream file;
	file.open(path, std::ios::in);
	std::string lineString;
	std::string numString;
	getline(file, lineString);
	
	while(getline(file, lineString)){
		std::stringstream lineStream;
		lineStream.str(lineString);
		int x = 0;
		while(getline(lineStream, numString, ',')){
			switch(x){
				case 3:
					position.push_back(atof(numString.c_str()));
					break;
				case 4:
					velocity.push_back(atof(numString.c_str()));
					break;
			}
			x++;
		}
	}
}

std::vector<double> PathReader::GetPositions(){
    return position;
}
std::vector<double> PathReader::GetVelocities(){
    return velocity;
}
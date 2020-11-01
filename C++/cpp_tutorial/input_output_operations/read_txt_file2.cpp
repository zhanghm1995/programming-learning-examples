#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <fstream>
#include <iostream>
#include <cstring>
#include <map>

#include <Eigen/Dense>

void Split(const std::string& input_str, std::vector<std::string>& output, const char* delim)  
{
    int pos = 0;  
    int npos = 0;  
    int regexlen = strlen(delim);  
    while((npos = input_str.find(delim, pos)) != -1) {  
        std::string tmp = input_str.substr(pos, npos - pos);  
        output.push_back(tmp);  
        pos = npos + regexlen;  
    }
    output.push_back(input_str.substr(pos, input_str.length() - pos));  
}

std::string& ltrim(std::string& str, const std::string& chars = "\t\n\v\f\r ")
{
    str.erase(0, str.find_first_not_of(chars));
    return str;
}
 
std::string& rtrim(std::string& str, const std::string& chars = "\t\n\v\f\r ")
{
    str.erase(str.find_last_not_of(chars) + 1);
    return str;
}
 
std::string& trim(std::string& str, const std::string& chars = "\t\n\v\f\r ")
{
    return ltrim(rtrim(str, chars), chars);
}

Eigen::MatrixXf GetMatrix(const std::vector<float>& P) {
    Eigen::MatrixXf P2_ = Eigen::Map<const Eigen::Matrix<float, 3, 4, Eigen::RowMajor> >(P.data());

    // Eigen::MatrixXf P2_ = Eigen::Map<const Eigen::MatrixXf>(P.data(), 3, 4);
    Eigen::MatrixXf P2_ = Eigen::Map<const Eigen::Matrix<float, 3, 4, Eigen::RowMajor> >(P.data());
    return P2_;
}

int main()
{
    std::ifstream file("/mnt/work/Datasets/KITTI/tracking/training/calib/0016.txt");
    std::string line;

    std::map<std::string, std::vector<std::string> > calib_content;
    while (std::getline(file, line)) {
        std::cout<<line<<std::endl;
        std::vector<std::string> line_content;
        Split(rtrim(line), line_content, " ");
        std::cout<<line_content.size()<<std::endl;
        std::vector<std::string> calib_param(line_content.begin() + 1, line_content.end());
        calib_content[line_content[0]] = calib_param;
    }

    auto P2_temp = calib_content["P2:"];
    std::vector<float> P2(P2_temp.size());
    std::transform(P2_temp.begin(), P2_temp.end(), P2.begin(), [](const std::string& val) {
      return std::atof(val.c_str()); });

    
    Eigen::MatrixXf P2_ = GetMatrix(P2);
    std::cout<<P2_<<std::endl;

    
    
    return 0;
}
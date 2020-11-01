/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2020年10月23日
* Copyright    :
* Descriptoin  : Learn how to use C++ to read txt format file
* References   :
======================================================================*/

#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <iterator>

struct OXTSInfo {
    double lat;
    double lon;
    double alt;
    double roll;
    double pitch;
    double yaw;
};

bool ReadFile(const std::string& file_name)
{
    bool success = false;
    FILE *fp = fopen(file_name.c_str(), "r");
    if (!fp) {
        return success;
    }

    int num_line = -1;
    while (!feof(fp)) {
        ++num_line;
        OXTSInfo d;
        double trash;
        if (fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf "
                        "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                    &d.lat, &d.lon, &d.alt, &d.roll, &d.pitch, &d.yaw,
                    &trash, &trash, &trash, &trash, &trash, &trash, &trash, &trash,
                    &trash, &trash, &trash, &trash, &trash, &trash, &trash, &trash,
                    &trash, &trash, &trash, &trash, &trash, &trash, &trash, &trash)==30) {
            std::cout<<"==========="<<num_line<<"==========="<<d.lat<<" "<<d.lon<<" "<<d.alt<<" "<<d.roll<<std::endl;
        }
    }
    fclose(fp);
    success = true;
    return success;
}

/**
 * @brief By using std::stringstream and std::istream_iterator to split the string line according to the
 *                space
 *                Note: This method could ignore the extra blank lines and the extra blank spaces
 */ 
void ReadFile2(const std::string& file_name)
{
    std::ifstream in(file_name, std::ios::in);

    if (!in.is_open()) {
        std::cout<<"[ReadFile2] Cannot open the file: "<<file_name<<std::endl;
        return;
    }

    std::string line;
    int num_line = -1;
    while (getline(in, line)) {
        ++num_line;
        std::stringstream ss(line);
        
        std::cout<<"==========="<<num_line<<"===========";

        std::istream_iterator<std::string> begin(ss);
        std::istream_iterator<std::string> end;
        std::vector<std::string> content_vec(begin, end);

        // Print the results
        std::copy(content_vec.begin(), content_vec.end(), std::ostream_iterator<std::string>(std::cout, " "));

        std::cout<<"  "<<content_vec.size()<<std::endl;
    }
    
    in.close();
}

void ReadFile3(const std::string& file_name)
{
    std::ifstream in(file_name, std::ios::in);
    std::string line;

    int num_line = -1;
    while (getline(in, line, ' ')) {
        ++num_line;
        std::cout<<"==========="<<num_line<<"===========";
        std::cout<<line<<std::endl;
    }
    in.close();
}


using namespace std;

int main()
{
    std::string file = "../data/kitti_oxts.txt";
    // ReadFile(file);
    // ReadFile2(file);
    ReadFile2(file);

    vector<char> ta = {'a', 'b', 'c'};
    std::string ch;
    ch.clear();
    ch.assign(ta.begin(),ta.end());
    cout<<ch<<endl;

    return 0;
}
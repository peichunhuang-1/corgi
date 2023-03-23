#ifndef CSV_READER_HPP
#define CSV_READER_HPP
#include <iostream>
#include <fstream>
#include <numeric>
#include <vector>
#include <sstream>
#include <map>
#include <stdexcept>
#include "quaternion.hpp"

class CSV_reader
{
    public:
        std::map<int, std::vector<double> > content;
        int rows;
        int cols;
        void load_data(std::string file_name, int start_row);
        std::vector<Eigen::Vector3d> position_data()
        {
            std::vector<Eigen::Vector3d> r;
            for (int i = 0; i < rows; i++)
            {
                Eigen::Vector3d tmp;
                tmp << content[0][i], content[1][i], content[2][i];
                r.push_back(tmp);
            }
            return r;
        }
        std::vector<mathematic::quaternion> quaternion_data()
        {
            std::vector<mathematic::quaternion> r;
            for (int i = 0; i < rows; i++)
            {
                Eigen::Vector4d tmp;
                tmp << content[3][i], content[0][i], content[1][i], content[2][i];
                r.push_back(mathematic::quaternion(tmp));
            }
            return r;
        }

};
void CSV_reader::load_data(std::string file_name, int start_row)
{
    char delimiter = ',';
    auto ss = std::ostringstream{};
    std::ifstream input_file(file_name);
    if (!input_file.is_open()) {
        std::cerr << "Could not open the file - '"
             << file_name << "'" << std::endl;
        exit(EXIT_FAILURE);
    }
    ss << input_file.rdbuf();
    std::string file_contents = ss.str();
    std::istringstream sstream(file_contents);
    std::string record;
    for (int i = 0; i < start_row; i ++)
    {
        std::getline(sstream, record);
    }
    int counter[2] = {0, 0};
    while (std::getline(sstream, record)) {
        counter[1] = 0;
        std::istringstream line(record);
        while (std::getline(line, record, delimiter)) {
            try
            {
                content[counter[1]].push_back(stod(record));
            }
            catch(std::invalid_argument& e)
            {
                content[counter[1]].push_back(0.0);
            }
            counter[1] += 1;
        }
        counter[0] += 1;
    }
    rows = counter[0];
    cols = counter[1];
}
#endif
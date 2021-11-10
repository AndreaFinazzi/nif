#include <iostream>
#include <fstream>
#include <sstream>

#include "c_csv_tools.h"

void c_csv_tools::fill_matrix_d(const char *file_name, double **matrix,
                                const int row_count, const int col_count,
                                const char &delim)
{
    std::ifstream file(file_name);
    std::string line, cell;

    for (int row = 0; row < row_count; row++)
    {
        std::getline(file, line);
        std::stringstream line_stream(line);

        for (int col = 0; col < col_count; col++)
        {
            std::getline(line_stream, cell, delim);
            matrix[row][col] = std::stod(cell);
        }
    }

    file.close();
    return;
};
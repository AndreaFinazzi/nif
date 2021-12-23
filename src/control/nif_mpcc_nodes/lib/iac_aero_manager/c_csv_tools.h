#include <iostream>

class c_csv_tools
{
private:
    // Disable creating instances
    c_csv_tools();
    ~c_csv_tools();

public:
    static void fill_matrix_d(const char *file_name, double **matrix,
                              const int row_count, const int col_count,
                              const char &delim);
};

/*
 * @file    low_pass_filter.h
 * @author  Seungwook Lee
 * @date    2020-10-16
 * @brief   class implementation of 
 *          1st order low pass filter(frequency(in Hz) based)
 * @arg     input:
 *          output:
 * @usages
 * 
 */
#include <vector>

class low_pass_filter
{
public:
    template <typename T>
    low_pass_filter(T dt, T cut_f_hz, T raw_data_init);
    ~low_pass_filter();

    double compute_weight(double dt, double cut_f_radps);
    template <typename T>
    void getFilteredValue(T raw, T &filtered_data);
    template <typename T>
    void getFilteredArray(std::vector<T> raw_array, std::vector<T> &filtered_array);

private:
    double PI_ = 3.14159265359;
    double twoPI_ = 2.0 * 3.14159265359;

    double dt_;
    double cut_f_radps_;
    double raw_data_init_;
    double last_filtered_;
    double last_filtered_array_data_;
    double weight_;
};

template <typename T>
low_pass_filter::low_pass_filter(T dt, T cut_f_hz, T raw_data_init)
{
    dt_            = (double) dt;
    cut_f_radps_   = (double) cut_f_hz * twoPI_;
    raw_data_init_ = (double) raw_data_init;
    last_filtered_ = (double) raw_data_init;
    weight_        = compute_weight(dt_, cut_f_radps_);
}

low_pass_filter::~low_pass_filter()
{
}

template <typename T>
void low_pass_filter::getFilteredValue(T raw_data, T &filtered_data)
{
    double raw_data_d   = (double) raw_data;
    filtered_data       = (T) (weight_ * raw_data_d + (1-weight_) * last_filtered_);
    last_filtered_      = (double) filtered_data;
}

template <typename T>
void low_pass_filter::getFilteredArray(std::vector<T> raw_array, std::vector<T> &filtered_array)
{
    int array_size = raw_array.size();
    filtered_array.clear();

    last_filtered_array_data_ = (double) raw_array[0];
    filtered_array.push_back(raw_array[0]); // initial case of "i = 0"
    for(int i = 1; i < array_size; i++){
        double filtered_data = weight_ * raw_array[i] + (1 - weight_) * last_filtered_array_data_;
        filtered_array.push_back((T)filtered_data);
        last_filtered_array_data_ = filtered_data;  
    }
}

double low_pass_filter::compute_weight(double dt, double cut_f_radps)
{
    double weight = 2.0 * cut_f_radps / (3.0*cut_f_radps + 2.0 / dt);
    return weight;
}
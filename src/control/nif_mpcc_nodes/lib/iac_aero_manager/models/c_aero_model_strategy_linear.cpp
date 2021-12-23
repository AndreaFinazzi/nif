#include "c_aero_model_strategy_linear.h"

c_aero_model_strategy_linear::c_aero_model_strategy_linear()
{
    drag_beta_matrix = new double *[CSV_DIM_ROW];
    lift_front_beta_matrix = new double *[CSV_DIM_ROW];
    lift_rear_beta_matrix = new double *[CSV_DIM_ROW];

    for (int i = 0; i < CSV_DIM_ROW; i++)
    {
        drag_beta_matrix[i] = new double[CSV_DIM_COL];
        lift_front_beta_matrix[i] = new double[CSV_DIM_COL];
        lift_rear_beta_matrix[i] = new double[CSV_DIM_COL];
    }
    initBetaMatrices();
}

c_aero_model_strategy_linear::~c_aero_model_strategy_linear()
{
    for (int i = 0; i < CSV_DIM_ROW; i++)
    {
        delete[] drag_beta_matrix[i];
        delete[] lift_front_beta_matrix[i];
        delete[] lift_rear_beta_matrix[i];
    }
    delete[] drag_beta_matrix;
    delete[] lift_front_beta_matrix;
    delete[] lift_rear_beta_matrix;
}

void c_aero_model_strategy_linear::initBetaMatrices()
{
    const char* path = "/home/usrg/ros2_ws/src/mpcc_ros2/lib/iac_aero_manager/assets/a_aero_model_linear_drag.csv";
    //   read from file
    c_csv_tools::fill_matrix_d(path, drag_beta_matrix, CSV_DIM_ROW, CSV_DIM_COL, ';');
    c_csv_tools::fill_matrix_d(CSV_PATH_LIFT_FRONT, lift_front_beta_matrix, CSV_DIM_ROW, CSV_DIM_COL, ';');
    c_csv_tools::fill_matrix_d(CSV_PATH_LIFT_REAR, lift_rear_beta_matrix, CSV_DIM_ROW, CSV_DIM_COL, ';');
}

t_aero_state c_aero_model_strategy_linear::getAeroState(
    const t_self_veh_state &ego,
    const std::vector<t_veh_state> &others)
{
    t_aero_state aero_state = {
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0}};

    // signed value for x delta, absolute for y delta
    double delta_x = others[0].pose_x - ego.pose_x,
           delta_y = std::abs(others[0].pose_y - ego.pose_y),
           beta_value = 1.0;

    // Get y index
    int id_y_floor = floor(delta_y / CSV_GRANULARITY_Y),
        id_x_floor = floor(std::abs(delta_x) / CSV_GRANULARITY_X);

    if (delta_x <= 0 || delta_x >= activation_max_x || delta_y >= activation_max_y)
    {
        // No draft effect, do nothing
    }
    else if (delta_x <= CSV_MIN_X)
    {
        // Maximum activation area
        // If delta_x is under the min value, just consider the maximum advantage [y][0]

        // Assign to x-component of drag vector
        aero_state.drag_f[0] = getForce(drag_beta_matrix[id_y_floor][0], ego.vel, AERO_COEF_DRAG); // maximum advantage

        // Assign to z-component of lift vectors
        aero_state.lift_front_f[2] = getForce(lift_front_beta_matrix[id_y_floor][0], ego.vel, AERO_COEF_LIFT_FRONT);
        aero_state.lift_rear_f[2] = getForce(lift_rear_beta_matrix[id_y_floor][0], ego.vel, AERO_COEF_LIFT_REAR);
    }
    else if (delta_x < activation_max_x && delta_y < activation_max_y)
    {
        // Inside activation area -> refer to tables
        // Assign to x-component of drag vector
        aero_state.drag_f[0] = getForce(drag_beta_matrix[id_y_floor][id_x_floor], ego.vel, AERO_COEF_DRAG);

        // Assign to z-component of lift vectors
        aero_state.lift_front_f[2] = getForce(lift_front_beta_matrix[id_y_floor][id_x_floor], ego.vel, AERO_COEF_LIFT_FRONT);
        aero_state.lift_rear_f[2] = getForce(lift_rear_beta_matrix[id_y_floor][id_x_floor], ego.vel, AERO_COEF_LIFT_REAR);
    }

    return aero_state;
}

double c_aero_model_strategy_linear::getCr2Coefficient(
        const double &delta_x,
        const double &delta_y,
        const double &ego_v,
        const double &c_coeff)
{
    // signed value for x delta, absolute for y delta
    double beta_value = 1.0;

    // Get y index
    int id_y_floor = (int) floor(std::abs(delta_y) / CSV_GRANULARITY_Y),
        id_x_floor = (int) floor(std::abs(delta_x) / CSV_GRANULARITY_X);

    // std::cout << "delta_x : " << delta_x << " delta_y : " << delta_y << " ego_v : " << ego_v << std::endl;

    if (delta_x <= 0 || delta_x >= activation_max_x || std::abs(delta_y) >= activation_max_y)
    {
        // No draft effect, do nothing
        // std::cout << "No draft effect " << delta_x << ", " << delta_y << std::endl;
        return getCr2(1, ego_v, c_coeff);
    }
    else if (delta_x <= CSV_MIN_X)
    {
        // Maximum activation area
        // If delta_x is under the min value, just consider the maximum advantage [y][0]

        // Assign to x-component of drag vector
        // std::cout << "beta : " << drag_beta_matrix[id_y_floor][0] << std::endl;
        return getCr2(drag_beta_matrix[id_y_floor][0], ego_v, c_coeff); // maximum advantage
    }
    else if (delta_x < activation_max_x && std::abs(delta_y) < activation_max_y)
    {
        // Inside activation area -> refer to tables
        // Assign to x-component of drag vector
        // std::cout << "beta : " << drag_beta_matrix[id_y_floor][id_x_floor] << std::endl;
        return getCr2(drag_beta_matrix[id_y_floor][id_x_floor], ego_v, c_coeff);
    }
    else{
        std::cout << "in ELSE : " << std::endl;
    }
}

double c_aero_model_strategy_linear::getDeltaCr2Coefficient(
        const double &cr2_default,
        const double &delta_x,
        const double &delta_y,
        const double &ego_v,
        const double &c_coeff)
{
    double delta_cr2 = 0.0;
    delta_cr2 = cr2_default - this->getCr2Coefficient(delta_x, delta_y,ego_v,c_coeff);
    // std::cout << "cr2_default : " << cr2_default << " getCr2Coefficient : " << delta_cr2 << std::endl;

    return delta_cr2;
}

double c_aero_model_strategy_linear::getForce(const double beta, const double ego_v, double Clift)
{
    // Fdrag = 1/2*(v*v*r*(alpha-1)*Cd*A)]

    double v_ratio = ego_v / AERO_COEF_V_MAX;
    double alpha = 1 + std::min<double>(1, v_ratio) * (beta - 1);

    return 0.5 * AERO_COEF_AIR_DENSITY * AERO_COEF_CONTACT_SURFACE * (alpha - 1) * ego_v * ego_v * Clift;
}

double c_aero_model_strategy_linear::getCr2(const double beta, const double ego_v, double c_coeff)
{
    // Fdrag = 1/2*(v*v*r*(alpha-1)*Cd*A)]

    double v_ratio = ego_v / AERO_COEF_V_MAX;
    double alpha = 1 + std::min<double>(1, v_ratio) * (beta - 1);

    return 0.5 * AERO_COEF_AIR_DENSITY * AERO_COEF_CONTACT_SURFACE * (alpha) * c_coeff;
}
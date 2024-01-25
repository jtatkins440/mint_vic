Header header
int64 mint_input_seq_length
float64[] mint_input_times
float64[] mint_input_vel_x # using flat arrays instead of multiarrays for the sake of time and saving the data with less trouble
float64[] mint_input_vel_y
float64[] mint_input_acc_x
float64[] mint_input_acc_y
int64 mint_output_seq_length
float64[] mint_output_times
float64[] mint_output_dpos_x
float64[] mint_output_dpos_y

int64 line_input_seq_length
float64[] line_input_times
float64[] line_input_pos_x
float64[] line_input_pos_y
int64 line_output_dim
float64[] line_output_intercept
float64[] line_output_slope

int64 circle_input_seq_length
float64[] circle_input_times
float64[] circle_input_pos_x
float64[] circle_input_pos_y
int64 circle_output_dim
float64[] circle_output_center
float64 circle_output_radius

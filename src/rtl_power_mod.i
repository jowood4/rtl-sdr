
 /* example.i */
 %module rtl_power_mod
 %{
 /* Put header files here or function declarations like below */
	extern void rms_power(int ts_index, double *rms_pow_val, double *rms_pow_dc_val);

	extern void read_data(int index);

	extern void initialize_tuner_values(int index);

	extern void find_and_open_dev(void);

	extern void close_dev(void);

	extern void set_tuner(int index);
 %}

extern void rms_power(int ts_index, double *rms_pow_val, double *rms_pow_dc_val);

extern void read_data(int index);

extern void initialize_tuner_values(int index);

extern void find_and_open_dev(void);

extern void close_dev(void);

extern void set_tuner(int index);


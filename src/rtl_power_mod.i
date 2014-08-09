 
 /* example.i */
 %module rtl_power_mod
 %{
 /* Put header files here or function declarations like below */
	extern void rms_power(int ts_index, uint8_t *buf, double *rms_pow_val, double *rms_pow_dc_val);

	extern void read_data(int index, uint8_t *buf8);

	extern void initialize_tuner_values(int index);

	extern void find_and_open_dev(void);

	extern void close_dev(void);

	extern void set_tuner(int index);

	extern void set_value(int index, char param, double value);
 %}
 %include "stdint.i"
 %include "cpointer.i"
 %pointer_functions(uint8_t, uint8p);
 %pointer_functions(double, doublep);

extern void rms_power(int ts_index, uint8_t *buf, double *rms_pow_val, double *rms_pow_dc_val);

extern void read_data(int index, uint8_t *buf8);

extern void initialize_tuner_values(int index);

extern void find_and_open_dev(void);

extern void close_dev(void);

extern void set_tuner(int index);

extern void set_value(int index, char param, double value);


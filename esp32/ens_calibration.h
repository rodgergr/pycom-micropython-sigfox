#ifndef __INCLUDED_ENSCALIBRATION_H
#define __INCLUDED_ENSCALIBRATION_H


bool get_ens_calibration_status (void);
void run_ens_calibration (void);
void set_display_state(bool state);
bool get_power_fail_state(void);
bool is_forced_calibration_mode(void);
void disable_display_uart_and_wait(void);

#endif

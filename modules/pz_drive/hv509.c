#include "pz_drive.h"
void hv509_init(void) {}
void hv509_sr_stage(uint32_t word32) { (void)word32; }
void hv509_sr_write(uint32_t word32) { (void)word32; }
void hv509_sr_latch_if_pending(void) {}
void hv509_pol_init(void) {}
void hv509_pol_set(bool val) { (void)val; }
bool hv509_pol_get(void) { return false; }
void hv509_pol_toggle(void) {}

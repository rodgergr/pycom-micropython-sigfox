/*
 * Copyright (c) 2017, Pycom Limited.
 *
 * This software is licensed under the GNU GPL version 3 or any
 * later version, with permitted additional terms. For more information
 * see the Pycom Licence v1.0 document supplied with this file, or
 * available at https://www.pycom.io/opensource/licensing
 */

#ifndef MACHWDT_H_
#define MACHWDT_H_

extern const mp_obj_type_t mach_wdt_type;

extern void machine_wdt_start (uint32_t timeout_ms);

#endif // MACHWDT_H_

/*
 * Chip-specific setup code for the SAMA5D2 family
 *
 * Copyright (C) 2014 Atmel Corporation,
 *		      Bo Shen <voice.shen@atmel.com>
 *
 * Licensed under GPLv2 or later.
 */

#include <common.h>
#include <init.h>
#include <restart.h>
#include <mach/hardware.h>
#include <mach/at91_pmc.h>
#include <mach/board.h>
#include <mach/at91_rstc.h>

static void sama5d2_restart(struct restart_handler *rst)
{
	at91sam9g45_reset(IOMEM(SAMA5D2_BASE_MPDDRC),
			  IOMEM(SAMA5D2_BASE_RSTC + AT91_RSTC_CR));
}

/* --------------------------------------------------------------------
 *  Processor initialization
 * -------------------------------------------------------------------- */
static int sama5d2_setup(void)
{
	restart_handler_register_fn(sama5d2_restart);
	return 0;
}
pure_initcall(sama5d2_setup);

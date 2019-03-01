/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>

#include "camera.h"

#ifdef CONFIG_NETWORKING

void network_init(void)
{
	struct net_if *iface = net_if_get_default();

#ifdef CONFIG_NET_DHCPV4
	net_dhcpv4_start(iface);
#endif
}
#else
static inline void network_init(void) {}
#endif /*CONFIG_NETWORKING */

void main(void)
{
	/* icarus app entry point */
	network_init();

	camera_run();
}

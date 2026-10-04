#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

// lwipopts_examples_common.h defaults LWIP_SOCKET to 0 (guarded by #ifndef,
// since the httpd/REST layer in http_rest.c only ever needed the raw API),
// so it has to be turned on here, before that header is pulled in, for
// ota_client.c's outbound HTTP client (lwip/sockets.h + lwip/netdb.h) to be
// usable.
#define LWIP_SOCKET 1

// Generally you would define your own explicit list of lwIP options
// (see https://www.nongnu.org/lwip/2_1_x/group__lwip__opts.html)
//
// This example uses a common include to avoid repetition
#include "lwipopts_examples_common.h"

// lwipopts_examples_common.h #defines LWIP_NETCONN to 0 unconditionally (no
// #ifndef guard), but the socket API is implemented on top of netconn, so it
// has to be turned back on here, after that header, for ota_client.c.
#undef LWIP_NETCONN
#define LWIP_NETCONN 1



#if !NO_SYS
#define TCPIP_THREAD_STACKSIZE 4096
#define DEFAULT_THREAD_STACKSIZE 1024
#define ASYNC_CONTEXT_DEFAULT_FREERTOS_TASK_STACK_SIZE 4096
#define DEFAULT_RAW_RECVMBOX_SIZE 8
#define TCPIP_MBOX_SIZE 8
#define LWIP_TIMEVAL_PRIVATE 0
#define TCPIP_THREAD_PRIO   7

// // not necessary, can be done either way
#define LWIP_TCPIP_CORE_LOCKING_INPUT 1

// // ping_thread sets socket receive timeout, so enable this feature
#define LWIP_SO_RCVTIMEO 1
// ota_client.c/ota_tls.c set a send timeout too (SO_SNDTIMEO) so an outbound
// write can never block forever; without this it's silently a no-op.
#define LWIP_SO_SNDTIMEO 1

// This firmware's web server and mDNS responder only ever used lwIP's raw
// tcp_*/udp_* API, which never goes through netconn -- so these netconn
// mailbox sizes were never given explicit values and stayed at their
// lwIP default of 0 (see lwip/opt.h). A size-0 mailbox becomes a
// zero-length FreeRTOS queue in this port's sys_mbox_new(), which is
// undefined behaviour on FreeRTOS (xQueueCreate() requires length >= 1).
// ota_client.c's outbound TLS client is the first thing in this codebase
// to actually open a socket via lwip_socket()/lwip_connect(), which goes
// through netconn and therefore through these mailboxes -- and hung/locked
// up the whole board right at that step until these were sized properly.
#define DEFAULT_TCP_RECVMBOX_SIZE  8
#define DEFAULT_ACCEPTMBOX_SIZE    4
#define DEFAULT_UDP_RECVMBOX_SIZE  4
#endif

// Lwip features
#define LWIP_HTTPD_CGI                  1    // Enable HTTPCGI
#define LWIP_HTTPD_SUPPORT_POST         0    // Enable POST
#define LWIP_HTTPD_CUSTOM_FILES         0   
#define LWIP_HTTPD_DYNAMIC_FILE_READ    0
#define LWIP_HTTPD_DYNAMIC_HEADERS      0
#define LWIP_HTTPD_MAX_REQ_LENGTH       2400
#define LWIP_HTTPD_MAX_REQUEST_URI_LEN  2000
#define LWIP_HTTPD_MAX_CGI_PARAMETERS   32
#define LWIP_HTTPD_DYNAMIC_HEADERS      0

// Outbound HTTP client support (self-hosted firmware update checks), used by
// ota_client.c. LWIP_DNS is already 1 in lwipopts_examples_common.h; just
// give ourselves a second DNS server slot since STA mode can hand us more
// than one via DHCP.
#define DNS_MAX_SERVERS 2

// MDNS
#define LWIP_MDNS_RESPONDER 1
#define LWIP_IGMP 1
#define LWIP_NUM_NETIF_CLIENT_DATA 1
#define MDNS_MAX_SERVICES 2  // increase from 1 to 2 
#define MDNS_RESP_USENETIF_EXTCALLBACK  1
#define MEMP_NUM_SYS_TIMEOUT (LWIP_NUM_SYS_TIMEOUT_INTERNAL + 8)
#define MEMP_NUM_TCP_PCB 12

#endif

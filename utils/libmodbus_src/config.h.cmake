/*
    This file is from https://github.com/ongun-kanat/libmodbus/tree/cmake
    Copyright Ongun Kanat
*/

#ifndef CONFIG_H
#define CONFIG_H

#cmakedefine01 HAVE_ARPA_INET_H
#cmakedefine01 HAVE_BYTESWAP_H
#cmakedefine01 HAVE_ERRNO_H
#cmakedefine01 HAVE_FCNTL_H
#cmakedefine01 HAVE_LIMITS_H
#cmakedefine01 HAVE_LINUX_SERIAL_H
#cmakedefine01 HAVE_NETDB_H
#cmakedefine01 HAVE_NETINET_IN_H
#cmakedefine01 HAVE_NETINET_TCP_H
#cmakedefine01 HAVE_SYS_IOCTL_H
#cmakedefine01 HAVE_SYS_PARAMS_H
#cmakedefine01 HAVE_SYS_SOCKET_H
#cmakedefine01 HAVE_SYS_TIME_H
#cmakedefine01 HAVE_SYS_TYPES_H
#cmakedefine01 HAVE_TERMIOS_H
#cmakedefine01 HAVE_TIME_H
#cmakedefine01 HAVE_UNISTD_H

#cmakedefine01 HAVE_ACCEPT4
#cmakedefine01 HAVE_FORK
#cmakedefine01 HAVE_GETADDRINFO
#cmakedefine01 HAVE_GETTIMEOFDAY
#cmakedefine01 HAVE_INET_NTOA
#cmakedefine01 HAVE_MALLOC
#cmakedefine01 HAVE_MEMSET
#cmakedefine01 HAVE_SELECT
#cmakedefine01 HAVE_SOCKET
#cmakedefine01 HAVE_STRERROR
#cmakedefine01 HAVE_STRLCPY

#cmakedefine HAVE_TIOCRS485
#cmakedefine HAVE_TIOCM_RTS

#cmakedefine01 HAVE_WINSOCK2_H

#ifdef HAVE_TIOCM_RTS
#define HAVE_DECL_TIOCM_RTS 1
#else
#define HAVE_DECL_TIOCM_RTS 0
#endif

#ifdef HAVE_TIOCRS485
#define HAVE_DECL_TIOCSRS485 1
#else
#define HAVE_DECL_TIOCSRS485 0
#endif

#endif

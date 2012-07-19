# $Colby: erlang/drv_adrienne/BSDMakefile,v 1.2 2006/04/05 19:10:49 olgeni Exp $

KMOD=	adrienne
SRCS=	adrienne.c \
	adrienne.h \
	device_if.h \
	bus_if.h \
	pci_if.h

.include <bsd.kmod.mk>

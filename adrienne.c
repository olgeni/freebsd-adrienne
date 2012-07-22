/*-
 * Copyright (c) 2012 Giacomo Olgeni. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/types.h>
#include <sys/systm.h>
#include <sys/module.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/uio.h>
#include <sys/poll.h>
#include <sys/selinfo.h>
#include <machine/bus.h>
#include <sys/rman.h>
#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>

#define ADRIENNE_PCI_VENDOR 0xAECB
#define ADRIENNE_PCI_DEVICE 0x6250

/* Adrienne documentation (iomap2.txt) */
#define AEC_VENDOR_CODE_LOW_BYTE           0x00
#define AEC_VENDOR_CODE_HIGH_BYTE          0x01
#define PCI_TC_BOARD_NUMBER_CODE_LOW_BYTE  0x02
#define PCI_TC_BOARD_NUMBER_CODE_HIGH_BYTE 0x03
#define SOFTWARE_REVISION_LETTER           0x06
#define SOFTWARE_REVISION_NUMBER           0x07
#define BOARD_CAPABILITIES_CODE            0x08
#define BOARD_OPERATING_MODE_INFORMATION   0x0D
#define PENDING_INTERRUPTS_BIT_MAP         0x0E
#define BOARD_TO_HOST_MAILBOX_PORT         0x0F
#define READER_TIME_BITS_FRAMES            0x10
#define READER_TIME_BITS_SECONDS           0x11
#define READER_TIME_BITS_MINUTES           0x12
#define READER_TIME_BITS_HOURS             0x13
#define READER_USER_BITS_FRAMES            0x14
#define READER_USER_BITS_SECONDS           0x15
#define READER_USER_BITS_MINUTES           0x16
#define READER_USER_BITS_HOURS             0x17
#define READER_EMBEDDED_BITS               0x18
#define READER_STATUS_BITS                 0x19
#define TC_COMPARATOR_STATUS_BITS          0x1B
#define BOARD_TO_HOST_MAILBOX_DATA_AREA    0x1C
#define HOST_TO_BOARD_MAILBOX_DATA_AREA    0x20
#define LTC_READER_CONTROL                 0x2C
#define COMPARATOR_CONTROL                 0x2D
#define INTERRUPT_CONTROL_BIT_MAP          0x2E
#define HOST_TO_BOARD_MAILBOX_PORT         0x2F
#define COMPARATOR_TIME_BITS_FRAMES        0x30
#define COMPARATOR_TIME_BITS_SECONDS       0x31
#define COMPARATOR_TIME_BITS_MINUTES       0x32
#define COMPARATOR_TIME_BITS_HOURS         0x33
#define SPECIAL_BOARD_STATUS_REGISTER_BASE 0xFC
#define SPECIAL_BOARD_STATUS_REGISTER      0xFE

/* Adrienne documentation (iomap2.txt) */
#define INTERRUPT_CONTROL_TC_READER       0x01
#define INTERRUPT_CONTROL_VIDEO_FIELD     0x02
#define INTERRUPT_CONTROL_TC_GENERATOR    0x04
#define INTERRUPT_CONTROL_SERIAL_TRANSMIT 0x08
#define INTERRUPT_CONTROL_SERIAL_RECEIVE  0x10
#define INTERRUPT_CONTROL_TC_COMPARATOR   0x20

/* Adrienne documentation (iomap2.txt) */
#define PENDING_INTERRUPT_TC_READER            0x01
#define PENDING_INTERRUPT_VIDEO_FIELD          0x02
#define PENDING_INTERRUPT_TC_GENERATOR         0x04
#define PENDING_INTERRUPT_SERIAL_TRANSMIT_DATA 0x08
#define PENDING_INTERRUPT_SERIAL_RECEIVE_DATA  0x10
#define PENDING_INTERRUPT_TC_COMPARATOR        0x20
#define PENDING_INTERRUPT_RESERVED             0x40
#define PENDING_INTERRUPT_COMMAND_RESPONSE     0x80

/* Adrienne documentation (iomap2.txt) */
#define STATUS_MAILBOX_DATA_READY       0x01
#define STATUS_INT_LINE_DRIVEN          0x02
#define STATUS_MAILBOX_FULL             0x04
#define STATUS_INT_LINE_DRIVING_ENABLED 0x08

/* Adrienne documentation (ioif2.txt) */
#define CMD_ACKNOWLEDGE               0x00
#define CMD_BOARD_SOFTWARE_RESET      0x02
#define CMD_SMPTE_MODE_AND_IDLE_MODE  0x03
#define CMD_EBU_MODE_AND_IDLE_MODE    0x04
#define CMD_IDLE_MODE                 0x20
#define CMD_LTC_READER_MODE           0x21
#define CMD_VITC_READER_MODE          0x22
#define CMD_AUTO_LTC_VITC_READER_MODE 0x23

#define DATA_BLOCK_SIZE 10

static d_open_t      adr_open;
static d_close_t     adr_close;
static d_ioctl_t     adr_ioctl;
static d_read_t      adr_read;
static d_write_t     adr_write;
static d_poll_t      adr_poll;
static driver_intr_t adr_intr;

static struct cdevsw adr_cdevsw = {
	.d_version = D_VERSION,
	.d_flags   = D_NEEDGIANT,
	.d_open    = adr_open,
	.d_close   = adr_close,
	.d_read    = adr_read,
	.d_write   = adr_write,
	.d_ioctl   = adr_ioctl,
	.d_poll    = adr_poll,
	.d_name    = "adrienne"
};

static devclass_t adr_devclass;

struct adr_sc {
	device_t		dev;
	struct cdev		*cdev;

	int			bar0_id;
	struct resource		*bar0_res;
	bus_space_tag_t		bar0_bt;
	bus_space_handle_t	bar0_bh;

	int			irq_id;
	struct resource		*irq_res;
	bus_space_tag_t		irq_bt;
	bus_space_handle_t	irq_bh;
	void			*irq_cookie;

	uint8_t			tc_hh, tc_mm, tc_ss, tc_ff;
	uint8_t			ub_hh, ub_mm, ub_ss, ub_ff;
	uint8_t			embedded, status;

	int			available;

	struct selinfo		selinfo;
	struct mtx		mutex;
};

static int
reset_board (struct adr_sc *sc)
{
	int counter = 1000;

	while (((bus_space_read_1 (sc->bar0_bt, sc->bar0_bh, SPECIAL_BOARD_STATUS_REGISTER)
                 & STATUS_MAILBOX_FULL) != 0) && (counter--))
		DELAY (10);

	bus_space_write_1 (sc->bar0_bt, sc->bar0_bh, HOST_TO_BOARD_MAILBOX_PORT, CMD_BOARD_SOFTWARE_RESET);
	bus_space_read_1 (sc->bar0_bt, sc->bar0_bh, HOST_TO_BOARD_MAILBOX_PORT);

	tsleep (sc->cdev, PRIBIO, "adricmd", hz / 2); /* nobody will call wakeup on sc->cdev */

	bus_space_write_1 (sc->bar0_bt, sc->bar0_bh, HOST_TO_BOARD_MAILBOX_PORT, CMD_ACKNOWLEDGE);
	bus_space_read_1 (sc->bar0_bt, sc->bar0_bh, HOST_TO_BOARD_MAILBOX_PORT);

	return 0;
}

static int
send_command (struct adr_sc *sc, uint8_t command)
{
	int counter = 1000;

	while (((bus_space_read_1 (sc->bar0_bt, sc->bar0_bh, SPECIAL_BOARD_STATUS_REGISTER) & STATUS_MAILBOX_FULL) != 0) && (counter--))
		DELAY (10);

	if (counter == 0) {
		if (bootverbose)
			printf ("send_command: counter timeout\n");

		return EIO;
	}

	bus_space_write_1 (sc->bar0_bt, sc->bar0_bh, HOST_TO_BOARD_MAILBOX_PORT, command);
	bus_space_read_1 (sc->bar0_bt, sc->bar0_bh, HOST_TO_BOARD_MAILBOX_PORT);

	if (tsleep (sc, PRIBIO, "adricmd", 1 * hz) == EWOULDBLOCK) {
		if (bootverbose)
			printf ("send_command: interrupt timeout\n");

		return EIO;
	}

	return 0;
}

static int
adr_open (struct cdev *cdev, int flag, int otyp, struct thread *td)
{
	struct adr_sc *sc = cdev->si_drv1;

	device_busy (sc->dev);

	sc->available = 0;

	int error = 0;

	if ((error = send_command (sc, CMD_EBU_MODE_AND_IDLE_MODE)))
		return error;

	if ((error = send_command (sc, CMD_AUTO_LTC_VITC_READER_MODE)))
		return error;

	return error;
}

static int
adr_close (struct cdev *cdev, int flag, int otyp, struct thread *td)
{
	struct adr_sc *sc = cdev->si_drv1;

	int error = send_command (sc, CMD_IDLE_MODE);

	device_unbusy (sc->dev);

	return error;
}

static int
adr_read (struct cdev *cdev, struct uio *uio, int ioflag)
{
	struct adr_sc *sc = cdev->si_drv1;

	int error = 0;

	if (!sc->available)
		if (tsleep (sc, PRIBIO, "adrird", 3 * hz) == EWOULDBLOCK)
			return EIO;

	uint8_t buffer[DATA_BLOCK_SIZE];

        uint8_t *ptr = buffer;

	*ptr++ = sc->tc_hh;
	*ptr++ = sc->tc_mm;
	*ptr++ = sc->tc_ss;
	*ptr++ = sc->tc_ff;
	*ptr++ = sc->ub_hh;
	*ptr++ = sc->ub_mm;
	*ptr++ = sc->ub_ss;
	*ptr++ = sc->ub_ff;
	*ptr++ = sc->embedded;
	*ptr++ = sc->status;

	sc->available = 0;

	/* FIXME probably not the best way to use uiomove */
	if ((error = uiomove (buffer, DATA_BLOCK_SIZE, uio)) != 0)
                return error;

	return error;
}

static int
adr_write (struct cdev *cdev, struct uio *uio, int ioflag)
{
	return EPERM;
}

static int
adr_ioctl (struct cdev *cdev, u_long cmd, caddr_t arg, int mode, struct thread *td)
{
	return EPERM;
}

static int
adr_poll (struct cdev *cdev, int events, struct thread *td)
{
	struct adr_sc *sc = cdev->si_drv1;

	int revents = 0, mask = 0;

	if ((events & (POLLIN | POLLRDNORM)) && sc->available)
		mask |= (POLLIN | POLLRDNORM);

	if (mask != 0)
		revents = events & mask;
	else
		selrecord (td, &sc->selinfo);

	return revents;
}

static void
adr_intr (void *parameter)
{
	struct adr_sc *sc = (struct adr_sc *) parameter;

	uint8_t response = bus_space_read_1 (sc->bar0_bt, sc->bar0_bh, BOARD_TO_HOST_MAILBOX_PORT);

	switch (response) {
	case 0x13:
		/* TC reader data is ready (must enable 2Eh bit 0 first!). */
		sc->tc_hh = bus_space_read_1 (sc->bar0_bt, sc->bar0_bh, READER_TIME_BITS_HOURS);
		sc->tc_mm = bus_space_read_1 (sc->bar0_bt, sc->bar0_bh, READER_TIME_BITS_MINUTES);
		sc->tc_ss = bus_space_read_1 (sc->bar0_bt, sc->bar0_bh, READER_TIME_BITS_SECONDS);
		sc->tc_ff = bus_space_read_1 (sc->bar0_bt, sc->bar0_bh, READER_TIME_BITS_FRAMES);
		sc->ub_hh = bus_space_read_1 (sc->bar0_bt, sc->bar0_bh, READER_USER_BITS_HOURS);
		sc->ub_mm = bus_space_read_1 (sc->bar0_bt, sc->bar0_bh, READER_USER_BITS_MINUTES);
		sc->ub_ss = bus_space_read_1 (sc->bar0_bt, sc->bar0_bh, READER_USER_BITS_SECONDS);
		sc->ub_ff = bus_space_read_1 (sc->bar0_bt, sc->bar0_bh, READER_USER_BITS_FRAMES);

		sc->embedded = bus_space_read_1 (sc->bar0_bt, sc->bar0_bh, READER_EMBEDDED_BITS);
		sc->status = bus_space_read_1 (sc->bar0_bt, sc->bar0_bh, READER_STATUS_BITS);

		sc->available = 1;

		wakeup (sc);

		if (SEL_WAITING (&sc->selinfo))
			selwakeup (&sc->selinfo);

		break;

	case 0x50:
		/* TC comparator match found (must enable 2Eh bit 5 first!). */
		break;

	default:
		/* Echo of the command code */
		wakeup (sc);
		break;
	}

	bus_space_write_1 (sc->bar0_bt, sc->bar0_bh, HOST_TO_BOARD_MAILBOX_PORT, CMD_ACKNOWLEDGE);
	bus_space_read_1 (sc->bar0_bt, sc->bar0_bh, HOST_TO_BOARD_MAILBOX_PORT);
}

static int
adr_probe (device_t dev)
{
	if ((pci_get_vendor (dev) == ADRIENNE_PCI_VENDOR) && (pci_get_device (dev) == ADRIENNE_PCI_DEVICE))
		return 0;

	return ENXIO;
}

static int
adr_attach (device_t dev)
{
	struct adr_sc *sc = (struct adr_sc *) device_get_softc (dev);

	sc->dev = dev;

	int unit = device_get_unit (dev);

	sc->cdev = make_dev (&adr_cdevsw, unit, UID_ROOT, GID_OPERATOR, 0666, "adr%d", unit);

	sc->cdev->si_drv1 = sc;

	sc->bar0_id = PCIR_BAR (0);

	sc->bar0_res = bus_alloc_resource_any (dev, SYS_RES_IOPORT, &(sc->bar0_id), RF_ACTIVE);

	if (sc->bar0_res == NULL) {
		device_printf (dev, "bus_alloc_resource_any failed (BAR0)\n");

		return ENXIO;
	}

	sc->bar0_bt = rman_get_bustag (sc->bar0_res);
	sc->bar0_bh = rman_get_bushandle (sc->bar0_res);

	sc->irq_id = 0x0;

	sc->irq_res = bus_alloc_resource_any (dev, SYS_RES_IRQ, &(sc->irq_id), RF_SHAREABLE | RF_ACTIVE);

#if __FreeBSD_version < 700000

	if (bus_setup_intr (dev, sc->irq_res, INTR_TYPE_MISC, adr_intr, sc, &(sc->irq_cookie))) {
		device_printf (dev, "bus_setup_intr failed\n");

		return ENXIO;
	}

#else

	if (bus_setup_intr (dev, sc->irq_res, INTR_TYPE_MISC, NULL, adr_intr, sc, &(sc->irq_cookie))) {
		device_printf (dev, "bus_setup_intr failed\n");

		return ENXIO;
	}

#endif

	mtx_init (&sc->mutex, "adr_mtx", NULL, MTX_DEF);

	device_printf (dev, "vendor code 0x%02X%02X, board number 0x%02X%02X, revision 0x%c%c\n",
                       bus_space_read_1 (sc->bar0_bt, sc->bar0_bh, AEC_VENDOR_CODE_HIGH_BYTE),
                       bus_space_read_1 (sc->bar0_bt, sc->bar0_bh, AEC_VENDOR_CODE_LOW_BYTE),
                       bus_space_read_1 (sc->bar0_bt, sc->bar0_bh, PCI_TC_BOARD_NUMBER_CODE_HIGH_BYTE),
                       bus_space_read_1 (sc->bar0_bt, sc->bar0_bh, PCI_TC_BOARD_NUMBER_CODE_LOW_BYTE),
                       bus_space_read_1 (sc->bar0_bt, sc->bar0_bh, SOFTWARE_REVISION_LETTER),
                       bus_space_read_1 (sc->bar0_bt, sc->bar0_bh, SOFTWARE_REVISION_NUMBER));

	/* Enable PCI Interrupt */
	bus_space_read_4 (sc->bar0_bt, sc->bar0_bh, SPECIAL_BOARD_STATUS_REGISTER_BASE);
	bus_space_write_4 (sc->bar0_bt, sc->bar0_bh, SPECIAL_BOARD_STATUS_REGISTER_BASE, 0x00000010);

	reset_board (sc);

	/* Enable TC Interrupt */
	bus_space_write_1 (sc->bar0_bt, sc->bar0_bh, INTERRUPT_CONTROL_BIT_MAP, INTERRUPT_CONTROL_TC_READER);
	bus_space_read_1 (sc->bar0_bt, sc->bar0_bh, INTERRUPT_CONTROL_BIT_MAP);

	int error = 0;

	if ((error = send_command (sc, CMD_IDLE_MODE)))
		return error;

	return error;
}

static int
adr_detach (device_t dev)
{
	struct adr_sc *sc = (struct adr_sc *) device_get_softc (dev);

	if (bus_teardown_intr (dev, sc->irq_res, sc->irq_cookie))
		device_printf (dev, "bus_teardown_intr failed\n");

	if (bus_release_resource (dev, SYS_RES_IRQ, sc->irq_id, sc->irq_res))
		device_printf (dev, "bus_release_resource failed\n");

	if (bus_release_resource (dev, SYS_RES_IOPORT, sc->bar0_id, sc->bar0_res))
		device_printf (dev, "bus_release_resource failed (BAR0)\n");

	destroy_dev (sc->cdev);

	mtx_destroy (&sc->mutex);

	return 0;
}

static device_method_t adr_methods[] = {
	/* Device interface */
	DEVMETHOD (device_probe,  adr_probe),
	DEVMETHOD (device_attach, adr_attach),
	DEVMETHOD (device_detach, adr_detach),
#if __FreeBSD__ >= 900000
	DEVMETHOD_END
#else
	{ NULL, NULL }
#endif
};

static driver_t adr_driver = {
	"adrienne",
	adr_methods,
	sizeof (struct adr_sc),
};

DRIVER_MODULE (adrienne, pci, adr_driver, adr_devclass, NULL, NULL);

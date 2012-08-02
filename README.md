freebsd-adrienne
================

Here you will find a simple driver for Adrienne PCI timecode readers
(http://www.adrielec.com/). It is not officially endorsed by Adrienne
Electronics, so you are on your own here...

The driver implementation is very simple and was derived from the board
documentation that you will find in the *doc* directory. Adrienne
Electronics includes full documentation with each product so you will be
able to adapt it to your hardware if you have a more recent version.

Unfortunately I no longer own Adrienne hardware so I am not able to test
further changes. However, the driver was actually used in production for a
in-house project and it managed to survive quite a few days of execution.
Then the project was stopped, but not because of a driver fault :)

In the unlikely case that you own an actual Adrienne timecode reader you
may give it a spin, otherwise it should get you started if you plan to
develop a simple character device driver in FreeBSD (do yourself a favor
and read "FreeBSD Device Drivers" by Joseph Kong before writing anything -
this driver was written well before reading it!)

How to install
==============

Run `make install` from the project directory:

	# make install
	install -o root -g wheel -m 555 adrienne.ko /boot/kernel
	kldxref /boot/kernel
	#

To clean up after building:

	# make clean cleandepend

Remember to do this when you switch architecture while using the same
source tree (i.e. when building from i386 and amd64 boxes over a NFS
share).

To load or unload the locally compiled driver:

	# make load

and

	# make unload

To load or unload the installed driver:

	# kldload adrienne

and

	# kldunload adrienne

To uninstall the driver:

	# rm -f /boot/kernel/adrienne.ko
	# kldxref /boot/kernel

To load at every boot, add `adrienne_load="YES"` to your `/boot/loader.conf`.

You might also create a package to be used with `pkg_add` and `pkg_delete`.
Be sure to use matching FreeBSD versions when deploying binaries.

Bugs
----

- Possibly still a few.
- Missing documentation (especially the read block format that you'll have
  to guess by reading the Adrienne docs).
- Most importantly, the source code does not conform to *style(9)* (even if
  it was improved a bit before publishing).

<!-- Local Variables: -->
<!-- fill-column: 75 -->
<!-- End: -->

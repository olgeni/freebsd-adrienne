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

Bugs
----

- Possibly still a few.
- Missing documentation (especially the read block format that you'll have
  to guess by reading the Adrienne docs).
- Most importantly, the source code does not conform to *style(9)* (even if
  it was improved a bit before publishing).

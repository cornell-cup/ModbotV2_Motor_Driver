#include "uart2.h"

// Code adapted from:
// https://github.com/cheaven/edison_native/blob/master/edisonlib/core/arduino/TTYUART.cpp

int detach_console(void) {

	/* We need to detach the current users of the system console, which we
	 * assume to be (i) kernel logger and (ii) getty or a command shell
	 * Here's how we're going to do it:
	 * - Change the kernel log level to filter log messages to the console
	 * - Modify inittab to stop it respawning gettys on the console
	 *   - Add a mechanism to restore normal behaviour on next reboot
	 * - Tell init to reload the modified inittab
	 * - Kill any processes still using the console
	 *   - Send them the KILL signal (bash doesn't seem to respond to TERM).
	 */

	int ret = 0;
	char cmd[256];
	char *ttydev;
	char _tty_name[] = "/dev/ttyMFD2";

	if (_tty_name == NULL)
		return -1;

	// Check if we've been here already
	if (!access("/etc/inittab.restore", F_OK))
		return 0;

	ttydev = strstr(_tty_name, "tty");
	if (!ttydev) {
		fprintf(stderr, "Invalid TTY name %s\n", _tty_name);
		return -1;
	}

	// Change the kernel log level
	system("export PATH=/bin:/usr/bin; dmesg -n 1");

	// We're about to change inittab, but we need our changes to be rolled
	// back on the next reboot.  So first, we make a backup, then we add a
	// rollback mechanism inside inittab itself.
	system("/bin/cp -f /etc/inittab /etc/inittab.restore");
	snprintf(cmd, sizeof(cmd),
		 "/bin/sed -i -e '/^\\(S.*getty.*%s$\\)/i\\\n"
		 "# Disabling getty to allow Arduino sketch to use %s and\n"
		 "# adding sysinit commands to restore normal behavior on next boot\n"
		 "' /etc/inittab",
		 ttydev, ttydev);
	system(cmd);
	snprintf(cmd, sizeof(cmd),
		 "/bin/sed -i -e '/^\\(S.*getty.*%s$\\)/i\\\n"
		 "res1:2345:sysinit:/bin/mv -f /etc/inittab.restore /etc/inittab\n"
		 "' /etc/inittab",
		 ttydev);
	system(cmd);
	snprintf(cmd, sizeof(cmd),
		 "/bin/sed -i -e '/^\\(S.*getty.*%s$\\)/i\\\n"
		 "res2:2345:sysinit:/sbin/telinit q\n"
		 "' /etc/inittab",
		 ttydev);
	system(cmd);

	// Modify inittab to stop use of this console device for user log-in
	snprintf(cmd, sizeof(cmd),
		 "/bin/sed -i -e 's/^\\(S.*getty.*%s$\\)/#\\1/' /etc/inittab",
		 ttydev);
	system(cmd);

	// Tell init to reload its configuration
	system("/sbin/telinit q");

	// Forcibly kill any processes still using the tty device.
	snprintf(cmd, sizeof(cmd),
		 "export PATH=/bin:/usr/bin; "
		 "lsof | grep '%s' | cut -f 1 | sort -u | xargs -r kill -9",
		 ttydev);
	system(cmd);

	return 0;
}
int reattach_console(void){

	// Restore the kernel log level
	system("/bin/dmesg -n 7");

	// Restore the original inittab and kick-start getty on the TTY
	system("/bin/mv -f /etc/inittab.restore /etc/inittab");
	system("/sbin/telinit q");

	return 0;
}

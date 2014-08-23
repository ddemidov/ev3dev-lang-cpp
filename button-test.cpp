#include <fcntl.h>
#include <linux/input.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "ev3dev.h"

#define BITS_PER_LONG (sizeof(long) * 8)

bool test_bit(int bit, const long unsigned *bytes) {
	/* bit in bytes is 1 when released and 0 when pressed */
	return !(bytes[bit / BITS_PER_LONG] & 1 << (bit % BITS_PER_LONG));
}

int main() {
	unsigned long buf[(KEY_CNT + BITS_PER_LONG - 1) / BITS_PER_LONG];
	int fd;
	bool up, down, left, right, enter, escape;

	fd = open("/dev/input/by-path/platform-gpio-keys.0-event", O_RDONLY);
	while (escape == 0) {
		if (ioctl(fd, EVIOCGKEY(sizeof(buf)), buf) < 0) {
			/* handle error */
		} else {

			up = test_bit(KEY_UP, buf);
			down = test_bit(KEY_DOWN, buf);
			left = test_bit(KEY_LEFT, buf);
			right = test_bit(KEY_RIGHT, buf);
			enter = test_bit(KEY_ENTER, buf);
			escape = test_bit(KEY_ESC, buf);

			printf("up:%d down:%d left:%d right:%d enter:%d esc:%d\n", up, down, left, right, enter, escape);
			usleep(100000);
		}
	}
	close(fd);
}

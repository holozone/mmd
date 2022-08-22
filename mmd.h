#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <string>

// Hilfsfunktionen
int canonmode(int onoff) {
	static struct termios oldt, newt;
	if (onoff == 1) {
		tcgetattr(STDIN_FILENO, &oldt);
		newt = oldt;
		newt.c_lflag &= ~(ICANON|ECHO);
		if (tcsetattr( STDIN_FILENO, TCSANOW, &newt) == -1) printf("Setting non canon mode failed \n");
		else printf("success: canon mode is off \n");
	} else {
		/*restore the old settings*/
		if (tcsetattr(STDIN_FILENO, TCSANOW, &oldt)  == -1) printf("restore canon mode failed \n");
		else printf("success: canon mode is restored \n");
	}
	return 0;
}

// Nonblocking test if a key was hit
bool keyhit() {
	struct timeval tv;
	int retval;
	/* Wait up to 100 µseconds. */
	tv.tv_sec = 0;
	tv.tv_usec = 100;
	fd_set readfds;
	FD_ZERO(&readfds);
	FD_SET(0,&readfds);
	retval = select(1, &readfds, NULL, NULL, &tv);
	if (retval != 0) return true;
	return false;
}

std::string input;
int collect_input(char ic) {
	switch (ic) {
	case 10:	
		break;
	case 27:
		input = "";
		break;
	default:
		input += ic;
		printf("%c", ic);
		return 1;
	}
	return 0;
}

#include "konami.h"

int button_fd;

void btn_gpio_init(void) 
{
	char path[60];
	int fd;
	int i;

	sprintf(path, "%s/export", GPIO_PATH);
	fd = open(path, O_WRONLY);
	if (fd < 0) {
		printf("Can't open export file\n");
		return;
	}

	write(fd, "154", 3);
	write(fd, "155", 3);

	close(fd);

	for (i = BUTTON0; i >= BUTTON1; i--) {
		sprintf(path, "%s/gpio%d/direction", GPIO_PATH, i);
		
		printf("opening %s..\n", path);
		fd = open(path, O_WRONLY);
		if (fd < 0) {
			printf("Can't open direction file for gpio%d\n", i);
			return;
		}

		write(fd, "in", 2);
		close(fd);
	}

}

#if 0
int main(void) 
{
	char path[50];

	sprintf(path, "%s/gpio%d/value", GPIO_PATH, num);
	if (access(path, F_OK) < 0) btn_gpio_init();

	sprintf(path, "%s/gpio%d/value", GPIO_PATH, BUTTON0);
	button_fd = open(path, O_RDONLY);
	if (button_fd < 0) { 
		printf("Cannot open BUTTON0\n"); 
		exit(EXIT_FAILURE);
	}

	for(;;) {
		monitor_button(BUTTON0);
	}

	return 0;
}
#endif

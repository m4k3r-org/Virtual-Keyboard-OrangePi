
// Server side C/C++ program to demonstrate Socket programming 
#include <unistd.h> 
#include <stdio.h> 
#include <sys/socket.h> 
#include <stdlib.h> 
#include <netinet/in.h> 
#include <string.h> 

#include <X11/Xlib.h>
#include <X11/keysym.h>
#include <X11/extensions/XTest.h>

#define PORT 8080 

int main(int argc, char const *argv[]) 
{ 
	Display *display;
	unsigned int keycode;
	display = XOpenDisplay(NULL);


    int server_fd, new_socket, valread; 
    struct sockaddr_in address; 
    int opt = 1; 
    int addrlen = sizeof(address); 
    char buffer[1024] = {0}; 
       
    // Creating socket file descriptor 
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) 
    { 
        perror("socket failed"); 
        exit(EXIT_FAILURE); 
    } 
       
    // Forcefully attaching socket to the port 8080 
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, 
                                                  &opt, sizeof(opt))) 
    { 
        perror("setsockopt"); 
        exit(EXIT_FAILURE); 
    } 
    address.sin_family = AF_INET; 
    address.sin_addr.s_addr = INADDR_ANY; 
    address.sin_port = htons( PORT ); 
       
    // Forcefully attaching socket to the port 8080 
    if (bind(server_fd, (struct sockaddr *)&address,  
                                 sizeof(address))<0) 
    { 
        perror("bind failed"); 
        exit(EXIT_FAILURE); 
    } 
    if (listen(server_fd, 3) < 0) 
    { 
        perror("listen"); 
        exit(EXIT_FAILURE); 
    } 
    if ((new_socket = accept(server_fd, (struct sockaddr *)&address,  
                       (socklen_t*)&addrlen))<0) 
    { 
        perror("accept"); 
        exit(EXIT_FAILURE); 
    }
    int count = 0;
    valread = 1;
    while (valread > 0) 
    { 
    	valread = read( new_socket , buffer, 1024); 
    	printf("Pressed key: %s \n",buffer );
		count ++;


		char pressedKey = buffer[0];

		switch(pressedKey)
		{
		 case 'E':
		 	keycode = XKeysymToKeycode(display, XK_Escape);
			break;
		 case '<':
		 	keycode = XKeysymToKeycode(display, XK_BackSpace);
		 	break;
		 case '^':
		 	keycode = XKeysymToKeycoe(display, XK_Caps_Lock);
		 	break;
		 case '\n':
		 	keycode = XKeysymToKeycode(display, XK_Return);
		 	break;
		 default:
		 	keycode = XKeysymToKeycode(display, pressedKey);
		 	break;
		}
		
		XTestFakeKeyEvent(display, keycode, True, 0);
		XTestFakeKeyEvent(display, keycode, False, 0);
		XFlush(display);

    } 
    return 0; 
} 


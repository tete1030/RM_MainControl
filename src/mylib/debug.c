#include <stdio.h>
#include "debug.h"

#ifdef DEBUG
void __trace(char* func_name, char* msg)
{
	printf("[%s] %s\r\n", func_name, msg);
}
#endif

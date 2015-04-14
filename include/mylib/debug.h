#ifndef __DEBUG_H__
#define __DEBUG_H__
#ifdef DEBUG

void __trace(char* func_name, char* msg);
//#define TRACE(a,b) __trace(a,b)

#else

//#define TRACE(a,b) 0

#endif //DEBUG
#endif

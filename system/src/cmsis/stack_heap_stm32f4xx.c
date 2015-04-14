#define STACK_BASE 0x20020000
#define STACK_SIZE 0x400
#define HEAP_SIZE 0x200
#define HEAP_BASE (STACK_BASE - STACK_SIZE - HEAP_SIZE)

#if defined ( __GNUC__ )

/*
 void volatile dummy_function(void)
 {
 asm volatile (
 ".global __stack\n\t"
 ".global _Main_Stack_Size\n\t"
 ".global _Main_Stack_Limit\n\t"
 ".global _Heap_Begin\n\t"
 ".global _Heap_Limit\n\t"
 ".equ __stack, %0\n\t"
 ".equ _Main_Stack_Size, %1\n\t"
 ".equ _Main_Stack_Limit, %2\n\t"
 ".equ _Heap_Begin, %3\n\t"
 ".equ _Heap_Limit, %4\n\t"::
 "r" (STACK_BASE),
 "r" (STACK_SIZE),
 "r" (STACK_BASE-STACK_SIZE),
 "r" (HEAP_BASE),
 "r" (HEAP_BASE+HEAP_SIZE)
 );
 }*/

#elif defined ( __CC_ARM )

__asm static void dummy_function(void)
{

#if defined ( __MICROLIB )

	EXPORT __stack
	EXPORT _estack
	EXPORT __initial_sp
	EXPORT __heap_base
	EXPORT __heap_limit
	EXPORT _Heap_Begin
	EXPORT _Heap_Limit

_estack EQU STACK_BASE
__stack EQU STACK_BASE
__initial_sp EQU STACK_BASE
__heap_base EQU HEAP_BASE
__heap_limit EQU (HEAP_BASE + HEAP_SIZE)
_Heap_Begin EQU HEAP_BASE
_Heap_Limit EQU (HEAP_BASE + HEAP_SIZE)

#else
	IMPORT __use_two_region_memory
	EXPORT __user_initial_stackheap

__user_initial_stackheap

	LDR R0, = HEAP_SIZE
	LDR R1, = STACK_BASE
	LDR R2, = (HEAP_BASE + HEAP_SIZE)
	LDR R3, = (STACK_BASE - STACK_SIZE)
	BX LR

#endif
}

#endif

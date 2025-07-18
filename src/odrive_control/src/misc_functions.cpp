#include <stdio.h>
#include <sys/time.h>
#include <odrive_control/misc_functions.hpp>
#define DEBUG false 
/*bit masking function to seperate upper 6 (node_id) and lower 5 (cmd_id) bits of the can_id in the can frame*/
void bit_masking(can_frame_odrive &can_frame)
{
	
        if (DEBUG) printf("%s >> in bit_masking function\n", __func__);
	__u32 cmd_id_mask = 0x1F;
	__u32 node_id_mask = 0x7E0;
	can_frame.cmd_id = (cmd_id_mask & can_frame.cframe.can_id) ;
	can_frame.node_id = (node_id_mask & can_frame.cframe.can_id) >> 5;	
    
}


#include <pthread.h>
#include <stdio.h>

void PrintThreadID()
{
    // Get the thread ID of the current thread
    pthread_t threadId = pthread_self();
    printf("Thread ID: %lu\n", (unsigned long)threadId);

    // ... Rest of the thread's functionality ...

}



#include <execinfo.h>
#include <stdio.h>
void printStackTrace() {
    void* callstack[256];
    int frames = backtrace(callstack, 256);
    char** symbols = backtrace_symbols(callstack, frames);
    if (symbols == NULL) {
        perror("backtrace_symbols");
        return;
    }
    for (int i = 0; i < frames; i++) {
        printf("%s\n", symbols[i]);
    }
    free(symbols);
}
void handle_segfault(int sig)
{
  pthread_t self = pthread_self();
  printf("%s >> Segmentation fault in thread  %ld\n",__func__, (long) self);
  printf("%s >> \n\n**********Printing Stack Trace ************\n\n",__func__) ;
  printStackTrace() ;
  printf("%s >> \n ** Exiting ********* \n", __func__);
  exit(-1) ;
}


double CurrentTimeInMillisec()
{
        if (DEBUG) printf("%s >> in CurrentTimeInMillisec function\n", __func__);
	struct timeval timeVar ;
	gettimeofday(&timeVar, NULL);
	double s1 = (long) (timeVar.tv_sec) * 1000;
	double s2 = (timeVar.tv_usec/1000);
	return(s1+s2);
}
bool CheckForTimeOut(double TimeOutValue, double StartTime) 
{
        double CurrentTime = CurrentTimeInMillisec() ;
        if ( (CurrentTime - StartTime) > TimeOutValue )
           return( true) ; 
        else return(false);
}    

/*architecture dependent float to byte converter (Endian-ness)
*/
void float2Bytes(float float_variable, uint8_t * bytes_temp){ 
  if (DEBUG) printf("%s >> in float2Bytes function\n", __func__);
  union {
    float a;
    uint8_t bytes[4];
  } thing;
  thing.a = float_variable;
  memcpy(bytes_temp, thing.bytes, 4);
}

//architecture dependent byte to float converter (Endian-ness)
void bytes2Float(uint8_t * bytes_temp, float* float_variable){ 
  if (DEBUG) printf("%s >> in bytes2Float function\n", __func__);
  union {
    float a;
    uint8_t bytes[4];
  } thing;
  //swap around for different endlian
  thing.bytes[0] = bytes_temp[0];
  thing.bytes[1] = bytes_temp[1];
  thing.bytes[2] = bytes_temp[2];
  thing.bytes[3] = bytes_temp[3];
  *(float_variable) = thing.a;
  if (DEBUG) printf("b2f float = %f, bytes_temp: %x %x %x %x \n",thing.a, bytes_temp[0],bytes_temp[1],bytes_temp[2],bytes_temp[3]);
  if (DEBUG) printf("b2f *float = %f, thing.bytes: %x %x %x %x \n",*(float_variable), thing.bytes[0],thing.bytes[1],thing.bytes[2],thing.bytes[3]);
		
}


/* Function to reverse arr[] from start to end
void rvereseArray(auto arr[], int start, int end) 
{ 
    while (start < end) 
    { 
        auto temp = arr[start];  
        arr[start] = arr[end]; 
        arr[end] = temp; 
        start++; 
        end--; 
    }  
}      
  
*/

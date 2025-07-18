#include<stdio.h>
#include<stdlib.h>
#include <time.h>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <stdbool.h>
#include <pthread.h>                                                            

int ChildProcessId ;
bool CameraReady = false;
time_t curtime;

void signal_handlerSIGUSR2(int signo)
{
    int pid ;

    //pid = getpid();

    if(signo == SIGUSR2)
    {
        //printf("In signalHandler with SIGUSR1\n");
        //if(pid == parent)
        //    kill(pid_1,SIGUSR1);
           time(&curtime);
           //kill(pid_2,SIGUSR2);
           printf("YanthraMoveCommunicationInterface signal_handlerSIGUSR2 handler Received signal at %s \n", ctime(&curtime));
           CameraReady = true ;
    }
}


int value = 5;

// This is the function called on receipt of signal(cntl+c)
void sig_handler (int sig)
{
    printf("\n YanthraCommunicationInterface sig_handler :: This is signal handler for signal %d\n", sig);
    if( sig == SIGUSR2) {
        CameraReady = true;
        printf("YanthraCommunicationInterface : sig_handler Processing SIGUSR2\n") ;
    }
}

void catch_alarm( int Alarmsignal)
{
    signal (Alarmsignal, catch_alarm);
}

// NOT TESTED YET
int WaitForCameraReady()
{
    /*
    struct sigaction act;
    act.sa_handler = sig_handler;
    sigemptyset(&act.sa_mask);
    sigaddset(&act.sa_mask, SIGUSR2);
    act.sa_flags = 0;

    sigset_t sig_term_kill_mask;
    sigaddset(&sig_term_kill_mask, SIGUSR2);
    //sigaddset(&sig_term_kill_mask, SIGKILL);

    //sigaction(SIGINT, &act, NULL);
    */

    ////// Critical section /////////
    value++;

    printf("\n YanthraMove->WaitForCameraReady : Value is %d\n", value);
    int SigWaitReturnValue ;
    if (CameraReady == false) {
        printf("YanthraMove->WaitForCameraReady Waiting for signals SIGUSR2 \n");
        /* sigsuspend(&sig_term_kill_mask); */
        sigset_t   set;
        sigemptyset(&set); 
        if (sigaddset (&set, SIGALRM) == -1) {
            perror("YanthraMove YanthraMove->WaitForCameraReady() : Sigaddset error SIGALRM");                                                  
            pthread_exit((void *)1);                                                    
        }
        if(sigaddset(&set, SIGUSR2) == -1) {                                           
            perror("YanthraMove YanthraMove->WaitForCameraReady() : Sigaddset error SIGUSR2");                                                  
            pthread_exit((void *)1);                                                    
        } 
        //alarm(1);
        //Wait for Signal SIGUSR2 or timeout
        if(sigwait(&set,&SigWaitReturnValue )) {                                                 
            perror("YanthraMove YanthraMove->WaitForCameraReady(): Sigwait error");                                                    
            //pthread_exit((void *)2);                                                    
	} 
        switch (SigWaitReturnValue) {
            case SIGUSR1:
                printf("YanthraMove->WaitForCameraReady Recieved SIGUSR1 \n");
                break;
            case SIGUSR2:
                printf("YanthraMove->WaitForCameraReady Recieved SIGUSR2 \n");
                // driver exit
                CameraReady = true;
                break;
            case SIGALRM:
                printf("YanthraMove->WaitForCameraReady Recieved SIGALRM\n");
                break;
            case SIGTTOU:
                printf("YanthraMove->WaitForCameraReady Recieved SIGTTOU\n");
                break;
            default:
                printf("YanthraMove->WaitForCameraReady Recieved Default\n");
                break;
        }                                                                   
        printf("\n YanthraMove->WaitForCameraReady Return from suspended successful\n");
    }
    return 0;
}


int main()
{
    //// Code for Blocking and Unblocking Signals ****/
  sigset_t set,set2;
  struct sigaction sigs;
  // Install Signal Handler for SIGUSR2 and SIGALRM
    signal(SIGUSR2,signal_handlerSIGUSR2);
    signal(SIGALRM,signal_handlerSIGUSR2);

  //sigs.sa_handler = signal_handlerSIGUSR2 ;
  //sigemptyset(&sigs.sa_mask);
  //sigs.sa_flags=SA_ONESHOT;
  // sigaction(SIGUSR2, &sigs,0);
  sigemptyset(&set);
  sigemptyset(&set2);
  //sigaddset(&set,SIGUSR2);
  if (sigaddset (&set, SIGALRM) == -1) {
            perror("YanthraMove YanthraMove->main() : Sigaddset error SIGALRM");                                                  
            pthread_exit((void *)1);                                                    
  }
  if(sigaddset(&set, SIGUSR2) == -1) {                                           
            perror("YanthraMove YanthraMove->main() : Sigaddset error SIGUSR2");                                                  
            pthread_exit((void *)1);                                                    
  } 
 
  printf("YanthraMoveCommunicationInterface->main() ***BLOCKING** SIGUSR2, SIGALRM at  %s\n",ctime(&curtime));
  if(sigprocmask(SIG_BLOCK, &set, NULL)==0){
    printf("YanthraMoveCommunicationInterface->main() ***BLOCKED** SIGUSR2, SIGALRM at  %s\n",ctime(&curtime));
  }
  sigpending(&set2);
  if (sigismember(&set2,SIGUSR2)==1)
  {
    printf("The SIGUSR2 signal is blocked\n");  //it should print this
  }
  if (sigismember(&set2,SIGALRM)==1)
  {
    printf("The SIGALRM signal is blocked\n");  //it should print this
  }

  //printf("YanthraMoveCommunicationInterface Going to Wait for 2 seconds at   %s\n",ctime(&curtime));
  //sleep(2);
  //printf("YanthraMove -> main() : Sending SIGUSR2 to itself\n");
  // kill(getpid(),SIGUSR2); //the signal shouldn't work

  printf("YanthraMoveCommunicationInterface Going to Wait for 2 seconds AGAIN at   %s\n",ctime(&curtime));
  sleep(2);

/////// End of Sample Code

    int pid_1 ;
    int parentPID = getpid();

    /*
    sigset_t base_mask;
    sigset_t  waiting_mask;
    sigemptyset (&base_mask);
    sigaddset (&base_mask, SIGUSR2);
    */
    //sigaddset (&base_mask, SIGTSTP);
    //
    printf("YanthraMoveCommunicationInterface->main(): Parent ProcessID : %d \n",parentPID) ;
    printf(" Forking a child in YanthraMoveCommunicationInterface\n") ;
    if ( (ChildProcessId = fork() ) < -1) {
        printf("YanthraMoveCommunicationInterface  ChildProcess ID %d, childProcess FAILED \n", ChildProcessId) ;
        exit(0);
    }
    else if (ChildProcessId == 0) {
        // Execution is in the child process CottonDetectCommunicationInterface
        // We need to Block all Interrupts before the child process unblock the interrupts
        printf(" Execution in ChildProcess proceesID : %d \n", getpid()) ;
        printf("Spawning ./OakDCommunicationInterface.py\n")  ;
       // execl ( "./CottonDetectCommunicationInterface", (char *) NULL);
       // execl ( "./OakDCommunicationInterface.py", (char *) NULL);
        execl ( "./CottonDetect.py", (char *) NULL);
        printf(" ERROR Could Not Spawn python3 ./OakDCommunicationInterface.py\n");
        perror("Exec failed");
        exit(EXIT_FAILURE);
        exit(-1);
    }

/*
    else
    {

        // Block user interrupts while doing other processing. 
        // UNBlock all user interrupts now .
        printf(" YanthraMoveCommunicationInterface MASKING SIGUSR2  %s\n",ctime(&curtime));
        sigprocmask (SIG_SETMASK, &base_mask, NULL);

        printf("YanthraMoveCommunicationInterface  ChildProcess ID : %d \n", ChildProcessId) ;
        time(&curtime) ;
        printf(" YanthraMoveCommunicationInterface  Checking  Camera Status at   %s\n",ctime(&curtime));
        if (!CameraReady) printf("YanthraMoveCommunicationInterface  CAMERA NOT READY at %s\n",ctime(&curtime)) ;
        else {
            printf("YanthraMoveCommunicationInterface : ERROR, CameraReady Already True\n");
            exit(-1);
        }
    }
*/

  printf("YanthraMoveCommunicationInterface->main() UNBLOCKING SIGUSR2, SIGALRM at  %s\n",ctime(&curtime));
  if(sigprocmask(SIG_UNBLOCK, &set, NULL)==0){
   printf("YanthraMoveCommunicationInterface->main() UNBLOCKED SIGUSR2, SIGALRM at  %s\n",ctime(&curtime));
  }
  printf("YanthraMoveCommunicationInterface Going to Wait for 2 seconds at   %s\n",ctime(&curtime));
 
  printf("YanthraMoveCommunication - main() CameraReady %s \n", CameraReady ? "TRUE" : "FALSE");
   while(1)
    {
        //printf("YanthraMoveCommunicationInterface :: Pausing for SIGUSR2 signal from CottonDetectProcess\n") ;
        //pause() ;
        // Sending Redundant Signal to Child Process 
        //kill(ChildProcessId, SIGUSR1);
        WaitForCameraReady() ;
        if ( CameraReady ) {
            time(&curtime) ;
            printf("YanthraMoveCommunicationInterface Camera Ready at   %s\n",ctime(&curtime));
            CameraReady = false ;
            printf("YanthraMoveCommunicationInterface Sleeping for 10 seconds\n");
            sleep ( 10 ) ;
            //printf("YanthraMoveCommunicationInterface Killing ChildProcess %d\n",ChildProcessId);
            //kill(ChildProcessId, SIGUSR1);
            time(&curtime) ;
            printf("YanthraMoveCommunicationInterface Sent Signal to cottonDetect at  %s\n",ctime(&curtime));
        }
        kill(ChildProcessId, SIGUSR1);
    }
}

#include <errno.h>
#include <signal.h>
/*
int cs = 0;
     int value = 5;

     // This is the function called on receipt of signal(cntl+c) 
     void sig_handler (int sig)
     {
        printf("\n This is signal handler for signal %d\n", sig);
        cs =1;
     }
*/


/*
int MethodToWaitOnSignal()
    {
        struct sigaction act;
        act.sa_handler = sig_handler;
        sigemptyset (&act.sa_mask);
        sigaddset (&act.sa_mask, SIGPIPE);
        act.sa_flags = 0;

        sigset_t sig_term_kill_mask;
        sigaddset (&sig_term_kill_mask, SIGTERM);
        sigaddset (&sig_term_kill_mask, SIGKILL);

        sigaction (SIGINT, &act, NULL);

        ////// Critical section /////////
        value++;
        printf ("\n Value is %d\n", value);
        while (cs == 0)
            sigsuspend (&sig_term_kill_mask);

        printf("\n Return from suspended successful\n");
        int ret = sigismember (&act.sa_mask, SIGPIPE);
        if (ret)
            printf("\n Old Mask that is SIGPIPE is restored after return from sigsuspend\n");
        else
            printf("\n Old Mask that is SIGPIPE failed to restored after return from sigsuspend\n");

         return 0; }

         */

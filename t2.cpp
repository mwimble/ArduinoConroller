#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
 
struct TFoo {
    int vMain;
    int vChild;
};

void *thread1Function(void *arg) {
    TFoo* myGlob = (TFoo*) arg;
    int counter = 0;
    int i = 0;
    for (; i < 5; ++i) {
        myGlob->vChild++;
        printf("Thread Child: counter=%d vChild=%d, vMain=%d\n", ++counter, myGlob->vChild, myGlob->vMain );
        usleep(1);
    }

    return 0;
}
 
void *thread2Function(void *arg) {
    TFoo* myGlob = (TFoo*) arg;
    int counter = 0;
    int i = 0;
    for (; i < 5; ++i) {
        myGlob->vMain++;
        printf("Thread Mail: counter=%d vChild=%d, vMain=%d\n", ++counter, myGlob->vChild, myGlob->vMain );
        usleep(1);
    }

    return 0;
}
 
void *print_message_function( void *ptr );
 
int main(int argc, char **argv) {
     pthread_t thread1, thread2;
     const char *message1 = "Thread 1";
     const char *message2 = "Thread 2";
     int  iret1, iret2;
 
    TFoo myGlob;
    myGlob.vMain = 0;
    myGlob.vChild = 0;

    /* Create independent threads each of which will execute function */
 
     iret1 = pthread_create( &thread1, NULL, thread1Function, (void*) &myGlob);
     if (iret1) {
         fprintf(stderr,"Error - pthread_create() return code: %d\n",iret1);
         exit(EXIT_FAILURE);
     }
 
     iret2 = pthread_create( &thread2, NULL, thread2Function, (void*) &myGlob);
     if(iret2) {
         fprintf(stderr,"Error - pthread_create() return code: %d\n",iret2);
         exit(EXIT_FAILURE);
     }
 
     printf("pthread_create() for thread 1 returns: %d\n",iret1);
     printf("pthread_create() for thread 2 returns: %d\n",iret2);
 
     /* Wait till threads are complete before main continues. Unless we  */
     /* wait we run the risk of executing an exit which will terminate   */
     /* the process and all threads before the threads have completed.   */
 
     pthread_join( thread1, NULL);
     pthread_join( thread2, NULL);
 
    printf("Program join exit\n");
     exit(EXIT_SUCCESS);
     return 0;
}

void *print_message_function( void *ptr )
{
     char *message;
     message = (char *) ptr;
     printf("%s \n", message);
     return 0;
}

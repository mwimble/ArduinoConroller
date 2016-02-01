#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <unistd.h>

struct TFoo {
	int vMain;
	int vChild;
};

int main(int argc, char **argv) {
	printf("--beginning of program\n");

	TFoo *myGlob = (TFoo*) mmap(NULL, sizeof *myGlob, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANON, -1, 0);

	myGlob->vMain = 0;
	myGlob->vChild = 0;

    int counter = 0;
    pid_t pid = fork();

    if (pid == 0)
    {
        // child process
        int i = 0;
        for (; i < 5; ++i)
        {
        	myGlob->vChild++;
            printf("child process: counter=%d vChild=%d, vMain=%d\n", ++counter, myGlob->vChild, myGlob->vMain );
            usleep(50);
        }
    }
    else if (pid > 0)
    {
        // parent process
        int j = 0;
        for (; j < 5; ++j)
        {
        	myGlob->vMain++;
            printf("parent process: counter=%d vChild=%d, vMain=%d\n", ++counter, myGlob->vChild, myGlob->vMain );
            usleep(50);
        }
    }
    else
    {
        // fork failed
        printf("fork() failed!\n");
        return 1;
    }

    printf("--end of program--\n");

    return 0;
}


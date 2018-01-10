#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include "dcurl.h"

/* number of task that CPU can execute concurrently */
#define MAX_CPU_THREAD 0

/* number of task that GPU can execute concurrently */
#define MAX_GPU_THREAD 5

/* mutex protecting critical section */
pthread_mutex_t mtx;

/* Semaphore that blocks excessive task*/
sem_t notify;

/* check whether dcurl is initialized */
int isInitialized = 0;

/* Respective number for Mutex */
int cpu_mutex_id[MAX_CPU_THREAD] = {0};
int gpu_mutex_id[MAX_GPU_THREAD] = {0};

/* Foreign Functions */
void pwork_ctx_init(void);
char *PowCL(char *trytes, int mwm, int index);

int get_mutex_id(int *mutex_id, int env)
{
    int MAX = (env == 1) ? MAX_CPU_THREAD : MAX_GPU_THREAD;
    for (int i = 0; i < MAX; i++) {
        if (mutex_id[i] == 0) {
            mutex_id[i] = 1;
            return i;
        }
    }
    return -1;
}

void dcurl_init(void)
{
    isInitialized = 1;
    pthread_mutex_init(&mtx, NULL);
    sem_init(&notify, 0, 0);
    pwork_ctx_init();
}

void dcurl_entry(char *trytes, int mwm)
{
    static int num_cpu_thread = 0;
    static int num_gpu_thread = 0;
    static int num_waiting_thread = 0;
    int selected_mutex_id = -1;
    int selected_entry = -1;

    pthread_mutex_lock(&mtx);
    if (num_cpu_thread < MAX_CPU_THREAD) {
        num_cpu_thread++;
        /* get mutex number */
        selected_mutex_id = get_mutex_id(cpu_mutex_id, 1);
        selected_entry = 1;
        pthread_mutex_unlock(&mtx);
    } else if (num_gpu_thread < MAX_GPU_THREAD) {
        num_gpu_thread++;
        selected_mutex_id = get_mutex_id(gpu_mutex_id, 2);
        selected_entry = 2;
        pthread_mutex_unlock(&mtx);
    } else {
        num_waiting_thread++;
        pthread_mutex_unlock(&mtx);
        sem_wait(&notify);
        /* get mutex number */
        pthread_mutex_lock(&mtx);
        selected_entry = 1;
        /* get mutex number. If return value is -1, which means cpu queue full */
        if ((selected_mutex_id = get_mutex_id(cpu_mutex_id, 1)) == -1) {
            selected_mutex_id = get_mutex_id(gpu_mutex_id, 2);
            selected_entry = 2;
        }
        pthread_mutex_unlock(&mtx);
    }

    //printf("%s\n", PowC(trytes, mwm, selected_mutex_id));

    switch (selected_entry) {
        case 1:
            printf("%s\n", PowC(trytes, mwm, selected_mutex_id));
            break;
        case 2:
            printf("%s\n", PowCL(trytes, mwm, selected_mutex_id));
            break;
        default:
            printf("error produced\n");
            exit(0);
    }
   
    pthread_mutex_lock(&mtx);
    
    if (selected_entry == 1)
        cpu_mutex_id[selected_mutex_id] = 0;
    else
        gpu_mutex_id[selected_mutex_id] = 0;
    
    if (num_waiting_thread > 0) {
        sem_post(&notify);
        num_waiting_thread--;
    } else {
        if (selected_entry == 1)
            num_cpu_thread--;
        else
            num_gpu_thread--;
    }
    pthread_mutex_unlock(&mtx);
}



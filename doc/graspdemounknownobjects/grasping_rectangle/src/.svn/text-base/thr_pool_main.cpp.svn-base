#include "thr_pool.h"
#include <stdio.h>

static void* func(void* arg) {
    int i;
    for (i=0;i<100000;i++);
    printf("func: %d\n",i);
}

int main() {
    thr_pool_t *pool = thr_pool_create(4, 4, 0, NULL);
    printf("Init\n");
    int ret = thr_pool_queue(pool, func, NULL);
    printf("%d ret\n",ret);
    ret = thr_pool_queue(pool, func, NULL);
    printf("%d ret\n",ret);
    ret = thr_pool_queue(pool, func, NULL);
    printf("%d ret\n",ret);
    thr_pool_wait(pool);
    printf("done!\n");
    thr_pool_destroy(pool);
    return 0;
}

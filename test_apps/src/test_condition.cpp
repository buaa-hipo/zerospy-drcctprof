#include <stdio.h>

void condition_test(int n) {
    int tmp=1;
    int x = n;
    // cmp will set CF=1, so sbb should substract 2, instead of 1
    int b = tmp - 2;
    asm ("movl $1,%0\n\t"
         "subl $2,%0"
         : "=r" (tmp));
    asm ("sbb $1,%0" 
         : "=r" (n)
         : "r" (n));
    printf("x=%d, n=%d, tmp=%d, b=%d\n", x, n, tmp, b);
    if(n==x-1) {
        printf("SBB ERROR: CF is Wrong!\n");
    } else if(n==x-2) {
        printf("SBB PASS!\n");
    } else {
        printf("ERROR: unkown results!\n");
    }
}

int main() {
    condition_test(5);
    return 0;
}

//http://www1.cts.ne.jp/~clab/hsample/Flow/Flow09.html ・ｽ・ｽ・ｽ


#define ABS(a) ((a) < 0 ? - (a) : (a))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define SQUARE(x) ((x) * (x))
#define LIMU(a, b) ((a) > (b) ? (b) : (a))
#define LIML(a, b) ((a) < (b) ? (b) : (a))
#define LIM(a, b, c)  ((a) > (b) ? (b) : ((a) < (c) ? (c) : (a)))

int MOTOR_PI(int inValue, int inKp, int inKi, long *iopBuf);
int MOTOR_LPF(int inValue, int inT, long *iopBuf);


#include<sys/stat.h>
#include<fcntl.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<time.h>

int main(void)
{
  const char* pathname = "/proc/cpuinfo";
  FILE* fd = 0;
  int i = 0;
  struct timespec time_start={0, 0},time_end={0, 0};
  int buff[4] = {0};
  int test = 0;

  if ((fd = fopen(pathname, "r")) < 0) {
    printf("open file error");
    return 0;
  }
  printf("open file successful\n");
  clock_gettime(CLOCK_REALTIME, &time_start);
  for (i = 0; i < 96 * 1024; i ++) {
      fread(buff, sizeof(buff) - 1, 1, fd);
      //memset(&time_end, 0, sizeof(time_end));
      //test ++;
  }
  clock_gettime(CLOCK_REALTIME, &time_end);
  fclose(fd);
  printf("start time %llus,%llu ns\n", 
  (long long unsigned int)time_start.tv_sec, (long long unsigned int)time_start.tv_nsec);
  printf("end   time %llus,%llu ns\n", 
  (long long unsigned int)time_end.tv_sec, (long long unsigned int)time_end.tv_nsec);
  printf("duration:%llus %lluns\n", 
  (long long unsigned int)(time_end.tv_sec-time_start.tv_sec), (long long unsigned int)(time_end.tv_nsec-time_start.tv_nsec));

  return 0;
}
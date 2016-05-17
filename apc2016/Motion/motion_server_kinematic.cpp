#include "motion_kinematic.cpp"
#include <signal.h>

void run_motion_server();

int main(int argc,char** argv)
{
  signal(SIGPIPE,SIG_IGN);
  run_motion_server();
}

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>
LOG_MODULE_REGISTER(thermal_ctrl,LOG_LEVEL_INF);
#define target_temp 25
#define P_max 20
#define k 0.5
#define amb_temp 20
#define dt 0.1 //Time period when the reading are taken (100ms)
#define c_mass 10

#define kp 2.5
#define ki 0.1
#define kd 0.05

#define priority_physics 1
#define priority_pid 2
#define priority_heater 3

struct shared_resource{
 struct k_mutex lock;
 double T_current;
 double T_target;
 double u;
 };
 
 struct shared_resource sys={
 .T_current=amb_temp,
 .T_target=target_temp,
 .u=0
 };

K_MUTEX_DEFINE(sys_lock);
K_MSGQ_DEFINE(actuator_msg,sizeof(double),10,4);  //A queue has been created with 10 slots of storage with each block of sizeof(double) bytes. actuator_msg represents the name of the queue and 4 is how the memory is aligned. 

void physics(void *a, void *b, void *c){
  while(1){
    k_mutex_lock(&sys_lock,K_FOREVER);
    double dq_out=k*(sys.T_current-amb_temp);
    double dq_in=P_max*sys.u;
    double delta_t_c=(double)dt/c_mass;
    sys.T_current=sys.T_current+delta_t_c*(dq_in-dq_out);
    k_mutex_unlock(&sys_lock);
    k_msleep(100);
  }
}
void pid(void *a, void *b, void *c){
  double integral=0;
  double last_error=0;
  while(1){
    k_mutex_lock(&sys_lock,K_FOREVER);
    double error=sys.T_target-sys.T_current;
    double integral=(error*dt)+integral; //THis is done to replace the integration; this gives us the error accumulated over time
    double derivative=(error-last_error)/dt;
    double u=(kp*error)+(ki*integral)+(kd*derivative);
    if (u>1.0) u=1.0;
    if(u<0.0) u=0.0;
    
    sys.u=u;
    last_error=error;
    
    LOG_INF("Temp: %.2fc | Target: %.2fc | Heater PID variable: %.2f",sys.T_current,sys.T_target,sys.u);
    k_mutex_unlock(&sys_lock);
    k_msleep(100);
    
  }
}

static int command_shell(const struct shell *sh, size_t argc, char **argv){
  if(argc!=2){
    shell_error(sh,"Usage: set_temp <value>");
    return -1;
  }
  double value=atof(argv[1]);
  k_mutex_lock(&sys_lock,K_FOREVER);
  sys.T_target=value;
  k_mutex_unlock(&sys_lock);
  shell_print(sh,"Target temperature set to %.2f",value);
  return 0;
}

SHELL_CMD_REGISTER(set_temperature,NULL,"Set temp",command_shell);

K_THREAD_DEFINE(physics_id,1024,physics,NULL,NULL,NULL,priority_physics,0,0);
K_THREAD_DEFINE(pid_id,1024,pid,NULL,NULL,NULL,priority_pid,0,0);

int main(void){
  LOG_INF("Systems started");
  return 0;
}


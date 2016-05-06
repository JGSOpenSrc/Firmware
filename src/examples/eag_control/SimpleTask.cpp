#include"SimpleTask.h"
#include <cstdlib>

// A list of pointers to SimpleTask objects which are registered in the scheduler.
static TaskList *task_list;

int run_job(int, char**);

SimpleTask::~SimpleTask()
{

  if(!this->thread_should_exit){
    this->stop();
  }

  this->join();

  // remove the task from the list, deleting the task node
  task_list->remove_task(this);
}

/*
Registers this task in the nuttx scheduler. This method must be called by the
Task object.

@param priority the priority of the thread (see px4_tasks.h)
@param scheduler the scheduler to use (see px4_tasks.h)
*/
void SimpleTask::schedule(int priority, int scheduler)
{
  // A list of all scheduled tasks
  static TaskList tasks;
  task_list = &tasks;

  // Add the task if its not already in the list (no duplicate names allowed)
  if(task_list->get_task(this->task_name) == NULL){
    task_list->add_task(this);
  }

  char* const* argv = (char* const*)&this->task_name;
  // Schedule the task
  this->pid = px4_task_spawn_cmd(this->task_name,
                                scheduler,
                                priority,
                                this->stack_size,
                                run_job,
                                argv);
}
/*
Callback method used to enter a task's working thread.
The signature of run_job must match that of px4_main_t.

@param argv should contain a singal cstring containing the name of the task
*/
int run_job(int argc, char** argv)
{
    SimpleTask *task = task_list->get_task(argv[0]);

    if(NULL == task){
      PX4_ERR("Task with name %s was not in the task list!", argv[0]);
      return -1;
    }
    // while(1);
    return task->job();
}

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

// Registers the job in the scheduler
void SimpleTask::start()
{
  // Guard clause
  if(this->is_running()) return;

  this->thread_should_exit = false;

  // A list of all scheduled tasks
  static TaskList tasks;
  task_list = &tasks;

  // Add the task if its not already in the list (no duplicate names allowed)
  if(task_list->get_task(_task_name) == NULL){
    task_list->add_task(this);
  }

  char* const* argv = (char* const*)&_task_name;
  // Schedule the task
  pid = px4_task_spawn_cmd(_task_name,
                            _scheduler,
                            _priority,
                            _stack_size,
                            run_job,
                            argv);
}

void SimpleTask::thread_runtime_exception(const char *exception)
{
  PX4_ERR("Runtime exception in %s: %s\n Thread terminated.",
          _task_name, exception);
  thread_should_exit = true;
}
/*
Callback method used to enter a task's working thread.
The signature of run_job must match that of px4_main_t.

@param argv should contain a single cstring containing the name of the task
*/
int run_job(int argc, char** argv)
{
    SimpleTask *task = task_list->get_task(argv[0]);

    if(NULL == task){
      PX4_ERR("Task with name %s was not in the task list!", argv[0]);
      return -1;
    }
    return task->job();
}

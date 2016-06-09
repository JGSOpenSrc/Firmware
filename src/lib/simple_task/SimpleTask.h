#include <px4_config.h>
#include <px4_log.h>
#include <nuttx/sched.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <cstring>
#include <cstdlib>
/* Abstract class for deriving task objects.
   This class uses the functions exported from px4_tasks.h.

   Derived classes must implement job(), which is the entry point to the thread.

   @author Joseph Sullivan <jgs.424112@gmail.com> */
#ifndef SIMPLE_TASK_H
#define SIMPLE_TASK_H

class SimpleTask {

public:

  void start();

  void stop() { if(thread_running) thread_should_exit = true; }

  void join() { while(thread_running); }

  bool is_running() { return thread_running; }

  bool should_exit() { return thread_should_exit; }

  const char* get_name() { return _task_name; }

  int get_pid() { return pid; }

  /* How to implement job():

     job() should set the thread_running flag, execute until the
     thread_should_exit flag is true, and when finished set the threa_running
     flag to false.
  */
  virtual int job() = 0;

private:

  const char* _task_name;

  const int _priority;

  const int _stack_size;

  const int _scheduler;

  int pid;

  bool thread_should_exit;

protected:
  SimpleTask(const char* task_name, int priority, int stack_size, int scheduler) :
  _task_name(task_name),
  _priority(priority),
  _stack_size(stack_size),
  _scheduler(scheduler),
  pid(0),
  thread_should_exit(false),
  thread_running(false)
  {}

  ~SimpleTask();

  bool thread_running;

  void thread_runtime_exception(const char *exception);
};

typedef struct TaskNode{
  SimpleTask *task;
  TaskNode *next;
  TaskNode *prev;
}TaskNode;


/*
A simple list impelementation of TaskNode elements.
This is used to keep track of the scheduled tasks so that task pointers
can be retrieved by the callback method run_job.
*/
class TaskList {
public:
  TaskList()
  {
    head = (TaskNode*)malloc(sizeof(TaskNode));
    head->task = NULL;
    head->next = NULL;
    head->prev = NULL;
  }

  ~TaskList()
  {
    TaskNode *node = head;
    TaskNode *next = head->next;

    free(head);

    while(NULL != next){
      node = next;
      next = node->next;
      free(node);
    }
  }

  // Adds a task to the end of the list.
  void add_task(SimpleTask* task)
  {
      // Null pointer
      if(NULL == task) return ;

      // Task is already in the list
      if(NULL != get_node(task)) return ;

      TaskNode *node = head;

      // Find the end of the list
      while(NULL != node->next) { node = node->next; }

      if(NULL == node->task){
        // Case in which the head does not point to a task
        node->task = task;
      }
      else {
        // Case in which the head does point to a task
        node->next = (TaskNode*)malloc(sizeof(TaskNode));
        node->next->task = task;
        node->next->next = NULL;
        node->next->prev = node;
      }
  }

  // Removes a task and deletes its node
  void remove_task(SimpleTask* task)
  {
    // Null pointer
    if(NULL == task) return;

    // List is empty
    if(NULL == head->task) return;

    // Task is not in the list
    TaskNode* node = get_node(task);
    if(NULL == node) return;

    // heal the list
    // General case - node is somewhere in the middle
    if(NULL != node->next && NULL != node->prev){
      node->next->prev = node->prev;
      node->prev->next = node->next;
    }
    // Node is the head
    else if(NULL != node->next) head = node->next;
    // Node is at the end
    else if(NULL != node->prev) node->prev->next = NULL;

    free(node);
  }

  // Gets the task pointer corresponding to the task name
  SimpleTask *get_task(const char* name)
  {
    // Null pointer
    if(NULL == name) return NULL;

    // List is empty
    if(NULL == head->task) return NULL;

    // head is the right task
    if(0 == strcmp(name, head->task->get_name())){
      return head->task;
    }

    TaskNode* node = head->next;

    // loop over list contents
    while(NULL != node){
      // Found the task
      if(0 == strcmp(name, node->task->get_name())){
        return node->task;
      }
      node = node->next;
    }

    // Task not found
    return NULL;
  }

  // Gets a task node corresponding to the task pointer
  TaskNode *get_node(SimpleTask* task)
  {
    // Null pointer
    if(NULL == task) return NULL;

    // List is empty
    if(NULL == head->task) return NULL;

    // Iterate over the list
    TaskNode* node = head;
    do {
      if(node->task == task) return node;
      node = node->next;
    } while(NULL != node);

    // Task not found
    return NULL;
  }

private:
  TaskNode *head;
};

#endif

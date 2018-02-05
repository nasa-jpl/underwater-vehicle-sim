#ifndef ACTION_EXECUTOR_H
#define ACTION_EXECUTOR_H

#include <memory>

template <class T>
class ActionExecutor
{
public:
	ActionExecutor() {};
	virtual ~ActionExecutor() {}

	virtual bool execute(std::shared_ptr<T> action)=0;
	
	/**
	* Monitors and updates the state of the yoyo action in the ros simulation 
	*/
	virtual void monitor(std::shared_ptr<T> action)=0;

	/**
	* Allows the yoyo action to trigger a replan in the ros simulation 
	*/
	virtual bool triggerReplan(std::shared_ptr<T> action)=0;

	virtual void cancel(std::shared_ptr<T> action)=0;

private:

};
#endif